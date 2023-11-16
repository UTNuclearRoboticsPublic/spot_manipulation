# This node is based on the spot_ros and spot_wrapper with ONLY the AsyncTask to get the images
# with the publishers to stream them out to ROS
## Author: Fabian Parra
import argparse
import copy
import functools
import logging
import math
import threading
import time
import typing
from collections import namedtuple
from dataclasses import dataclass, field

import bosdyn.client
import bosdyn.client.auth
import bosdyn.client.util
import bosdyn.geometry
import numpy
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import (ResponseError, RpcError, create_standard_sdk,
                           math_helpers)
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.image import (ImageClient,
                                 UnsupportedPixelFormatRequestedError,
                                 build_image_request)
from bosdyn.client.point_cloud import build_pc_request
from bosdyn.client.robot_command import RobotCommandClient
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import DoubleValue

import actionlib
import rospy
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage

front_image_sources = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "frontleft_depth",
    "frontright_depth",
]
"""List of image sources for front image periodic query"""
side_image_sources = [
    "left_fisheye_image",
    "right_fisheye_image",
    "left_depth",
    "right_depth",
]
"""List of image sources for side image periodic query"""
rear_image_sources = ["back_fisheye_image", "back_depth"]
"""List of image sources for rear image periodic query"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"
"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
point_cloud_sources = ["velodyne-point-cloud"]
"""List of point cloud sources"""
hand_image_sources = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]
"""List of image sources for hand image periodic query"""


# TODO: Missing Hand images
CAMERA_IMAGE_SOURCES = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]
DEPTH_IMAGE_SOURCES = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
]
DEPTH_REGISTERED_IMAGE_SOURCES = [
    "frontleft_depth_in_visual_frame",
    "frontright_depth_in_visual_frame",
    "right_depth_in_visual_frame",
    "left_depth_in_visual_frame",
    "back_depth_in_visual_frame",
]
ImageBundle = namedtuple(
    "ImageBundle", ["frontleft", "frontright", "left", "right", "back"]
)
ImageWithHandBundle = namedtuple(
    "ImageBundle", ["frontleft", "frontright", "left", "right", "back", "hand"]
)

IMAGE_SOURCES_BY_CAMERA = {
    "frontleft": {
        "visual": "frontleft_fisheye_image",
        "depth": "frontleft_depth",
        "depth_registered": "frontleft_depth_in_visual_frame",
    },
    "frontright": {
        "visual": "frontright_fisheye_image",
        "depth": "frontright_depth",
        "depth_registered": "frontright_depth_in_visual_frame",
    },
    "left": {
        "visual": "left_fisheye_image",
        "depth": "left_depth",
        "depth_registered": "left_depth_in_visual_frame",
    },
    "right": {
        "visual": "right_fisheye_image",
        "depth": "right_depth",
        "depth_registered": "right_depth_in_visual_frame",
    },
    "back": {
        "visual": "back_fisheye_image",
        "depth": "back_depth",
        "depth_registered": "back_depth_in_visual_frame",
    },
    "hand": {
        "visual": "hand_color_image",
        "depth": "hand_depth",
        "depth_registered": "hand_depth_in_color_frame",
    },
}

IMAGE_TYPES = {"visual", "depth", "depth_registered"}


@dataclass(frozen=True, eq=True)
class CameraSource:
    camera_name: str
    image_types: typing.List[str]


@dataclass(frozen=True)
class ImageEntry:
    camera_name: str
    image_type: str
    image_response: image_pb2.ImageResponse


def robotToLocalTime(timestamp, robot):
    """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

    Args:
        timestamp: google.protobuf.Timestamp
        robot: Robot handle to use to get the time skew
    Returns:
        google.protobuf.Timestamp
    """

    rtime = Timestamp()

    rtime.seconds = timestamp.seconds - robot.time_sync.endpoint.clock_skew.seconds
    rtime.nanos = timestamp.nanos - robot.time_sync.endpoint.clock_skew.nanos
    if rtime.nanos < 0:
        rtime.nanos = rtime.nanos + 1000000000
        rtime.seconds = rtime.seconds - 1

    # Workaround for timestamps being incomplete
    if rtime.seconds < 0:
        rtime.seconds = 0
        rtime.nanos = 0

    return rtime


class DefaultCameraInfo(CameraInfo):
    """Blank class extending CameraInfo ROS topic that defaults most parameters"""

    def __init__(self):
        super().__init__()
        self.distortion_model = "plumb_bob"

        self.D.append(0)
        self.D.append(0)
        self.D.append(0)
        self.D.append(0)
        self.D.append(0)

        self.K[1] = 0
        self.K[3] = 0
        self.K[6] = 0
        self.K[7] = 0
        self.K[8] = 1

        self.R[0] = 1
        self.R[1] = 0
        self.R[2] = 0
        self.R[3] = 0
        self.R[4] = 1
        self.R[5] = 0
        self.R[6] = 0
        self.R[7] = 0
        self.R[8] = 1

        self.P[1] = 0
        self.P[3] = 0
        self.P[4] = 0
        self.P[7] = 0
        self.P[8] = 0
        self.P[9] = 0
        self.P[10] = 1
        self.P[11] = 0


def getImageMsg(data, spot_cameras_wrapper):
    """Takes the imag and  camera data and populates the necessary ROS messages

    Args:
        data: Image proto
        spot_cameras_wrapper: A SpotWrapper object
    Returns:
        (tuple):
            * Image: message of the image captured
            * CameraInfo: message to define the state and config of the camera that took the image
    """
        max_samples=10, break_on_success=False
    )

    image_msg = Image()
    local_time = spot_cameras_wrapper.robotToLocalTime(data.shot.acquisition_time)
    image_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    image_msg.header.frame_id = data.shot.frame_name_image_sensor
    image_msg.height = data.shot.image.rows
    image_msg.width = data.shot.image.cols

    # Color/greyscale formats.
    if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = True
        image_msg.step = 3 * data.shot.image.cols
        image_msg.data = data.shot.image.data

    # Uncompressed.  Requires pixel_format.
    if data.shot.image.format == image_pb2.Image.FORMAT_RAW:
        # One byte per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            image_msg.encoding = "mono8"
            image_msg.is_bigendian = True
            image_msg.step = data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Three bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Four bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            image_msg.encoding = "rgba8"
            image_msg.is_bigendian = True
            image_msg.step = 4 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Little-endian uint16 z-distance from camera (mm).
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            image_msg.encoding = "16UC1"
            image_msg.is_bigendian = False
            image_msg.step = 2 * data.shot.image.cols
            image_msg.data = data.shot.image.data

    camera_info_msg = DefaultCameraInfo()
    local_time = spot_cameras_wrapper.robotToLocalTime(data.shot.acquisition_time)
    camera_info_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    camera_info_msg.header.frame_id = data.shot.frame_name_image_sensor
    camera_info_msg.height = data.shot.image.rows
    camera_info_msg.width = data.shot.image.cols

    camera_info_msg.K[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.K[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.K[4] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.K[5] = data.source.pinhole.intrinsics.principal_point.y

    camera_info_msg.P[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.P[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.P[5] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.P[6] = data.source.pinhole.intrinsics.principal_point.y

    return image_msg, camera_info_msg


def GetPointCloudMsg(data, spot_cameras_wrapper):
    """Takes the imag and  camera data and populates the necessary ROS messages

    Args:
        data: PointCloud proto (PointCloudResponse)
        spot_cameras_wrapper: A SpotWrapper object
    Returns:
           PointCloud: message of the point cloud (PointCloud2)
    """
    point_cloud_msg = PointCloud2()
    local_time = spot_cameras_wrapper.robotToLocalTime(
        data.point_cloud.source.acquisition_time
    )
    point_cloud_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    point_cloud_msg.header.frame_id = data.point_cloud.source.frame_name_sensor
    if data.point_cloud.encoding == point_cloud_pb2.PointCloud.ENCODING_XYZ_32F:
        point_cloud_msg.height = 1
        point_cloud_msg.width = data.point_cloud.num_points
        point_cloud_msg.fields = []
        for i, ax in enumerate(("x", "y", "z")):
            field = PointField()
            field.name = ax
            field.offset = i * 4
            field.datatype = PointField.FLOAT32
            field.count = 1
            point_cloud_msg.fields.append(field)
        point_cloud_msg.is_bigendian = False
        point_cloud_np = np.frombuffer(data.point_cloud.data, dtype=np.uint8)
        point_cloud_msg.point_step = 12  # float32 XYZ
        point_cloud_msg.row_step = point_cloud_msg.width * point_cloud_msg.point_step
        point_cloud_msg.data = point_cloud_np.tobytes()
        point_cloud_msg.is_dense = True
    else:
        rospy.logwarn("Not supported point cloud data type.")
    return point_cloud_msg


class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__(
            "robot_image_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_image_async(self._image_requests)
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncPointCloudService(AsyncPeriodicQuery):
    """
    Class to get point cloud at regular intervals.  get_point_cloud_from_sources_async query sent to the robot at
    every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, point_cloud_requests):
        super(AsyncPointCloudService, self).__init__(
            "robot_point_cloud_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._point_cloud_requests = point_cloud_requests

    def _start_query(self):
        if self._callback and self._point_cloud_requests:
            callback_future = self._client.get_point_cloud_async(
                self._point_cloud_requests
            )
            callback_future.add_done_callback(self._callback)
            return callback_future


class RateLimitedCall:
    """
    Wrap a function with this class to limit how frequently it can be called within a loop
    """

    def __init__(self, fn, rate_limit):
        """

        Args:
            fn: Function to call
            rate_limit: The function will not be called faster than this rate
        """
        self.fn = fn
        self.min_time_between_calls = 1.0 / rate_limit
        self.last_call = 0

    def __call__(self):
        now_sec = time.time()
        if (now_sec - self.last_call) > self.min_time_between_calls:
            self.fn()
            self.last_call = now_sec


class SpotCamerasROS:
    def __init__(self):
        self.spot_cameras_wrapper = None
        # self.last_tf_msg = TFMessage()

        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB
        self.callbacks["hand_image"] = self.HandImageCB
        self.callbacks["lidar_points"] = self.PointCloudCB
        self.active_camera_tasks = []
        self.camera_pub_to_async_task_mapping = {}

    def FrontImageCB(self, results):
        """Callback for when the Spot Camera gets new front image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_cameras_wrapper.front_images
        if data:
            image_msg0, camera_info_msg0 = getImageMsg(
                data[0], self.spot_cameras_wrapper
            )
            self.frontleft_image_pub.publish(image_msg0)
            self.frontleft_image_info_pub.publish(camera_info_msg0)
            image_msg1, camera_info_msg1 = getImageMsg(
                data[1], self.spot_cameras_wrapper
            )
            self.frontright_image_pub.publish(image_msg1)
            self.frontright_image_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(
                data[2], self.spot_cameras_wrapper
            )
            self.frontleft_depth_pub.publish(image_msg2)
            self.frontleft_depth_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(
                data[3], self.spot_cameras_wrapper
            )
            self.frontright_depth_pub.publish(image_msg3)
            self.frontright_depth_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_cameras_wrapper.side_images
        if data:
            image_msg0, camera_info_msg0 = getImageMsg(
                data[0], self.spot_cameras_wrapper
            )
            self.left_image_pub.publish(image_msg0)
            self.left_image_info_pub.publish(camera_info_msg0)
            image_msg1, camera_info_msg1 = getImageMsg(
                data[1], self.spot_cameras_wrapper
            )
            self.right_image_pub.publish(image_msg1)
            self.right_image_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(
                data[2], self.spot_cameras_wrapper
            )
            self.left_depth_pub.publish(image_msg2)
            self.left_depth_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(
                data[3], self.spot_cameras_wrapper
            )
            self.right_depth_pub.publish(image_msg3)
            self.right_depth_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_cameras_wrapper.rear_images
        if data:
            mage_msg0, camera_info_msg0 = getImageMsg(
                data[0], self.spot_cameras_wrapper
            )
            self.back_image_pub.publish(mage_msg0)
            self.back_image_info_pub.publish(camera_info_msg0)
            mage_msg1, camera_info_msg1 = getImageMsg(
                data[1], self.spot_cameras_wrapper
            )
            self.back_depth_pub.publish(mage_msg1)
            self.back_depth_info_pub.publish(camera_info_msg1)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])

    def HandImageCB(self, results):
        """Callback for when the Spot Wrapper gets new hand image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        time_sync_client = self._robot.ensure_client(TimeSyncClient.default_service_name)
        time_sync_endpoint = TimeSyncEndpoint(time_sync_client)
        did_establish = time_sync_endpoint.establish_timesync(
        data = self.spot_cameras_wrapper.hand_images
        if data:
            mage_msg0, camera_info_msg0 = getImageMsg(
                data[0], self.spot_cameras_wrapper
            )
            self.hand_image_mono_pub.publish(mage_msg0)
            self.hand_image_mono_info_pub.publish(camera_info_msg0)
            mage_msg1, camera_info_msg1 = getImageMsg(
                data[1], self.spot_cameras_wrapper
            )
            self.hand_depth_pub.publish(mage_msg1)
            self.hand_depth_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(
                data[2], self.spot_cameras_wrapper
            )
            self.hand_image_color_pub.publish(image_msg2)
            self.hand_image_color_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(
                data[3], self.spot_cameras_wrapper
            )
            self.hand_depth_in_hand_color_pub.publish(image_msg3)
            self.hand_depth_in_color_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def PointCloudCB(self, results):
        """Callback for when the Spot Wrapper gets new point cloud data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_cameras_wrapper.point_clouds
        if data:
            point_cloud_msg = GetPointCloudMsg(data[0], self.spot_cameras_wrapper)
            self.point_cloud_pub.publish(point_cloud_msg)

            self.populate_lidar_static_transforms(data[0])

    def populate_camera_static_transforms(self, image_data):
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. This is the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
        image_data: Image protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_frames = [
            self.tf_name_vision_odom,
            self.tf_name_kinematic_odom,
            "body",
        ]
        for frame_name in image_data.shot.transforms_snapshot.child_to_parent_edge_map:
            if frame_name in excluded_frames:
                continue
            parent_frame = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            ).parent_frame_name
            existing_transforms = [
                (transform.header.frame_id, transform.child_frame_id)
                for transform in self.sensors_static_transforms
            ]
            if (parent_frame, frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            transform = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            )
            local_time = self.spot_cameras_wrapper.robotToLocalTime(
                image_data.shot.acquisition_time
            )
            tf_time = rospy.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(
                tf_time,
                transform.parent_frame_name,
                frame_name,
                transform.parent_tform_child,
            )
            self.sensors_static_transforms.append(static_tf)
            self.sensors_static_transform_broadcaster.sendTransform(
                self.sensors_static_transforms
            )

    def populate_lidar_static_transforms(self, point_cloud_data):
        """Check data received from one of the point cloud tasks and use the transform snapshot to extract the lidar frame
        transforms. This is the transforms from body->sensor, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
        point_cloud_data: PointCloud protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_frames = [
            self.tf_name_vision_odom,
            self.tf_name_kinematic_odom,
            "body",
        ]
        for (
            frame_name
        ) in (
            point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map
        ):
            if frame_name in excluded_frames:
                continue
            parent_frame = point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            ).parent_frame_name
            existing_transforms = [
                (transform.header.frame_id, transform.child_frame_id)
                for transform in self.sensors_static_transforms
            ]
            if (parent_frame, frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            transform = point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            )
            local_time = self.spot_cameras_wrapper.robotToLocalTime(
                point_cloud_data.point_cloud.source.acquisition_time
            )
            tf_time = rospy.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(
                tf_time,
                transform.parent_frame_name,
                frame_name,
                transform.parent_tform_child,
            )
            self.sensors_static_transforms.append(static_tf)
            self.sensors_static_transform_broadcaster.sendTransform(
                self.sensors_static_transforms
            )

    def check_for_subscriber(self):
        for pub in list(self.camera_pub_to_async_task_mapping.keys()):
            task_name = self.camera_pub_to_async_task_mapping[pub]
            if (
                task_name not in self.active_camera_tasks
                and pub.get_num_connections() > 0
            ):
                self.spot_cameras_wrapper.update_image_tasks(task_name)
                self.active_camera_tasks.append(task_name)
                print(
                    f"Detected subscriber for {task_name} task, adding task to publish"
                )

    def main(self):
        """Main function for the SpotROS class. Gets config from ROS and initializes the wrapper. Holds lease from
        wrapper and updates all async tasks at the ROS rate"""
        rospy.init_node("spot_cameras", anonymous=True)

        self.rates = rospy.get_param("~rates", {})
        if "loop_frequency" in self.rates:
            loop_rate = self.rates["loop_frequency"]
        else:
            loop_rate = 50

        for param, rate in self.rates.items():
            if rate > loop_rate:
                rospy.logwarn(
                    "{} has a rate of {} specified, which is higher than the loop rate of {}. It will not "
                    "be published at the expected frequency".format(
                        param, rate, loop_rate
                    )
                )

        rate = rospy.Rate(loop_rate)
        self.robot_name = rospy.get_param("~robot_name", "spot")
        self.username = rospy.get_param("~username", "default_value")
        self.password = rospy.get_param("~password", "default_value")
        self.hostname = rospy.get_param("~hostname", "default_value")

        self.logger = logging.getLogger("rosout")

        rospy.loginfo("Starting ROS driver for Spot")
        self.spot_cameras_wrapper = SpotCameraWrapper(
            username=self.username,
            password=self.password,
            hostname=self.hostname,
            robot_name=self.robot_name,
            logger=self.logger,
            rates=self.rates,
            callbacks=self.callbacks,
        )

        # Images #
        self.back_image_pub = rospy.Publisher("camera/back/image", Image, queue_size=10)
        self.frontleft_image_pub = rospy.Publisher(
            "camera/frontleft/image", Image, queue_size=10
        )
        self.frontright_image_pub = rospy.Publisher(
            "camera/frontright/image", Image, queue_size=10
        )
        self.left_image_pub = rospy.Publisher("camera/left/image", Image, queue_size=10)
        self.right_image_pub = rospy.Publisher(
            "camera/right/image", Image, queue_size=10
        )
        self.hand_image_mono_pub = rospy.Publisher(
            "camera/hand_mono/image", Image, queue_size=10
        )
        self.hand_image_color_pub = rospy.Publisher(
            "camera/hand_color/image", Image, queue_size=10
        )

        # Depth #
        self.back_depth_pub = rospy.Publisher("depth/back/image", Image, queue_size=10)
        self.frontleft_depth_pub = rospy.Publisher(
            "depth/frontleft/image", Image, queue_size=10
        )
        self.frontright_depth_pub = rospy.Publisher(
            "depth/frontright/image", Image, queue_size=10
        )
        self.left_depth_pub = rospy.Publisher("depth/left/image", Image, queue_size=10)
        self.right_depth_pub = rospy.Publisher(
            "depth/right/image", Image, queue_size=10
        )
        self.hand_depth_pub = rospy.Publisher("depth/hand/image", Image, queue_size=10)
        self.hand_depth_in_hand_color_pub = rospy.Publisher(
            "depth/hand/depth_in_color", Image, queue_size=10
        )
        self.frontleft_depth_in_visual_pub = rospy.Publisher(
            "depth/frontleft/depth_in_visual", Image, queue_size=10
        )
        self.frontright_depth_in_visual_pub = rospy.Publisher(
            "depth/frontright/depth_in_visual", Image, queue_size=10
        )

        # EAP Pointcloud #
        self.point_cloud_pub = rospy.Publisher(
            "lidar/points", PointCloud2, queue_size=10
        )

        # Image Camera Info #
        self.back_image_info_pub = rospy.Publisher(
            "camera/back/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_image_info_pub = rospy.Publisher(
            "camera/frontleft/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_image_info_pub = rospy.Publisher(
            "camera/frontright/camera_info", CameraInfo, queue_size=10
        )
        self.left_image_info_pub = rospy.Publisher(
            "camera/left/camera_info", CameraInfo, queue_size=10
        )
        self.right_image_info_pub = rospy.Publisher(
            "camera/right/camera_info", CameraInfo, queue_size=10
        )
        self.hand_image_mono_info_pub = rospy.Publisher(
            "camera/hand_mono/camera_info", CameraInfo, queue_size=10
        )
        self.hand_image_color_info_pub = rospy.Publisher(
            "camera/hand_color/camera_info", CameraInfo, queue_size=10
        )

        # Depth Camera Info #
        self.back_depth_info_pub = rospy.Publisher(
            "depth/back/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_depth_info_pub = rospy.Publisher(
            "depth/frontleft/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_depth_info_pub = rospy.Publisher(
            "depth/frontright/camera_info", CameraInfo, queue_size=10
        )
        self.left_depth_info_pub = rospy.Publisher(
            "depth/left/camera_info", CameraInfo, queue_size=10
        )
        self.right_depth_info_pub = rospy.Publisher(
            "depth/right/camera_info", CameraInfo, queue_size=10
        )
        self.hand_depth_info_pub = rospy.Publisher(
            "depth/hand/camera_info", CameraInfo, queue_size=10
        )
        self.hand_depth_in_color_info_pub = rospy.Publisher(
            "camera/hand/depth_in_color/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_depth_in_visual_info_pub = rospy.Publisher(
            "depth/frontleft/depth_in_visual/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_depth_in_visual_info_pub = rospy.Publisher(
            "depth/frontright/depth_in_visual/camera_info", CameraInfo, queue_size=10
        )

        self.camera_pub_to_async_task_mapping = {
            self.frontleft_image_pub: "front_image",
            self.frontleft_depth_pub: "front_image",
            self.frontleft_image_info_pub: "front_image",
            self.frontright_image_pub: "front_image",
            self.frontright_depth_pub: "front_image",
            self.frontright_image_info_pub: "front_image",
            self.back_image_pub: "rear_image",
            self.back_depth_pub: "rear_image",
            self.back_image_info_pub: "rear_image",
            self.right_image_pub: "side_image",
            self.right_depth_pub: "side_image",
            self.right_image_info_pub: "side_image",
            self.left_image_pub: "side_image",
            self.left_depth_pub: "side_image",
            self.left_image_info_pub: "side_image",
            self.hand_image_color_pub: "hand_image",
            self.hand_image_mono_pub: "hand_image",
            self.hand_image_mono_info_pub: "hand_image",
            self.hand_depth_pub: "hand_image",
            self.hand_depth_in_hand_color_pub: "hand_image",
        }

        # """Defining a TF publisher manually because of conflicts between Python3 and tf"""
        # self.tf_pub = rospy.Publisher("tf", TFMessage, queue_size=10)
        # self.metrics_pub = rospy.Publisher("status/metrics", Metrics, queue_size=10)

        rate_check_for_subscriber = RateLimitedCall(
            self.check_for_subscriber, self.rates["check_subscribers"]
        )

        rospy.loginfo("Cameras started")
        while not rospy.is_shutdown():
            self.spot_cameras_wrapper.updateTasks()
            rate_check_for_subscriber()
            rate.sleep()


class SpotCameraWrapper:

    SPOT_CLIENT_NAME = "ros_spot"

    def __init__(
        self,
        username: str,
        password: str,
        hostname: str,
        robot_name: str,
        logger: logging.Logger,
        rates: typing.Optional[typing.Dict] = None,
        callbacks: typing.Optional[typing.Dict] = None,
    ):
        """
        Args:
            username: Username for authentication with the robot
            password: Password for authentication with the robot
            hostname: ip address or hostname of the robot
            robot_name: Optional name of the robot
            start_estop: If true, the wrapper will be an estop endpoint
            estop_timeout: Timeout for the estop in seconds. The SDK will check in with the wrapper at a rate of
                           estop_timeout/3 and if there is no communication the robot will execute a gentle stop.
            rates: Dictionary of rates to apply when retrieving various data from the robot # TODO this should be an object to be unambiguous
            callbacks: Dictionary of callbacks which should be called when certain data is retrieved # TODO this should be an object to be unambiguous
            use_take_lease: Use take instead of acquire to get leases. This will forcefully take the lease from any
                            other lease owner.
            get_lease_on_action: If true, attempt to acquire a lease when performing an action which requires a
                                 lease. Otherwise, the user must manually take the lease. This will also attempt to
                                 power on the robot for commands which require it - stand, rollover, self-right.
            continually_try_stand: If the robot expects to be standing and is not, command a stand.  This can result
                                   in strange behavior if you use the wrapper and tablet together.
        """
        self._username = username
        self._password = password
        self._hostname = hostname
        self._robot_name = robot_name

        if robot_name is not None:
            self._frame_prefix = robot_name + "/"
        self._logger = logger
        if rates is None:
            self._rates = {}
        else:
            self._rates = rates
        if callbacks is None:
            self._callbacks = {}
        else:
            self._callbacks = callbacks

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._point_cloud_requests = []
        for source in point_cloud_sources:
            self._point_cloud_requests.append(build_pc_request(source))

        self._hand_image_requests = []
        for source in hand_image_sources:
            self._hand_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._camera_image_requests = []
        for camera_source in CAMERA_IMAGE_SOURCES:
            self._camera_image_requests.append(
                build_image_request(
                    camera_source,
                    image_format=image_pb2.Image.FORMAT_JPEG,
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                    quality_percent=50,
                )
            )

        self._depth_image_requests = []
        for camera_source in DEPTH_IMAGE_SOURCES:
            self._depth_image_requests.append(
                build_image_request(
                    camera_source, pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                )
            )

        self._depth_registered_image_requests = []
        for camera_source in DEPTH_REGISTERED_IMAGE_SOURCES:
            self._depth_registered_image_requests.append(
                build_image_request(
                    camera_source, pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                )
            )

        # try:
        #     self._sdk = create_standard_sdk("image_capture")
        # except Exception as e:
        #     self._logger.error("Error creating SDK object: %s", e)
        #     self._valid = False
        #     return

        # self._logger.info("Initialising robot at {}".format(self._hostname))
        # self._robot = self._sdk.create_robot(self._hostname)

        # bosdyn.client.util.authenticate("user", "b7bih964c5qg")
        # # bosdyn.client.util.authenticate(self._username, self._password)

        self._sdk = bosdyn.client.create_standard_sdk("image_capture")
        self._robot = self._sdk.create_robot(self._hostname)
        self._robot.authenticate(self._username, self._password)

        if self._robot:
            # Clients
            self._logger.info("Creating clients...")
            initialised = False
            while not initialised:
                try:
                    # self._robot_state_client = self._robot.ensure_client(
                    #     RobotStateClient.default_service_name
                    # )
                    # self._world_objects_client = self._robot.ensure_client(
                    #     WorldObjectClient.default_service_name
                    # )
                    self._robot_command_client = self._robot.ensure_client(
                        RobotCommandClient.default_service_name
                    )

                    self._image_client = self._robot.ensure_client(
                        ImageClient.default_service_name
                    )

                    try:
                        self._point_cloud_client = self._robot.ensure_client(
                            VELODYNE_SERVICE_NAME
                        )
                    except Exception as e:
                        self._point_cloud_client = None
                        self._logger.info("No point cloud services are available.")

                    initialised = True
                except Exception as e:
                    sleep_secs = 15
                    self._logger.warning(
                        "Unable to create client service: {}. This usually means the robot hasn't "
                        "finished booting yet. Will wait {} seconds and try again.".format(
                            e, sleep_secs
                        )
                    )
                    time.sleep(sleep_secs)

            # Add hand camera requests
            if self._robot.has_arm():
                # time_sync_client = self._robot.ensure_client(TimeSyncClient.default_service_name)
                # time_sync_endpoint = TimeSyncEndpoint(time_sync_client)
                # did_establish = time_sync_endpoint.establish_timesync(
                self._camera_image_requests.append(
                    build_image_request(
                        "hand_color_image",
                        image_format=image_pb2.Image.FORMAT_JPEG,
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                )
                self._depth_image_requests.append(
                    build_image_request(
                        "hand_depth",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
                    )
                )
                self._depth_registered_image_requests.append(
                    build_image_request(
                        "hand_depth_in_hand_color_frame",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
                    )
                )

            # Build image requests by camera
            self._image_requests_by_camera = {}
            for camera in IMAGE_SOURCES_BY_CAMERA:
                if camera == "hand" and not self._robot.has_arm():
                    continue
                self._image_requests_by_camera[camera] = {}
                image_types = IMAGE_SOURCES_BY_CAMERA[camera]
                for image_type in image_types:
                    if image_type.startswith("depth"):
                        image_format = image_pb2.Image.FORMAT_RAW
                        pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                    else:
                        image_format = image_pb2.Image.FORMAT_JPEG
                        pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8

                    source = IMAGE_SOURCES_BY_CAMERA[camera][image_type]
                    self._image_requests_by_camera[camera][
                        image_type
                    ] = build_image_request(
                        source,
                        image_format=image_format,
                        pixel_format=pixel_format,
                        quality_percent=75,
                    )

            # Async Tasks
            self._async_task_list = []

            self._front_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("front_image", 0.0)),
                self._callbacks.get("front_image", None),
                self._front_image_requests,
            )
            self._side_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("side_image", 0.0)),
                self._callbacks.get("side_image", None),
                self._side_image_requests,
            )
            self._rear_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("rear_image", 0.0)),
                self._callbacks.get("rear_image", None),
                self._rear_image_requests,
            )
            self._hand_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("hand_image", 0.0)),
                self._callbacks.get("hand_image", None),
                self._hand_image_requests,
            )

            robot_tasks = [
                # self._robot_state_task,
                self._front_image_task
            ]

            if self._point_cloud_client:
                self._point_cloud_task = AsyncPointCloudService(
                    self._point_cloud_client,
                    self._logger,
                    max(0.0, self._rates.get("point_cloud", 0.0)),
                    self._callbacks.get("lidar_points", None),
                    self._point_cloud_requests,
                )
                robot_tasks.append(self._point_cloud_task)

            self._async_tasks = AsyncTasks(robot_tasks)

            self.camera_task_name_to_task_mapping = {
                "hand_image": self._hand_image_task,
                "side_image": self._side_image_task,
                "rear_image": self._rear_image_task,
                "front_image": self._front_image_task,
            }

            self._robot_id = None

    @property
    def robot_name(self):
        return self._robot_name

    @property
    def frame_prefix(self):
        return self._frame_prefix

    @property
    def logger(self):
        """Return logger instance of the SpotWrapper"""
        return self._logger

    @property
    def is_valid(self):
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self):
        """Return robot's ID"""
        return self._robot_id

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def hand_images(self):
        """Return latest proto from the _hand_image_task"""
        return self._hand_image_task.proto

    @property
    def point_clouds(self):
        """Return latest proto from the _point_cloud_task"""
        return self._point_cloud_task.proto

    def has_arm(self, timeout=None):
        return self._robot.has_arm(timeout=timeout)

    @property
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def robotToLocalTime(self, timestamp):
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """
        return robotToLocalTime(timestamp, self._robot)

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            print(f"Update tasks failed with error: {str(e)}")

    def update_image_tasks(self, image_name):
        """Adds an async tasks to retrieve images from the specified image source"""

        task_to_add = self.camera_task_name_to_task_mapping[image_name]

        if task_to_add == self._hand_image_task and not self._robot.has_arm():
            self._logger.warning(
                "Robot has no arm, therefore the arm image task can not be added"
            )
            return

        if task_to_add in self._async_tasks._tasks:
            self._logger.warning(
                f"Task {image_name} already in async task list, will not be added again"
            )
            return

        self._async_tasks.add_task(self.camera_task_name_to_task_mapping[image_name])

    def get_frontleft_rgb_image(self):
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontleft_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_frontright_rgb_image(self):
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontright_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_left_rgb_image(self):
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "left_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_right_rgb_image(self):
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "right_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_back_rgb_image(self):
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "back_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_hand_rgb_image(self):
        if not self.has_arm():
            return None
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "hand_color_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_images(
        self, image_requests: typing.List[image_pb2.ImageRequest]
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError as e:
            return None
        if self.has_arm():
            return ImageWithHandBundle(
                frontleft=image_responses[0],
                frontright=image_responses[1],
                left=image_responses[2],
                right=image_responses[3],
                back=image_responses[4],
                hand=image_responses[5],
            )
        else:
            return ImageBundle(
                frontleft=image_responses[0],
                frontright=image_responses[1],
                left=image_responses[2],
                right=image_responses[3],
                back=image_responses[4],
            )

    def get_camera_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        return self.get_images(self._camera_image_requests)

    def get_depth_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        return self.get_images(self._depth_image_requests)

    def get_depth_registered_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        return self.get_images(self._depth_registered_image_requests)

    def get_images_by_cameras(
        self, camera_sources: typing.List[CameraSource]
    ) -> typing.List[ImageEntry]:
        """Calls the GetImage RPC using the image client with requests
        corresponding to the given cameras.

        Args:
           camera_sources: a list of CameraSource objects. There are two
               possibilities for each item in this list. Either it is
               CameraSource(camera='front') or
               CameraSource(camera='front', image_types=['visual', 'depth_registered')

                - If the former is provided, the image requests will include all
                  image types for each specified camera.
                - If the latter is provided, the image requests will be
                  limited to the specified image types for each corresponding
                  camera.

              Note that duplicates of camera names are not allowed.

        Returns:
            a list, where each entry is (camera_name, image_type, image_response)
                e.g. ('frontleft', 'visual', image_response)
        """
        # Build image requests
        image_requests = []
        source_types = []
        cameras_specified = set()
        for item in camera_sources:
            if item.camera_name in cameras_specified:
                self._logger.error(
                    f"Duplicated camera source for camera {item.camera_name}"
                )
                return None
            image_types = item.image_types
            if image_types is None:
                image_types = IMAGE_TYPES
            for image_type in image_types:
                try:
                    image_requests.append(
                        self._image_requests_by_camera[item.camera_name][image_type]
                    )
                except KeyError:
                    self._logger.error(
                        f"Unexpected camera name '{item.camera_name}' or image type '{image_type}'"
                    )
                    return None
                source_types.append((item.camera_name, image_type))
            cameras_specified.add(item.camera_name)

        # Send image requests
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError:
            self._logger.error(
                "UnsupportedPixelFormatRequestedError. "
                "Likely pixel_format is set wrong for some image request"
            )
            return None

        # Return
        result = []
        for i, (camera_name, image_type) in enumerate(source_types):
            result.append(
                ImageEntry(
                    camera_name=camera_name,
                    image_type=image_type,
                    image_response=image_responses[i],
                )
            )
        return result


if __name__ == "__main__":
    SCR = SpotCamerasROS()
    SCR.main()
