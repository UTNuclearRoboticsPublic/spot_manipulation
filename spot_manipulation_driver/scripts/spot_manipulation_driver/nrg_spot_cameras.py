#! /usr/bin/env python
##############################################################################
#      Title     : nrf_spot_cameras.py
#      Project   : spot_manipulation_driver
#      Created   : 01/15/2023
#      Author    : 
#      Copyright : Copyright© The University of Texas at Austin, 2023-2030. All
#      rights reserved.
#
#          All files within this directory are subject to the following, unless
#          an alternative license is explicitly included within the text of
#          each file.
#
#          This software and documentation constitute an unpublished work
#          and contain valuable trade secrets and proprietary information
#          belonging to the University. None of the foregoing material may be
#          copied or duplicated or disclosed without the express, written
#          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
#          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
#          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
#          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
#          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
#          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
#          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
#          University be liable for incidental, special, indirect, direct or
#          consequential damages or loss of profits, interruption of business,
#          or related expenses which may arise from use of software or
#          documentation, including but not limited to those resulting from
#          defects in software and/or documentation, or loss or inaccuracy of
#          data of any kind.
##############################################################################


##############################################################################
##                                                                          ##
## Node to get an image publisher using regular Threads.  (CB ~ 3Hz)        ##
##  (Use spot_cameras.py instead)                                           ##
##                                                                          ##
##############################################################################


import argparse
import logging
import sys
import threading
import time

from importlib import reload
from multiprocessing import Process, Queue
from threading import BrokenBarrierError, Thread
from queue import Empty, Full

import actionlib
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String



import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.api.image_pb2 import ImageSource
from bosdyn.client import math_helpers
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.image import ImageClient
import tf2_ros

LOGGER = bosdyn.client.util.get_logger()

# Mapping from visual to depth data
VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE = {
    'frontleft_fisheye_image': 'frontleft_depth_in_visual_frame',
    'frontright_fisheye_image': 'frontright_depth_in_visual_frame'
}

hand_image_sources = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]

# Don't let the queues get too backed up
QUEUE_MAXSIZE = 10

RAW_IMAGES_QUEUE = Queue(QUEUE_MAXSIZE)

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

def _update_thread(async_task):
    while not rospy.is_shutdown():
        async_task.update()
        time.sleep(0.01)





# Taken from spot_detect_and_follow
class AsyncImage(AsyncPeriodicQuery):
    """Grab image."""

    def __init__(self, image_client, image_sources):
        # Period is set to be about 15 FPS
        super(AsyncImage, self).__init__('images', image_client, LOGGER, period_sec=0.067)
        self.image_sources = image_sources

    def _start_query(self):
        return self._client.get_image_from_sources_async(self.image_sources)




class NRGSpotCameras:
    def __init__(self, argv):
        
        parser = argparse.ArgumentParser()
        bosdyn.client.util.add_base_arguments(parser)
        options = parser.parse_args(argv)

        

        # Create robot object with an image client.
        sdk = bosdyn.client.create_standard_sdk('image_capture')
        robot = sdk.create_robot(options.hostname)
        bosdyn.client.util.authenticate("user", "b7bih964c5qg")
        robot.sync_with_directory()
        robot.time_sync.wait_for_sync()

        self.pub = rospy.Publisher('chatter', String, queue_size=10)

        self.hand_image_color_pub = rospy.Publisher("camera/hand_color/image", Image, queue_size=10)
        self.hand_image_color_info_pub = rospy.Publisher("camera/hand_color/camera_info", CameraInfo, queue_size=10)

        # Hand image
        self.hand_image_mono_pub = rospy.Publisher(
            "camera/hand_mono/image", Image, queue_size=10
        )
        # self.hand_image_color_pub = rospy.Publisher(
        #     "camera/hand_color/image", Image, queue_size=10
        # )
        # Camera Info
        self.hand_image_mono_info_pub = rospy.Publisher(
            "camera/hand_mono/camera_info", CameraInfo, queue_size=10
        )
        # self.hand_image_color_info_pub = rospy.Publisher(
        #     "camera/hand_color/camera_info", CameraInfo, queue_size=10
        # )

        # Front Left image
        # self.frontleft_fisheye_image = rospy.Publisher(
        #     "camera/hand_mono/image", Image, queue_size=10
        # )
        # self.hand_image_color_pub = rospy.Publisher(
        #     "camera/hand_color/image", Image, queue_size=10
        # )
        # # Camera Info
        # self.hand_image_mono_info_pub = rospy.Publisher(
        #     "camera/hand_mono/camera_info", CameraInfo, queue_size=10
        # )
        # self.hand_image_color_info_pub = rospy.Publisher(
        #     "camera/hand_color/camera_info", CameraInfo, queue_size=10
        # )



        image_client = robot.ensure_client(ImageClient.default_service_name)
        source_list = self.get_source_list(image_client)
        print("===============")
        print(source_list)
        print("===============")
        image_task = AsyncImage(image_client, source_list)

        task_list = [image_task]
        _async_tasks = AsyncTasks(task_list)

        print("Update Thread")
        update_thread = Thread(target=_update_thread, args=[_async_tasks])
        update_thread.daemon = True
        update_thread.start()

        print("task proto")
        while any(task.proto is None for task in task_list):
            time.sleep(0.1)

        print("image capture")
        sleep_between_capture = 0.5
        # Start image capture process
        image_capture_thread = Process(target=self.capture_images,
                                        args=(image_task, sleep_between_capture),
                                        daemon=True)
        image_capture_thread.start()

        image_pub_thread = threading.Thread(
            target=self.publish_hand_image
        )    
        image_pub_thread.setDaemon(True)
        
        image_pub_thread.start()
                
        self.rate = rospy.Rate(20)

        

    def capture_images(self, image_task, sleep_between_capture):
        """ Captures images and places them on the queue

        Args:
            image_task (AsyncImage): Async task that provides the images response to use
            sleep_between_capture (float): Time to sleep between each image capture
        """
        
        while not rospy.is_shutdown():
            get_im_resp = image_task.proto
            start_time = time.time()
            if not get_im_resp:
                continue
            

            depth_responses = {
                img.source.name: img
                for img in get_im_resp
                if img.source.image_type == ImageSource.IMAGE_TYPE_DEPTH
            }
            entry = {}
            for im_resp in get_im_resp:
                if im_resp.source.image_type == ImageSource.IMAGE_TYPE_VISUAL:
                    source = im_resp.source.name
                    depth_source = VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE[source]
                    depth_image = depth_responses[depth_source]

                    acquisition_time = im_resp.shot.acquisition_time
                    image_time = acquisition_time.seconds + acquisition_time.nanos * 1e-9

                    try:
                        # image = Image.open(io.BytesIO(im_resp.shot.image.data))
                        source = im_resp.source.name                        
                        entry[source] = {
                            'source': source,
                            # 'world_tform_cam': world_tform_cam,
                            # 'world_tform_gpe': world_tform_gpe,
                            'raw_image_time': image_time,
                            # 'cv_image': image,
                            'image': im_resp,
                            'visual_dims': (im_resp.shot.image.cols, im_resp.shot.image.rows),
                            'depth_image': depth_image,
                            'system_cap_time': start_time,
                            'image_queued_time': time.time()
                        }
                    except Exception as exc:  # pylint: disable=broad-except
                        print(f'Exception occurred during image capture {exc}')
            
            try:
                RAW_IMAGES_QUEUE.put_nowait(entry)
            except Full as exc:
                print(f'RAW_IMAGES_QUEUE is full: {exc}')
            time.sleep(0.5)


    def getImageMsg(self,data):
        """Takes the imag and  camera data and populates the necessary ROS messages

        Args:
            data: Image proto
            
        Returns:
            (tuple):
                * Image: message of the image captured
                * CameraInfo: message to define the state and config of the camera that took the image
        """
        image_msg = Image()
        now = rospy.Time.now()
        rospy.loginfo("Current time", now.secs, now.nsecs)
        image_msg.header.stamp = rospy.Time(now.secs, now.nsecs)
        image_msg.header.frame_id = data.shot.frame_name_image_sensor
        image_msg.height = data.shot.image.rows
        image_msg.width = data.shot.image.cols
        print(image_msg.height)
        print(image_msg.width)
        print("format: ", data.shot.image.format)
        print("pixel format: ", data.shot.image.pixel_format)
        print("pixel format: ", data.shot.image.pixel_format)
        print("format: ", data.source.image_formats)

        # Color/greyscale formats.
        # JPEG format
        if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
            print("FORMAT_JPEG")
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Uncompressed.  Requires pixel_format.
        if data.shot.image.format == image_pb2.Image.FORMAT_RAW:
            print("FORMAT_RAW")
            # One byte per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                print("PIXEL_FORMAT_GREYSCALE_U8")
                image_msg.encoding = "mono8"
                image_msg.is_bigendian = True
                image_msg.step = data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Three bytes per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                print("PIXEL_FORMAT_RGB_U8")
                image_msg.encoding = "rgb8"
                image_msg.is_bigendian = True
                image_msg.step = 3 * data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Four bytes per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                print("PIXEL_FORMAT_RGBA_U8")
                image_msg.encoding = "rgba8"
                image_msg.is_bigendian = True
                image_msg.step = 4 * data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Little-endian uint16 z-distance from camera (mm).
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                print("PIXEL_FORMAT_DEPTH_U16")
                image_msg.encoding = "16UC1"
                image_msg.is_bigendian = False
                image_msg.step = 2 * data.shot.image.cols
                image_msg.data = data.shot.image.data

        camera_info_msg = DefaultCameraInfo()
        now = rospy.Time.now()
        camera_info_msg.header.stamp = rospy.Time(now.secs, now.nsecs)
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


    def GetPointCloudMsg(self, data, spot_wrapper):
        """Takes the imag and  camera data and populates the necessary ROS messages

        Args:
            data: PointCloud proto (PointCloudResponse)
            spot_wrapper: A SpotWrapper object
        Returns:
            PointCloud: message of the point cloud (PointCloud2)
        """
        point_cloud_msg = PointCloud2()
        local_time = spot_wrapper.robotToLocalTime(data.point_cloud.source.acquisition_time)
        now = rospy.Time.now()
        point_cloud_msg.header.stamp = rospy.Time(now.secs, now.nsecs)
        # point_cloud_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
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


    # Taken from spot_wrapper.py
    # class AsyncImageService(AsyncPeriodicQuery):
    #     """Grab camera images from the robot."""

    #     def __init__(self, client, logger, rate, callback, image_requests):
    #         super(AsyncImageService, self).__init__(
    #             "robot_image_service", client, logger, period_sec=1.0 / max(rate, 1.0)
    #         )
    #         self._callback = None
    #         if rate > 0.0:
    #             self._callback = callback
    #         self._image_requests = image_requests

    #     def _start_query(self):
    #         if self._callback:
    #             callback_future = self._client.get_image_async(self._image_requests)
    #             callback_future.add_done_callback(self._callback)
    #             return callback_future


    def get_source_list(self,image_client):
        """Gets a list of image sources and filters based on config dictionary

        Args:
            image_client: Instantiated image client
        """

        # We are using only the visual images with their corresponding depth sensors
        sources = image_client.list_image_sources()
        source_list = []
        print("Inside function ====")
        print(sources)
        print("Inside function ====")
        for source in sources:
            if source.image_type == ImageSource.IMAGE_TYPE_VISUAL:
                # only append if sensor has corresponding depth sensor
                if source.name in VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE:
                    source_list.append(source.name)
                    source_list.append(VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE[source.name])


        # for source in sources:
        #     source_list.append(source.name)


        return source_list

    def publish_hand_image(self):        
        # self.image_msg1 = Image()
        # mage_msg0, camera_info_msg0 = self.getImageMsg(image)

        # image_msg1.header.frame_id = "Test"
        # image_msg1.height = image.shot.image.rows
        # image_msg1.width = image.shot.image.cols
        # image_msg1.encoding = "rgb8"
        # image_msg1.is_bigendian = True
        # image_msg1.step = 3 * image.shot.image.cols
        # image_msg1.data = image.shot.image.data
        
        while not rospy.is_shutdown():
            try:
                entry = RAW_IMAGES_QUEUE.get_nowait()
            except Empty:
                time.sleep(0.1)
                continue
            for _, capture in entry.items():
                if capture['source'] == 'frontleft_fisheye_image':
                    print(capture['source'])
                    image = capture['image']

                # image_msg1 = Image()
                    mage_msg0, camera_info_msg0 = self.getImageMsg(image)

                # image_msg1.header.frame_id = "Test"
                # image_msg1.height = image.shot.image.rows
                # image_msg1.width = image.shot.image.cols
                # image_msg1.encoding = "rgb8"
                # image_msg1.is_bigendian = True
                # image_msg1.step = 3 * image.shot.image.cols
                # image_msg1.data = image.shot.image.data
                    self.hand_image_color_pub.publish(mage_msg0)
                    self.hand_image_color_info_pub.publish(camera_info_msg0)
            


    

            
        # self.hand_image_color_info_pub.publish(camera_info_msg0)
        # 


    

def main(argv):
    rospy.loginfo("node")
    nrg_spot_cameras = NRGSpotCameras(argv)
        
    

    


    while not rospy.is_shutdown():
        rospy.spin()

    
    # hand_image_thread.join()


    
    


if __name__ == "__main__":
    rospy.init_node("nrg_spot_cameras")
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
