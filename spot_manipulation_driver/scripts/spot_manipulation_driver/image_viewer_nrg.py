# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).


##############################################################################
##                                                                          ##
## Modified version of """Simple image display example."""                  ##
## to publish image to a ROS topic                                          ##
##                                                                          ##
##############################################################################


import argparse
import logging
import sys
import time
from importlib import reload

import rospy
import numpy as np
from scipy import ndimage
from sensor_msgs.msg import Image, CameraInfo

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.time_sync import TimedOutError

_LOGGER = logging.getLogger(__name__)

VALUE_FOR_Q_KEYSTROKE = 113
VALUE_FOR_ESC_KEYSTROKE = 27

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180
}


hand_image_color_pub = rospy.Publisher(
        "test/camera/hand_color/image", Image, queue_size=10
    )

def publish_image(image):
    # num_channels = 1  # Assume a default of 1 byte encodings.
    image_msg1 = Image()
    if image:                
    #     mage_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
    #     self.hand_image_mono_pub.publish(mage_msg0)
    #     self.hand_image_mono_info_pub.publish(camera_info_msg0)

    #     image_msg2, camera_info_msg2 = getImageMsg(data[1], self.spot_wrapper)
        image_msg1.header.frame_id = "Test"
        image_msg1.height = image.shot.image.rows
        image_msg1.width = image.shot.image.cols
        image_msg1.encoding = "rgb8"
        image_msg1.is_bigendian = True
        print("cols: ", image.shot.image.cols)
        image_msg1.step = 3 * image.shot.image.cols
        image_msg1.data = image.shot.image.data
        hand_image_color_pub.publish(image_msg1)
    #     self.hand_image_color_info_pub.publish(camera_info_msg1)
    # self.rate.sleep()
    

def reset_image_client(robot):
    """Recreate the ImageClient from the robot object."""
    del robot.service_clients_by_name['image']
    del robot.channels_by_authority['api.spot.robot']
    return robot.ensure_client('image')


def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--image-sources', help='Get image from source(s)', action='append')
    # parser.add_argument('--image-service', help='Name of the image service to query.',
    #                     default=ImageClient.default_service_name)
    # parser.add_argument('-j', '--jpeg-quality-percent', help='JPEG quality percentage (0-100)',
    #                     type=int, default=50)
    # parser.add_argument('-c', '--capture-delay', help='Time [ms] to wait before the next capture',
    #                     type=int, default=100)
    # parser.add_argument('-r', '--resize-ratio', help='Fraction to resize the image', type=float,
    #                     default=1)
    # parser.add_argument(
    #     '--disable-full-screen',
    #     help='A single image source gets displayed full screen by default. This flag disables that.',
    #     action='store_true')
    # parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
    #                     action='store_true')
    options = parser.parse_args(argv)

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_capture')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync()

    # Images #
    # self.back_image_pub = rospy.Publisher("camera/back/image", Image, queue_size=10)
    # self.frontleft_image_pub = rospy.Publisher(
    #     "camera/frontleft/image", Image, queue_size=10
    # )
    # self.frontright_image_pub = rospy.Publisher(
    #     "camera/frontright/image", Image, queue_size=10
    # )
    # self.left_image_pub = rospy.Publisher("camera/left/image", Image, queue_size=10)
    # self.right_image_pub = rospy.Publisher(
    #     "camera/right/image", Image, queue_size=10
    # )
    
    hand_image_mono_pub = rospy.Publisher(
        "test/camera/hand_mono/image", Image, queue_size=10
    )
    # hand_image_color_pub = rospy.Publisher(
    #     "camera/hand_color/image", Image, queue_size=10
    # )

    # Depth #
    # self.back_depth_pub = rospy.Publisher("depth/back/image", Image, queue_size=10)
    # self.frontleft_depth_pub = rospy.Publisher(
    #     "depth/frontleft/image", Image, queue_size=10
    # )
    # self.frontright_depth_pub = rospy.Publisher(
    #     "depth/frontright/image", Image, queue_size=10
    # )
    # self.left_depth_pub = rospy.Publisher("depth/left/image", Image, queue_size=10)
    # self.right_depth_pub = rospy.Publisher(
    #     "depth/right/image", Image, queue_size=10
    # )
    hand_depth_pub = rospy.Publisher("depth/hand/image", Image, queue_size=10)
    hand_depth_in_hand_color_pub = rospy.Publisher(
        "test/depth/hand/depth_in_color", Image, queue_size=10
    )
    # self.frontleft_depth_in_visual_pub = rospy.Publisher(
    #     "depth/frontleft/depth_in_visual", Image, queue_size=10
    # )
    # self.frontright_depth_in_visual_pub = rospy.Publisher(
    #     "depth/frontright/depth_in_visual", Image, queue_size=10
    # )

    # EAP Pointcloud #
    # self.point_cloud_pub = rospy.Publisher(
    #     "lidar/points", PointCloud2, queue_size=10
    # )

    # Image Camera Info #
    # self.back_image_info_pub = rospy.Publisher(
    #     "camera/back/camera_info", CameraInfo, queue_size=10
    # )
    # self.frontleft_image_info_pub = rospy.Publisher(
    #     "camera/frontleft/camera_info", CameraInfo, queue_size=10
    # )
    # self.frontright_image_info_pub = rospy.Publisher(
    #     "camera/frontright/camera_info", CameraInfo, queue_size=10
    # )
    # self.left_image_info_pub = rospy.Publisher(
    #     "camera/left/camera_info", CameraInfo, queue_size=10
    # )
    # self.right_image_info_pub = rospy.Publisher(
    #     "camera/right/camera_info", CameraInfo, queue_size=10
    # )
    hand_image_mono_info_pub = rospy.Publisher(
        "test/camera/hand_mono/camera_info", CameraInfo, queue_size=10
    )
    hand_image_color_info_pub = rospy.Publisher(
        "test/camera/hand_color/camera_info", CameraInfo, queue_size=10
    )

    # Depth Camera Info #
    # self.back_depth_info_pub = rospy.Publisher(
    #     "depth/back/camera_info", CameraInfo, queue_size=10
    # )
    # self.frontleft_depth_info_pub = rospy.Publisher(
    #     "depth/frontleft/camera_info", CameraInfo, queue_size=10
    # )
    # self.frontright_depth_info_pub = rospy.Publisher(
    #     "depth/frontright/camera_info", CameraInfo, queue_size=10
    # )
    # self.left_depth_info_pub = rospy.Publisher(
    #     "depth/left/camera_info", CameraInfo, queue_size=10
    # )
    # self.right_depth_info_pub = rospy.Publisher(
    #     "depth/right/camera_info", CameraInfo, queue_size=10
    # )
    hand_depth_info_pub = rospy.Publisher(
        "test/depth/hand/camera_info", CameraInfo, queue_size=10
    )
    hand_depth_in_color_info_pub = rospy.Publisher(
        "test/camera/hand/depth_in_color/camera_info", CameraInfo, queue_size=10
    )
    # self.frontleft_depth_in_visual_info_pub = rospy.Publisher(
    #     "depth/frontleft/depth_in_visual/camera_info", CameraInfo, queue_size=10
    # )
    # self.frontright_depth_in_visual_info_pub = rospy.Publisher(
    #     "depth/frontright/depth_in_visual/camera_info", CameraInfo, queue_size=10
    # )

    image_client = robot.ensure_client(ImageClient.default_service_name)

    # requests = [
    #     build_image_request(source, quality_percent=options.jpeg_quality_percent,
    #                         resize_ratio=options.resize_ratio) for source in options.image_sources
    # ]


    hand_image_sources = [
        "hand_image",
        # "hand_depth",
        "hand_color_image",
        # "hand_depth_in_hand_color_frame",
    ]
    
    """List of image sources for hand image periodic query"""

    _hand_image_requests = []
    for source in hand_image_sources:
        _hand_image_requests.append(
            build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
        )
    t1 = time.time()
    image_count = 0
    while not rospy.is_shutdown():
        # print("while")
        try:
            images_future = image_client.get_image_async(_hand_image_requests, timeout=0.5)
            while not images_future.done() and not rospy.is_shutdown():
                rospy.sleep(0.01)
                
            images = images_future.result()
        except TimedOutError as time_err:
            if timeout_count_before_reset == 5:
                # To attempt to handle bad comms and continue the live image stream, try recreating the
                # image client after having an RPC timeout 5 times.
                _LOGGER.info('Resetting image client after 5+ timeout errors.')
                image_client = reset_image_client(robot)
                timeout_count_before_reset = 0
            else:
                timeout_count_before_reset += 1
        except Exception as err:
            _LOGGER.warning(err)
            continue
        
        print("len of images: " , len(images))
        publish_image(images[1])
        # for i in range(len(images)):
        #     publish_image(images[i])

        image_count += 1
        print(f'Mean image retrieval rate: {image_count/(time.time() - t1)}Hz')

        # rospy.spin()


    


if __name__ == '__main__':
    rospy.init_node("image_viewer_nrg")
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
