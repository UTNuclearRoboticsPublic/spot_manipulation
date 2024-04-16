import open3d as o3d
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Point

def get_points_from_depth_image(depth_image, camera_params, distance_x, distance_y, logger):
    """
    Create a point cloud from a depth image using camera parameters.

    Args:
    - depth_image: 2D array representing the depth image
    - camera_params: Dictionary containing camera parameters including
                     intrinsic_matrix (3x3 array), extrinsic_matrix (4x4 array),
                     and width/height of the image

    Returns:
    - point_cloud: Open3D point cloud object
    """

    # logger.info('get_points_from_depth_image')
    width = camera_params.width
    height = camera_params.height
    intrinsic_matrix = np.array( [camera_params.k[0],            0       , camera_params.k[2], 
                                 0        ,  camera_params.k[4], camera_params.k[5], 
                                 0        ,            0       ,        1          ] )
    # logger.info('camera intrensics set')
    bridge = CvBridge()

    cv_depth_image = bridge.imgmsg_to_cv2(depth_image, '16UC1')

    center_pixel = cv_depth_image[width//2, height//2]
    # Create Open3D Image from depth image
    depth_o3d = o3d.geometry.Image(cv_depth_image)
    # logger.info('got origin point')

    # Create Open3D camera parameters
    intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic(width, height, intrinsic_matrix[0], intrinsic_matrix[4],
                                  intrinsic_matrix[2], intrinsic_matrix[5])
    # logger.info('point cloud created')

    # Create Open3D point cloud
    point_cloud = o3d.geometry.PointCloud.create_from_depth_image(depth=depth_o3d,
                                                                   intrinsic=intrinsics_o3d)
                                                                #    extrinsic=extrinsic_matrix)
    # Get the closest point to our target point
    min_dist = np.inf
    target_point = np.array([distance_x, distance_y])
    for point in np.asarray(point_cloud.points):
        this_point = np.array([point[0], point[1]])
        dist = np.linalg.norm(this_point - target_point)
        if dist < min_dist:
            next_point = point
            min_dist = dist
    # logger.info('got target point')
    return Point(x=center_pixel/1000.0, y=0.0, z=0.0), Point(x=next_point[2], y=next_point[1], z=-next_point[0])

def get_points(depth_image, depth_image_info, distance_x, distance_y, logger):
    # logger.info('in get_points')
    old, new = get_points_from_depth_image(depth_image, depth_image_info, distance_x, distance_y, logger)
    # logger.info('got points')
    # old, new = select_point_from_point_cloud(point_cloud, distance_x, distance_y)
    return old, new

