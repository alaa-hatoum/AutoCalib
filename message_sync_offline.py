import os
import sys
import shutil
import argparse

import warnings
warnings.filterwarnings("ignore")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters  import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointField, PointCloud2, Image

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import open3d as o3d
import struct
import math
import yaml



class BagReaderNode(Node):
    def __init__(self, image_save_dir, pointcloud_save_dir):
        super().__init__('bag_reader_node')
        self.image_sub = Subscriber(self, Image, '/camera/center_wide/brg',qos_profile = qos_profile_sensor_data)
        self.pointcloud_sub = Subscriber(self, PointCloud2, '/lidar/center/bottom')
        self.synchronizer = ApproximateTimeSynchronizer([self.image_sub, self.pointcloud_sub], 30, slop=0.01)
        self.synchronizer.registerCallback(self.synchronized_callback)

        self.image_save_dir = image_save_dir
        self.pointcloud_save_dir = pointcloud_save_dir
        self.counter = 1
        self.cv_bridge = CvBridge()

    def synchronized_callback(self, image_msg, pointcloud_msg):
        # Process the synchronized camera and lidar data here
        self.get_logger().info("Synchronized camera and lidar data received")

        pcd_as_numpy_array = np.array(list(read_points(pointcloud_msg)))[:, :3]
        o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array))

        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_time = image_msg.header.stamp.sec+(image_msg.header.stamp.nanosec*(10**-9))
        pcl_time = pointcloud_msg.header.stamp.sec+(pointcloud_msg.header.stamp.nanosec*(10**-9))
        
        pcd_save_path = os.path.join(self.pointcloud_save_dir, f'{self.counter:04d}.pcd')
        o3d.io.write_point_cloud(pcd_save_path, o3d_pcd)
        print(pcd_save_path)
        
        img_save_path = os.path.join(self.image_save_dir, f'{self.counter:04d}.jpg')
        print(img_save_path)
        cv2.imwrite(img_save_path, cv_image)
        self.get_logger().info(f'Saved point cloud to {pcd_save_path}')

        self.counter += 1

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length
    
    return fmt
def main():
    rclpy.init()
    image_save_dir = r'/home/alaa/AutoCalib/calibration-002/images_center_wide'  # Specify the directory to save images
    pointcloud_save_dir = r'/home/alaa/AutoCalib/calibration-002/point_clouds_center_bottom'  # Specify the directory to save point clouds
    node = BagReaderNode(image_save_dir, pointcloud_save_dir)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    _DATATYPES = {}
    _DATATYPES[PointField.INT8]    = ('b', 1)
    _DATATYPES[PointField.UINT8]   = ('B', 1)
    _DATATYPES[PointField.INT16]   = ('h', 2)
    _DATATYPES[PointField.UINT16]  = ('H', 2)
    _DATATYPES[PointField.INT32]   = ('i', 4)
    _DATATYPES[PointField.UINT32]  = ('I', 4)
    _DATATYPES[PointField.FLOAT32] = ('f', 4)
    _DATATYPES[PointField.FLOAT64] = ('d', 8)
    main()
