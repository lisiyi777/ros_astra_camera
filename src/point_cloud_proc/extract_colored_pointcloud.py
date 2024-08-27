#!/usr/bin/env python

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl
import os
import struct
from pathlib import Path
import argparse

def extract_pointcloud_from_bag(bag_file, output_dir):
    bag = rosbag.Bag(bag_file, 'r')
    for topic, msg, t in bag.read_messages(topics=['/camera/depth_registered/points']):
        print(f"Topic: {topic}, Message Type: {type(msg)}")
        pc = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        pcl_array = []
        for p in pc:
            # Extract RGB components
            x, y, z = p[:3]
            rgb = p[3]
            # Extract RGB as a packed float, then unpack it
            rgb = struct.unpack('I', struct.pack('f', p[3]))[0]
            r = (rgb >> 16) & 0x0000FF
            g = (rgb >> 8) & 0x0000FF
            b = (rgb) & 0x0000FF
            packed_rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

            pcl_array.append([x, y, z, packed_rgb])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(pcl_array)
        timestamp = t.to_nsec()
        pcd_filename = os.path.join(output_dir, f"pointcloud_{timestamp}.pcd")
        pcl.save(pcl_data, pcd_filename)
        rospy.loginfo(f"Saved {pcd_filename}")

    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Iterate over .feather files in a result zip file."
    )
    parser.add_argument("bag_name", help="the name of bag file")
    parser.add_argument(
        "output_path", 
        type=Path, 
        nargs='?',
        default=Path("/project/point_cloud"), 
        help="Path to save the results"
    )
    args = parser.parse_args()


    output_dir = args.output_path.joinpath(args.bag_name)

    base_dir = Path("/project")
    bag_path = base_dir / f"{args.bag_name}.bag"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    extract_pointcloud_from_bag(bag_path, output_dir)

