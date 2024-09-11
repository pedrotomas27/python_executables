#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pcl_callback(msg):
    custom_cloud = generate_custom_pointcloud(msg)
    pub.publish(custom_cloud)

def generate_custom_pointcloud(msg):
    custom_points = []
    for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring", "time")):
        intensity = float(point[3])
        ring = np.uint16(point[4])
        custom_points.append([point[0], point[1], -point[2], intensity, ring])

    # Define the fields for the custom PointCloud2 message
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    ]

    # Create the custom PointCloud2 message
    header = msg.header
    custom_cloud_msg = pc2.create_cloud(header=header, fields=fields, points=custom_points)

    return custom_cloud_msg

def main():
    global pub
    rospy.init_node("pcl_transformer", anonymous=True)
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=10)
    rospy.Subscriber("/livox/points", PointCloud2, pcl_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
