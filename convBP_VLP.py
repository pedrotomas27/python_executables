#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pcl_callback(msg):
    custom_cloud = generate_custom_pointcloud(msg)
    pub.publish(custom_cloud)

def generate_custom_pointcloud(msg):
    # Read points from the incoming PointCloud2 message
    custom_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring", "timestamp")))

    # Extract individual fields from the custom_points list
    x = np.array(custom_points)[:, 0].astype(np.float32)
    y = np.array(custom_points)[:, 1].astype(np.float32)
    z = np.array(custom_points)[:, 2].astype(np.float32)
    intensity = np.array(custom_points)[:, 3].astype(np.float32)
    ring = np.array(custom_points)[:, 4].astype(np.uint16)
    timestamp = np.array(custom_points)[:, 5].astype(np.float32)

    # Calculate elapsed time from the message timestamp
    start_time = msg.header.stamp.to_sec()
    elapsed_times = (timestamp - start_time).astype(np.float32)

    # Prepare custom points array with correct dtype for each field
    custom_points_np = np.zeros(len(custom_points), dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('intensity', np.float32), ('ring', np.uint16), ('time', np.float32)
    ])
    custom_points_np['x'] = x
    custom_points_np['y'] = y
    custom_points_np['z'] = z
    custom_points_np['intensity'] = intensity
    custom_points_np['ring'] = ring
    custom_points_np['time'] = elapsed_times

    # Define the fields for the custom PointCloud2 message
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1)
    ]

    # Create the custom PointCloud2 message
    header = msg.header  # Use the header from the original message
    custom_cloud_msg = pc2.create_cloud(header=header, fields=fields, points=list(custom_points_np))

    return custom_cloud_msg

def main():
    global pub
    rospy.init_node("pcl_transformer", anonymous=True)
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=20)
    rospy.Subscriber("rslidar_points", PointCloud2, pcl_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
