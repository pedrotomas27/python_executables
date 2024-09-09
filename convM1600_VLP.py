#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pcl_callback(msg):
    custom_cloud = generate_custom_pointcloud(msg)
    pub.publish(custom_cloud)
    rospy.loginfo("Published custom point cloud")

def generate_custom_pointcloud(msg):
    # Precompute the start time from the header
    start_time = msg.header.stamp.to_sec()

    # Read points and process them in bulk using numpy for efficiency
    points = np.array(list(pc2.read_points(msg, skip_nans=False, field_names=("timestampSec", "timestampNsec", "x", "y", "z", "intensity", "ring"))))

    rospy.loginfo(f"Processing {len(points)} points")
    
    # Extract individual fields
    timestamp_secs = points[:, 0]
    timestamp_nsecs = points[:, 1]
    x = points[:, 2]
    y = points[:, 3]
    z = points[:, 4]
    intensity = points[:, 5].astype(np.float32)
    ring = points[:, 6].astype(np.uint16)  # Ensure ring is uint16

    # Calculate elapsed times
    current_times = timestamp_secs + timestamp_nsecs * 1e-9
    elapsed_times = (current_times - start_time).astype(np.float32)

    # Prepare custom points array with correct dtype for each field
    custom_points = np.zeros(points.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), 
                                                     ('intensity', np.float32), ('ring', np.uint16), ('time', np.float32)])
    custom_points['x'] = x
    custom_points['y'] = y
    custom_points['z'] = z
    custom_points['intensity'] = intensity
    custom_points['ring'] = ring
    custom_points['time'] = elapsed_times

    rospy.loginfo(f"Generated {len(custom_points)} custom points")

    # Define the fields for the custom PointCloud2 message
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),  # Use UINT16 for ring
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1)  # Correct offset to 20
    ]

    # Create the custom PointCloud2 message
    header = msg.header
    custom_cloud_msg = pc2.create_cloud(header=header, fields=fields, points=custom_points)

    return custom_cloud_msg

def main():
    global pub
    rospy.init_node("pcl_transformer", anonymous=True)
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=10)
    rospy.Subscriber("/m1600/pcl2", PointCloud2, pcl_callback)

    rospy.spin()  # Let rospy handle the spinning

if __name__ == "__main__":
    main()

