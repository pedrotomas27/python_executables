#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from collections import deque
import time

class LiDARRepublisher:
    def __init__(self):
        rospy.init_node('lidar_republisher', anonymous=True)
        
        # Parameters
        self.lidar_topic = rospy.get_param("~lidar_topic", "/lidar_0/m1600/pcl2")
        self.publish_topic = rospy.get_param("~publish_topic", "/republished_lidar")
        self.publish_rate = rospy.get_param("~publish_rate", 20)  # Hz
        self.interpolate_rate = rospy.get_param("~interpolate_rate", 10)  # Hz
        self.timeout = rospy.get_param("~timeout", 0.1)  # Time to wait before starting interpolation (seconds)
        
        # Subscribers and publishers
        rospy.loginfo(f"Subscribing to {self.lidar_topic}")
        self.sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        rospy.loginfo(f"Publishing to {self.publish_topic}")
        self.pub = rospy.Publisher(self.publish_topic, PointCloud2, queue_size=10)
        
        # State
        self.last_msg = None
        self.previous_msg = None
        self.last_publish_time = rospy.Time.now()
        self.last_received_time = rospy.Time.now()
        self.interpolate_start_time = None
        self.is_interpolating = False
        self.rate = rospy.Rate(self.publish_rate)
        self.interpolate_rate = rospy.Rate(self.interpolate_rate)
        
        # Buffer to store the latest messages for interpolation
        self.msg_buffer = deque(maxlen=2)
    
    def lidar_callback(self, msg):
        rospy.loginfo(f"Received message on {self.lidar_topic}")
        self.last_msg = msg
        self.last_received_time = rospy.Time.now()
        self.last_publish_time = rospy.Time.now()
        self.is_interpolating = False  # Stop interpolation when a new message is received

        if len(self.msg_buffer) == 5:
            self.msg_buffer.popleft()
        self.msg_buffer.append(msg)
    
    def interpolate_and_publish(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            time_since_last_message = (current_time - self.last_received_time).to_sec()
            time_since_last_publish = (current_time - self.last_publish_time).to_sec()
            interpolation_needed = time_since_last_message > self.timeout

            if self.last_msg is None:
                rospy.logwarn("No LiDAR messages received yet.")
            elif interpolation_needed:
                # Start interpolation if we haven't already started
                if not self.is_interpolating:
                    rospy.loginfo("Starting interpolation due to lack of new messages.")
                    self.interpolate_start_time = current_time
                    self.is_interpolating = True
                
                # Perform interpolation
                if len(self.msg_buffer) == 5:
                    previous_msg, current_msg = self.msg_buffer
                    alpha = (current_time - self.interpolate_start_time).to_sec() / (1.0 / self.interpolate_rate.sleep_dur.to_sec())
                    if alpha > 1.0:
                        alpha = 1.0  # Cap alpha to 1.0
                    interpolated_msg = self.interpolate(previous_msg, current_msg, alpha)
                    interpolated_msg.header.stamp = rospy.Time.now()  # Update the timestamp
                    self.pub.publish(interpolated_msg)
                else:
                    rospy.logwarn("Insufficient messages in buffer for interpolation.")
            else:
                # Publish the last received message
                rospy.loginfo("Publishing the last received message.")
                if self.last_msg:
                    self.last_msg.header.stamp = rospy.Time.now()  # Update the timestamp
                    self.pub.publish(self.last_msg)
            
            self.last_publish_time = rospy.Time.now()
            self.rate.sleep()
    
    def interpolate(self, msg1, msg2, alpha):
        # Extract point cloud data from both messages
        pc1 = pc2.read_points(msg1, field_names=("timestampSec", "timestampNsec", "x", "y", "z", "intensity", "ring"), skip_nans=True)
        pc2_data = pc2.read_points(msg2, field_names=("timestampSec", "timestampNsec", "x", "y", "z", "intensity", "ring"), skip_nans=True)
        
        # Convert to numpy arrays for interpolation
        points1 = np.array(list(pc1))
        points2 = np.array(list(pc2_data))
        
        # Check if point clouds have the same size
        if points1.shape[0] != points2.shape[0]:
            rospy.logwarn("Point clouds have different sizes, cannot interpolate.")
            return msg1
        
        # Perform linear interpolation
        interpolated_points = (1 - alpha) * points1[:, 2:5] + alpha * points2[:, 2:5]
        interpolated_intensity = (1 - alpha) * points1[:, 5] + alpha * points2[:, 5]
        interpolated_ring = (1 - alpha) * points1[:, 6] + alpha * points2[:, 6]
        
        # Interpolate timestamps
        timestampSec1 = msg1.fields[0].offset
        timestampSec2 = msg2.fields[0].offset
        timestampNsec1 = msg1.fields[1].offset
        timestampNsec2 = msg2.fields[1].offset
        interpolated_timestampSec = int((1 - alpha) * timestampSec1 + alpha * timestampSec2)
        interpolated_timestampNsec = int((1 - alpha) * timestampNsec1 + alpha * timestampNsec2)
        
        # Create new PointCloud2 message with interpolated data
        header = msg1.header
        fields = [
            PointField(name="timestampSec", offset=0, datatype=PointField.UINT32, count=1),
            PointField(name="timestampNsec", offset=4, datatype=PointField.UINT32, count=1),
            PointField(name="x", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=20, datatype=PointField.UINT16, count=1),
            PointField(name="ring", offset=22, datatype=PointField.UINT16, count=1)
        ]
        point_cloud_data = np.hstack((interpolated_points, interpolated_intensity[:, np.newaxis], interpolated_ring[:, np.newaxis]))
        cloud = pc2.create_cloud(header, fields, point_cloud_data.tolist())
        
        # Update timestamps
        cloud.header.stamp = rospy.Time.from_sec(interpolated_timestampSec + interpolated_timestampNsec * 1e-9)
        
        return cloud

if __name__ == "__main__":
    try:
        republisher = LiDARRepublisher()
        republisher.interpolate_and_publish()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Shutting down.")

