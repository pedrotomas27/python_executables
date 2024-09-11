#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Initialize the last position and timestamp
last_position = None
last_timestamp = None

# Define thresholds for filtering
TIME_THRESHOLD = 1.5  # seconds, a bit more than 1 Hz to allow some flexibility
XY_THRESHOLD = 1.5  # meters for x and y, slightly above the maximum walking speed per second
Z_THRESHOLD = 1  # meters for z

# Initialize the status
gps_status = -1

def gps_fix_callback(msg):
    global gps_status
    gps_status = msg.status.status

def gps_pose_callback(msg):
    global last_position, last_timestamp, gps_status

    # Only process the message if the GPS status is 1 or 2
    if gps_status not in [2]:
        return

    # Print the status message
    if gps_status == 1:
        rospy.loginfo("FLOAT value")
    elif gps_status == 2:
        rospy.loginfo("FIX value")

    current_position = msg.pose.pose.position
    current_timestamp = msg.header.stamp

    # Check if we have a previous position and timestamp
    if last_position is not None and last_timestamp is not None:
        # Calculate time difference
        time_diff = (current_timestamp - last_timestamp).to_sec()

        # Calculate distance differences
        dx = abs(current_position.x - last_position.x)
        dy = abs(current_position.y - last_position.y)
        dz = abs(current_position.z - last_position.z)

        # Log the differences for debugging
        rospy.loginfo(f"time_diff: {time_diff}, dx: {dx}, dy: {dy}, dz: {dz}")

        # If the time difference is small and any of the distance differences exceed the thresholds, filter it out
        if time_diff < TIME_THRESHOLD and (dx > XY_THRESHOLD or dy > XY_THRESHOLD or dz > Z_THRESHOLD):
            rospy.loginfo(f"Filtered out point with position: ({current_position.x}, {current_position.y}, {current_position.z})")
            return

    # Update the last position and timestamp
    last_position = current_position
    last_timestamp = current_timestamp

    # Create a new Odometry message
    odom_msg = Odometry()

    # Copy header from gps_pose to odom
    odom_msg.header = msg.header

    # Copy pose information
    odom_msg.pose.pose.position.x = current_position.x
    odom_msg.pose.pose.position.y = current_position.y
    odom_msg.pose.pose.position.z = current_position.z
    odom_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
    odom_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
    odom_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
    odom_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w

    # Copy covariance matrix
    odom_msg.pose.covariance = msg.pose.covariance

    # Publish the transformed message to /gt_poses
    odom_pub.publish(odom_msg)

def gps_pose_to_odom_listener():
    rospy.init_node('gps_pose_to_odom_listener', anonymous=True)

    # Subscribe to the gps_pose topic
    rospy.Subscriber('/gps_pose', PoseWithCovarianceStamped, gps_pose_callback)
    
    # Subscribe to the reach/fix topic to get the GPS status
    rospy.Subscriber('/reach/fix', NavSatFix, gps_fix_callback)

    # Publish to the odom topic
    global odom_pub
    odom_pub = rospy.Publisher('/gt_poses', Odometry, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        gps_pose_to_odom_listener()
    except rospy.ROSInterruptException:
        pass

