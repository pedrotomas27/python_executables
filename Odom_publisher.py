#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Point, Vector3
from unitree_legged_msgs.msg import HighState

# Create a transform broadcaster
odom_to_base_broadcaster = tf2_ros.TransformBroadcaster()

def normalize_quaternion(x, y, z, w):
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        return 0, 0, 0, 1  # Return identity quaternion if norm is zero
    else:
        return x/norm, y/norm, z/norm, w/norm

def high_state_callback(msg):
    # Extract relevant data
    position = msg.position
    quaternion = msg.imu.quaternion
    velocity = msg.velocity
    yaw_speed = msg.yawSpeed

    # Normalize the quaternion
    qx, qy, qz, qw = normalize_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    # Create and publish the Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base"

    # Set position
    odom_msg.pose.pose.position = Point(*position)

    # Set orientation
    odom_msg.pose.pose.orientation = Quaternion(qx, qy, qz, qw)

    # Set linear velocity
    odom_msg.twist.twist.linear = Vector3(*velocity)

    # Set angular velocity
    odom_msg.twist.twist.angular = Vector3(0, 0, yaw_speed)

    # Set covariance values
    # Position covariance (assuming no uncertainty in position)
    odom_msg.pose.covariance = [
        0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.001
    ]

    # Velocity covariance (assuming no uncertainty in velocity)
    odom_msg.twist.covariance = [
        0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.001
    ]

    # Publish the odometry message
    odom_pub.publish(odom_msg)

    # Create and broadcast the transform
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = odom_msg.header.stamp
    transform_stamped.header.frame_id = "odom"
    transform_stamped.child_frame_id = "base"
    transform_stamped.transform.translation = odom_msg.pose.pose.position
    transform_stamped.transform.rotation = odom_msg.pose.pose.orientation
    odom_to_base_broadcaster.sendTransform(transform_stamped)


def main():
    rospy.init_node('high_state_to_odom')

    # Create a publisher for the odometry message
    global odom_pub
    odom_pub = rospy.Publisher('/odom_go1', Odometry, queue_size=10)

    # Subscribe to the high_state topic
    rospy.Subscriber('/high_state', HighState, high_state_callback)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()

