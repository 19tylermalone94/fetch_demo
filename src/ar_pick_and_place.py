#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from control_msgs.msg import PointHeadGoal, PointHeadAction
from actionlib import SimpleActionClient

# Global variable to keep track of the closest distance to an object
closest_object_distance = None

# Callback function for the laser scan data
def laser_scan_callback(scan_data):
    global closest_object_distance
    # Assuming that the laser scan range array is centered
    # We only consider some central values to check for objects right in front of the robot
    front_indices = range(len(scan_data.ranges)//2 - 100, len(scan_data.ranges)//2 + 100)
    front_ranges = [scan_data.ranges[i] for i in front_indices]
    closest_object_distance = min(front_ranges)

# Function to make the robot's camera look down
def point_camera_down():
    # Create an action client to send goal requests to the point head controller
    client = SimpleActionClient('/head_controller/point_head', PointHeadAction)
    client.wait_for_server()

    # Define a goal to send to the action server
    goal = PointHeadGoal()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1.5  # Points 1.5 meter ahead of the robot
    goal.target.point.y = 0.0
    goal.target.point.z = 0.0 # Points down (adjust based on the actual height of your table)
    goal.min_duration = rospy.Duration(1.0)

    # Send the goal and wait for it to complete
    client.send_goal(goal)
    client.wait_for_result()

# Function to drive the robot to the table
def drive_to_table():
    global closest_object_distance
    rospy.init_node('drive_fetch_bot', anonymous=True)

    # Subscribe to the laser scan topic
    rospy.Subscriber('/base_scan_raw', LaserScan, laser_scan_callback)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.3  # Move forward at 0.3 m/s

    while not rospy.is_shutdown():
        # Check if we have received any laser scan data
        if closest_object_distance is not None:
            # If the closest object is within 0.5 meters, stop the robot
            if closest_object_distance < 0.5:
                move_cmd.linear.x = 0
                cmd_vel_pub.publish(move_cmd)
                rospy.loginfo("Table detected, stopping robot.")
                
                # Robot has stopped, now point the camera down
                rospy.loginfo("Pointing camera down to look for AR marker.")
                point_camera_down()

                # Implement the logic for AR marker detection here or start a new node/package
                # This could involve subscribing to the camera image topic and processing the image
                # to find the AR marker

                # Prevent the loop from continuing
                break
            else:
                # Otherwise, keep moving forward
                cmd_vel_pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        drive_to_table()
    except rospy.ROSInterruptException:
        pass
