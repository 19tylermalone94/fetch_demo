#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Global variable to keep track of the closest distance to an object
closest_object_distance = None

# Callback function for the laser scan data
def laser_scan_callback(scan_data):
    global closest_object_distance
    # Assuming that the laser scan range array is centered
    # We only consider some central values to check for objects right in front of the robot
    front_indices = range(len(scan_data.ranges)/2 - 100, len(scan_data.ranges)/2 + 100)
    front_ranges = [scan_data.ranges[i] for i in front_indices]
    closest_object_distance = min(front_ranges)

def drive_to_table():
    global closest_object_distance
    rospy.init_node('drive_fetch_bot', anonymous=True)
    
    # Subscribe to the laser scan topic
    rospy.Subscriber('/base_scan_raw', LaserScan, laser_scan_callback)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.2 # Move forward at 0.1 m/s
    
    while not rospy.is_shutdown():
        # Check if we have received any laser scan data
        if closest_object_distance is not None:
            # If the closest object is within 0.5 meters, stop the robot
            if closest_object_distance < 0.5:
                move_cmd.linear.x = 0
                pub.publish(move_cmd)
                rospy.loginfo("Table detected, stopping robot.")
                break
            else:
                # Otherwise, keep moving forward
                pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        drive_to_table()
    except rospy.ROSInterruptException:
        pass
