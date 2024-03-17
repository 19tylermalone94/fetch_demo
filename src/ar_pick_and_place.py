#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from control_msgs.msg import PointHeadGoal, PointHeadAction
from actionlib import SimpleActionClient
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import tf2_ros
import tf2_geometry_msgs

class FollowTrajectoryClient(object):
    def __init__(self, name, joint_names):
        self.client = SimpleActionClient("%s/follow_joint_trajectory" % name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(positions) != len(self.joint_names):
            rospy.logerr("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        trajectory.points.append(point)
        self.client.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
        self.client.wait_for_result()

# Global variables
closest_object_distance = None
ar_marker_pose = None

def laser_scan_callback(scan_data):
    global closest_object_distance
    front_ranges = [scan_data.ranges[i] for i in range(len(scan_data.ranges)//2 - 100, len(scan_data.ranges)//2 + 100) if scan_data.ranges[i] > 0]
    if front_ranges:  # Check if front_ranges is not empty
        closest_object_distance = min(front_ranges)

def ar_pose_marker_callback(msg):
    global ar_marker_pose
    for marker in msg.markers:
        if marker.id == 0:  # Assuming you are interested in marker with ID 0
            rospy.loginfo("AR marker detected")
            ar_marker_pose = PoseStamped()
            ar_marker_pose.header = marker.header
            ar_marker_pose.pose = marker.pose.pose
            break

def point_camera_down():
    client = SimpleActionClient('/head_controller/point_head', PointHeadAction)
    client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1.0
    goal.target.point.y = 0.0
    goal.target.point.z = -0.4  # Assuming you want to point down
    goal.min_duration = rospy.Duration(1.0)
    client.send_goal(goal)
    client.wait_for_result()

def extend_torso():
    rospy.loginfo("Extending torso")
    torso_client = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    torso_client.move_to([0.5], 5.0)

def move_arm_to_pose():
    global ar_marker_pose
    if not ar_marker_pose:
        rospy.logwarn("No AR marker detected. Cannot move arm.")
        return

    # Adjust the Z-coordinate to be 5cm above the marker
    ar_marker_pose.pose.position.z += 0.1

    rospy.loginfo("Moving arm to position 5cm above AR marker.")
    arm_group = MoveGroupCommander("arm")

    # Increase the planning time to allow more time for finding a solution
    arm_group.set_planning_time(20)
    # Increase the number of planning attempts
    arm_group.set_num_planning_attempts(10)

    # Set the updated pose as the target
    arm_group.set_pose_target(ar_marker_pose)

    # Attempt to move the arm
    success = arm_group.go(wait=True)
    arm_group.stop()  # Ensure that there is no residual movement
    arm_group.clear_pose_targets()

    if success:
        rospy.loginfo("Arm successfully moved to pose above AR marker.")
    else:
        rospy.logwarn("Failed to move arm to pose above AR marker.")



def drive_to_table():
    global closest_object_distance
    rospy.init_node('drive_fetch_bot', anonymous=True)
    
    rospy.Subscriber('/base_scan', LaserScan, laser_scan_callback)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_pose_marker_callback)
    
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    move_cmd = Twist()
    
    rospy.loginfo("Robot started moving towards the table")
    while not rospy.is_shutdown():
        if closest_object_distance and closest_object_distance < 0.5:
            rospy.loginfo("Table detected, stopping robot.")
            move_cmd.linear.x = 0
            cmd_vel_pub.publish(move_cmd)
            break
        else:
            move_cmd.linear.x = 0.2
            cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    rospy.sleep(1)  # Wait for the robot to completely stop

    rospy.loginfo("Reversing for 1 meter...")
    reverse_start_time = rospy.Time.now()
    reverse_duration = rospy.Duration(2)  # Duration to move 1 meter at 0.25 m/s
    move_cmd.linear.x = -0.25  # Move backwards at 0.25 m/s

    while rospy.Time.now() - reverse_start_time < reverse_duration:
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(0.1)  # Short sleep to keep sending commands

    move_cmd.linear.x = 0  # Stop the robot
    cmd_vel_pub.publish(move_cmd)

    extend_torso()
    point_camera_down()

    rospy.loginfo("Waiting for AR marker detection.")
    while not rospy.is_shutdown() and ar_marker_pose is None:
        rospy.sleep(1)  # Wait for AR marker to be detected

    move_arm_to_pose()

if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        drive_to_table()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
