#!/usr/bin/env python
import math
import time
import rospy
from robot_clients import FollowTrajectoryClient, PointHeadClient, GraspingClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


closest_object_distance = float('inf')


def laser_callback(msg):
    global closest_object_distance
    front_angles = msg.ranges[len(msg.ranges) // 3: 2 * len(msg.ranges) // 3]
    closest_object_distance = min(front_angles)


def move_forward():
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/base_scan_raw', LaserScan, laser_callback)
    speed = Twist()
    speed.linear.x = 0.5
    rate = rospy.Rate(10) # 10 Hz
    stop_distance = 1.25
    while not rospy.is_shutdown():
        if closest_object_distance > stop_distance:
            cmd_pub.publish(speed)
        else:
            speed.linear.x = 0
            cmd_pub.publish(speed)
            break
        rate.sleep()


def rotate_robot_180_degrees():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = math.radians(90)
    rotation_time = 2.66
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < rotation_time:
        pub.publish(twist)
        time.sleep(0.1)
    twist.angular.z = 0.0
    pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("pick_and_place")
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()
    rospy.loginfo("Moving to table...")
    move_forward()
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])
    head_action.look_at(2.66, 0, 0.0, "map")
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene()
        cube, grasps = grasping_client.getGraspableCube()
        if cube == None:
            rospy.logwarn("Perception failed.")
            continue
        if grasping_client.pick(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")
    grasping_client.tuck()
    rospy.loginfo("Lowering torso...")
    torso_action.move_to([0.0, ])
    rospy.loginfo("Moving to second table...")
    rotate_robot_180_degrees()
    move_forward()
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = cube.primitive_poses[0]
        pose.pose.position.z += 0.05
        pose.header.frame_id = cube.header.frame_id
        if grasping_client.place(cube, pose):
            break
        rospy.logwarn("Placing failed.")
    grasping_client.tuck()
    torso_action.move_to([0.0, ])
