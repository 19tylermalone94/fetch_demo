#!/usr/bin/env python
import rospy
import tf
import sys
from geometry_msgs.msg import PoseStamped
import moveit_commander


def transform_pose(input_pose, from_frame, to_frame):
    try:
        listener = tf.TransformListener()
        listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(4.0))
        transformed_pose = listener.transformPose(to_frame, input_pose)
        return transformed_pose
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
        return None


def tuck_arm(arm_group):
    joint_angles = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    arm_group.set_joint_value_target(joint_angles)
    arm_group.plan()
    arm_group.go(wait=True)
    arm_group.clear_pose_targets()


def move_fetch_arm(arm_group, target_pose):
    arm_group.set_pose_target(target_pose)
    arm_group.plan()
    arm_group.go(wait=True)
    arm_group.clear_pose_targets()


def main():
    rospy.init_node('transform_pose')
    moveit_commander.roscpp_initialize(sys.argv)
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    # arbitrary target pose in the robot's base_link frame
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0
    
    # transform base_link pose to head_camera_link pose
    transformed_pose = transform_pose(target_pose, "base_link", "head_camera_link")
    if transformed_pose:
        move_fetch_arm(arm_group, transformed_pose)
        rospy.sleep(1)
        tuck_arm(arm_group)
    else:
        rospy.logerr("Failed to transform pose or move arm.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
