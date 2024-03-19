#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


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


if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_arm', anonymous=True)
        arm_group = moveit_commander.MoveGroupCommander("arm")

        # arbitrary pose in the robot's base_link frame
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0

        move_fetch_arm(arm_group, target_pose)
        tuck_arm(arm_group)
        rospy.loginfo("Task complete. Shutting down...")
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
