#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def tuck_arm(arm_group):
    # Define the joint angles for the tuck pose
    joint_angles = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Set the joint angles
    arm_group.set_joint_value_target(joint_angles)

    # Plan and execute the motion
    plan = arm_group.plan()
    arm_group.go(wait=True)

    # When done, remember to clear targets
    arm_group.clear_joint_value_targets()


def move_fetch_arm(arm_group, target_pose):
    # Set the target pose. Here you need to define `target_pose` which is a geometry_msgs/PoseStamped
    arm_group.set_pose_target(target_pose)

    # Plan and execute the motion
    plan = arm_group.plan()
    arm_group.go(wait=True)

    # When done, remember to clear targets
    arm_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_arm', anonymous=True)

        # Instantiate a `MoveGroupCommander` object, which is an interface to the arm
        arm_group = moveit_commander.MoveGroupCommander("arm")

        # Define the target pose in the base frame
        # This is just an example. You'll need to define the actual pose
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5  # Example values
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0

        move_fetch_arm(arm_group, target_pose)
        tuck_arm(arm_group)

        # Shutdown moveit_commander
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass