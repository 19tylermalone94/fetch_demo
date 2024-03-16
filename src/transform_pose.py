#!/usr/bin/env python
import rospy
import tf
import sys
from geometry_msgs.msg import PoseStamped
import moveit_commander

def transform_pose(input_pose, from_frame, to_frame):
    """
    Transforms a pose from the source frame to the target frame using TF.
    """
    try:
        listener = tf.TransformListener()
        listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(4.0))
        transformed_pose = listener.transformPose(to_frame, input_pose)
        return transformed_pose
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
        rospy.logerr("TF Error when transforming from %s to %s: %s" % (from_frame, to_frame, str(ex)))
        return None

def tuck_arm(arm_group):
    """
    Tucks the arm by setting predefined joint values.
    """
    joint_goal = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    arm_group.set_joint_value_target(joint_goal)
    
    if arm_group.go(wait=True):
        rospy.loginfo("Arm successfully tucked.")
    else:
        rospy.logwarn("Failed to tuck the arm.")
    arm_group.clear_joint_value_targets()

def move_fetch_arm(arm_group, target_pose):
    """
    Moves the Fetch arm to the given target pose.
    """
    arm_group.set_pose_target(target_pose)
    if arm_group.go(wait=True):
        rospy.loginfo("Arm movement to target pose succeeded.")
    else:
        rospy.logwarn("Failed to move arm to target pose.")
    arm_group.clear_pose_targets()

def main():
    rospy.init_node('transform_pose')
    
    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Create instances for robot and move group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    # Define target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0
    
    # Transform target pose to camera frame
    transformed_pose = transform_pose(target_pose, "base_link", "head_camera_link")
    
    if transformed_pose:
        move_fetch_arm(arm_group, transformed_pose)
        rospy.sleep(1)  # Ensure the arm has enough time to complete the move
        tuck_arm(arm_group)
    else:
        rospy.logerr("Failed to transform pose or move arm.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
