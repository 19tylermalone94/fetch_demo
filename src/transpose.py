#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

def transform_pose(input_pose, from_frame, to_frame, tf_buffer):
    # Use the tf_buffer passed as a parameter to perform transformation
    try:
        transform = tf_buffer.lookup_transform(to_frame,
                                               from_frame,
                                               rospy.Time(0),
                                               rospy.Duration(5.0))  # Wait up to 5 seconds for the transform
        transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
        return transformed_pose.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        rospy.logerr('Transform error: %s', ex)
        return None

def move_fetch_arm(target_pose):
    arm_group = moveit_commander.MoveGroupCommander("arm")
    arm_group.set_planner_id("RRTConnectkConfigDefault")  # Use a specific planner

    # Set the target pose directly
    arm_group.set_pose_target(target_pose)

    # Plan to the given pose
    plan = arm_group.plan()
    if not plan.joint_trajectory.points:  # Check if plan is empty; if so, no plan was found
        rospy.logerr("No motion plan found.")
        return False

    # Execute the plan
    success = arm_group.execute(plan, wait=True)
    return success

if __name__ == '__main__':
    rospy.init_node('fetch_move_arm_tf', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    target_pose_base_frame = geometry_msgs.msg.PoseStamped()
    target_pose_base_frame.header.frame_id = "base_link"
    target_pose_base_frame.pose.position.x = 0.5
    target_pose_base_frame.pose.position.y = 0.0
    target_pose_base_frame.pose.position.z = 0.5
    target_pose_base_frame.pose.orientation.w = 1.0

    rospy.sleep(2)  # Wait for TF to be ready

    transformed_pose = transform_pose(target_pose_base_frame, "base_link", "head_camera_link", tf_buffer)
    
    if transformed_pose:
        success = move_fetch_arm(transformed_pose)
        if not success:
            rospy.logerr("Failed to execute motion plan.")
    else:
        rospy.logerr("Failed to transform pose.")
