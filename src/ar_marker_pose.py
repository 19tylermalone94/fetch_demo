#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from ar_track_alvar_msgs.msg import AlvarMarkers


marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


def visualize_ar_marker(ar_marker):
    marker = Marker()
    marker.header.frame_id = ar_marker.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "ar_markers"
    marker.id = ar_marker.id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = ar_marker.pose.pose
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker)


def tuck_arm():
    rospy.loginfo("Tucking the arm")
    arm_group = MoveGroupCommander("arm_with_torso")
    tucked_position = {
        'shoulder_pan_joint': 1.32,
        'shoulder_lift_joint': 1.40,
        'upperarm_roll_joint': -0.2,
        'elbow_flex_joint': 1.72,
        'forearm_roll_joint': 0.0,
        'wrist_flex_joint': 1.66,
        'wrist_roll_joint': 0.0
    }   
    arm_group.go(tucked_position, wait=True)
    arm_group.stop()
    rospy.loginfo("Arm is tucked")


def move_forward(distance, speed):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    move_cmd = Twist()
    move_cmd.linear.x = speed
    rospy.loginfo("Moving forward")
    start_time = rospy.Time.now()
    duration = distance / speed
    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(move_cmd)
        rospy.sleep(0.1)
    move_cmd.linear.x = 0
    pub.publish(move_cmd)
    rospy.loginfo("Finished moving forward")


def ar_pose_marker_callback(msg):
    global ar_subscriber
    try:
        for marker in msg.markers:
            visualize_ar_marker(marker)
            if marker.id == 0:
                rospy.loginfo("AR marker detected, moving arm to pose")
                move_arm_to_pose(marker.pose.pose)
                # Unsubscribe after successful detection and arm movement
                ar_subscriber.unregister()
                rospy.loginfo("Unsubscribed from AR marker detections.")
                break
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS Interrupt Exception: %s" % e)
    except Exception as e:
        rospy.logerr("General Exception: %s" % e)


def move_arm_to_pose(pose):
    pose.position.x -= 0.11
    arm_group = MoveGroupCommander("arm_with_torso")
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.loginfo("Arm moved to AR marker pose")


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('ar_marker_pose', anonymous=True)
    move_forward(distance=0.85, speed=0.2)
    ar_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_pose_marker_callback)
    rospy.sleep(5)
    tuck_arm()
    rospy.spin()
    roscpp_shutdown()
