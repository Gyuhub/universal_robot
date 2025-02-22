#!/usr/bin/env python3

import rospy
import tf.transformations
import numpy as np
import math

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, Quaternion

marker_pose = PoseStamped()
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


def wait_for_initial_pose():
    msg = rospy.wait_for_message("cartesian_pose_controller/ee_state",
                                 PoseStamped)
    rospy.loginfo('Initial pose %f %f %f', msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z)

    quat = msg.pose.orientation
    quat_norm = math.sqrt(quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w)
    quat.w = quat.w / quat_norm
    quat.x = quat.x / quat_norm
    quat.y = quat.y / quat_norm
    quat.z = quat.z / quat_norm
    
    marker_pose.pose.orientation = quat
    marker_pose.pose.position.x = msg.pose.position.x
    marker_pose.pose.position.y = msg.pose.position.y
    marker_pose.pose.position.z = msg.pose.position.z

if __name__ == "__main__":
    rospy.init_node("ee_pose_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    wait_for_initial_pose()

    pose_pub = rospy.Publisher(
        "ee_pose_desired", PoseStamped, queue_size=10)
    server = InteractiveMarkerServer("ee_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "ee_pose"
    int_marker.description = ("pose of the end-effecotr")
    int_marker.pose = marker_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name))

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    server.insert(int_marker, process_feedback)

    server.applyChanges()

    rospy.spin()
