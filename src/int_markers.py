#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

class StringMarkers:

    def __init__(self):

        rospy.init_node("String_Markers")
        
        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("string_markers")
        self.create_marker('left_hand', [0.15, -0.2, 0.2])
        self.create_marker('right_hand', [-0.15, -0.2, 0.2])
        self.create_marker('left_shoulder', [0.15, 0.0, 0.6])
        self.create_marker('right_shoulder', [-0.15, 0.0, 0.6])
        rospy.spin()

    def processFeedback(self, feedback):
        pass

    def create_marker(self, marker_name, init_pos=[0,0,0]):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/world"
        int_marker.name = marker_name
        int_marker.description = marker_name + ' marker'
        int_marker.scale = 0.2

        # create marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.5
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.75

        # create a non-interactive control which contains the box
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append( marker )

        # add the control to the interactive marker
        int_marker.controls.append( sphere_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)

        int_marker.pose.position.x = init_pos[0]
        int_marker.pose.position.y = init_pos[1]
        int_marker.pose.position.z = init_pos[2]

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()



if __name__=="__main__":
    try:
        StringMarkers()
    except rospy.ROSInterruptException: pass
   
