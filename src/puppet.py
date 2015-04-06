#!/usr/bin/env python

import rospy
import trep
import trep.ros as rostrep

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PuppetSimulator:

    def __init__(self):

        dt = 1/30.0
        self.connections = {'left_shoulder': [0.6, 0.15, 0.0, -0.6],
                            'right_shoulder':[0.6, -0.15, 0.0, -0.6],
                            'left_hand':     [0.2, 0.15, -0.2, -0.4],
                            'right_hand':    [0.2, -0.15, -0.2, -0.4]}

        rospy.init_node('Puppet_Simulator')
        rospy.Subscriber("string_markers/feedback", InteractiveMarkerFeedback, self.feedback)
        self.string_pub = rospy.Publisher('strings', Marker, queue_size=10)

        # Import puppet and string connection URDF
        self.system = rostrep.import_urdf(rospy.get_param('robot_description'))
        for connection in self.connections.keys():
            self.system = rostrep.import_urdf(rospy.get_param('string_description'), self.system, connection + '_')          
 
        # Add constraints from string ends to puppet
        for connection in self.connections.keys():
            trep.constraints.PointToPoint3D(self.system, connection + '_string', connection)

        trep.potentials.Gravity(self.system, (0, 0, -9.8))

        self.system.qk = sum(self.connections.values(),[])
        self.system.get_config('LElbowPhi').q = -1.5
        self.system.get_config('RElbowPhi').q = -1.5
        q0 = self.system.satisfy_constraints(keep_kinematic=True)

        rosmvi = rostrep.ROSMidpointVI(self.system, dt)
        rosmvi.initialize_from_configs(0.0, q0, dt, q0)
        self.create_strings()

        while not rospy.is_shutdown():
            rosmvi.step([], sum(self.connections.values(),[]))
            self.update_string()
            rosmvi.sleep()

    def feedback(self, data):
        self.connections[data.marker_name][:3] = [data.pose.position.z, data.pose.position.x, data.pose.position.y]

    def create_strings(self):
        self.str_markers = {}
        for connection in self.connections.keys():
            string_mark = Marker()
            string_mark.type = Marker.LINE_LIST
            string_mark.header.frame_id = "world"
            string_mark.scale.x = 0.005
            string_mark.color.a = 0.8
            string_mark.id = hash(connection)%(2**16)
            self.str_markers[connection] = string_mark
            
    def update_string(self):
        for connection in self.connections.keys():
            p1 = self.system.get_frame(connection + '_string').g()[:3,3]
            p2 = self.system.get_frame(connection + '_string-rylink').g()[:3,3]
            self.str_markers[connection].points = [Point(*p1),Point(*p2)]
            self.string_pub.publish(self.str_markers[connection])

if __name__ == '__main__':
    try:
        PuppetSimulator()
    except rospy.ROSInterruptException: pass
