#!/usr/bin/env python

import rospy
import trep
import trep.ros as rostrep

def simulator():

    q0 = [0.1, 0] # Initial configuration
    dt = 0.05 # Timestep

    rospy.init_node('RRBot_Simulator')

    # import rrbot URDF to trep system
    system = rostrep.import_urdf(rospy.get_param('robot_description'))
    trep.potentials.Gravity(system, (0, 0, -9.8))

    rosmvi = rostrep.ROSMidpointVI(system, dt)
    rosmvi.initialize_from_configs(0.0, q0, dt, q0)

    while not rospy.is_shutdown():
        rosmvi.step([])
        rosmvi.sleep()

if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException: pass
