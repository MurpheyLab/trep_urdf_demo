trep\_urdf\_demo
================

This [ROS](http://www.ros.org/) package contains two demos that use [trep](http://nxr.northwestern.edu/trep)
to integrate the dynamics of a two link RR robot and a create an interactive marionette simulation.

Both demos incorporate the URDF import tool available through trep.ros as well as the ROSMidpointVI integrator. 
The transforms for all frames provided by the URDF are published by the ROSMidpointVI class.

After cloning the repository to your ROS workspace, the demos can be run with the following launch files:

#### RRbot Simulator
`roslaunch trep_urdf_demo rrbot.launch`
 
#### Interactive Marionette Simulator
 `roslaunch trep_urdf_demo puppet.launch`
