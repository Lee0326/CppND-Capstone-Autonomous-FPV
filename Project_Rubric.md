# CPPND: Capstone-Autonomous FPV Simulation



## File and class structure

The code is tested on ROS (melodic) under ubuntu 18.04. All the ROS packages are stored in the `src` folder.  The `so3_quadrotor_simulator` is the simulation package, which is copied from the [HKUST-Aerial-Robotics](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) . The own created `drone_planning` has two functions:  1. the simulation of the gates' movement.  2. the trajectory planning according to the detected position of gates.  The `so3_control` contains a nodelet subscribing the position command topic published by the `drone_planning` node and transmits these command to so3 command that can be followed by the drone to the `so3_quadrotor_simulator` node.  The rqt_graph of the overall simulation is:

![rqt_graph](files/rqt_graph.png)

