# CPPND: Capstone-Autonomous FPV Simulation

## File and class structure

The code is tested on ROS (melodic) under ubuntu 18.04. All the ROS packages are stored in the `src` folder.  The `so3_quadrotor_simulator` is the simulation package, which is copied from the [HKUST-Aerial-Robotics](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) . The `drone_planning` has two functions:  1. the simulation of the gates' movement.  2. the trajectory planning according to the detected position of gates.  The trajectory generation in this package is realized by [a optimal controller](https://github.com/markwmuller/RapidQuadrocopterTrajectories) and the related files are `RapidTrajectoryGenerator.cpp` and `SingleAxisTrajectory.cpp`. The `so3_control` contains a nodelet subscribing the position command topic published by the `drone_planning` node and transmits these command to so3 command that can be followed by the drone to the `so3_quadrotor_simulator` node.  The rqt_graph of the overall simulation is:

![rqt_graph](files/rqt_graph.png)



There are three main classes:   `Gate`,`TrajectoryServer` and `SO3ControlNodelet`.  

- The `Gate` class simulates the movement of the gate. Once the gate is locked by the drone as a target , the position of the gate is predicted for the trajectory calculation. 
- The `TrajectoryServer` dynamically generates the desired trajectory according to the real-time position of the drone and the targeted gate position. All the moving gates are also launched in this class in multiple threads.
- The `SO3ControlNodelet` is a inherited ROS nodelet class to publish the so3_command to the drone model.

## Project Rubrics

- #### Loops, Functions, I/O

  - As can be seen from `gate_detector.cpp` and `trajectory_generator.cpp`, a variety of control structures (e.g. while loop, for loop and if else...) are used in the project. 
  - The simulator nodes are launched by the `simultor.launch` under the `/src/so3_quadrotor_simulator/launch` folder.

- #### Object Oriented Programming

  - The three classed mentioned above illustrate the implement of OOP programming.
  - All the data members in the `trajectory_generator.h` and `gate_detector.h` file are specified as public or private.
  - All class members that are set to argument values are initialized through member initialization lists. e.g. The constructor function of the `TrajectoryServer` and `Gate` classes.
  - Member data that is subject to an invariant is hidden from the user. e.g. the `ready_`in `gate_detector.h` can only modified by the public member function `setReady()`. 
  - To initialize the `TrajectoryServer` class, the overloaded constructor of `RapidTrajectoryGenerator` is added in `RapidTrajectoryGenerator.h`. (line 72 in `RapidTrajectoryGenerator.h`) 
  - The  function `onInit()` function in line159, `so3_control_nodelet.cpp` file overrides the virtual base function  `onInit()` in `nodelet` class.

- #### Memory Management

  - the `odom_callback` function receives the reference of `nav_msgs::Odometry::ConstPtr` as input (line 53 in  `trajectory_generator.h`) and `updateTarget` and `setCV` function in `Gate` also use pass-by-reference in `gate_detector.h`.
  - All the resources are obtained in the constructor function of class `Gate` and `TrajectoryServer`  in line 6 of `gate_detector.cpp` and line 2 of `trajectory_generator.cpp`.
  - All the member variables are defined as smart pointers, and there is no function of copy assignment operator, move constructor, move assignment operator, and destructor in the `Gate` and `TrajectoryServer` class.
  - The `segment_pt_` and `target_ptr_` are defined as shared pointer in lin 21 and 22 of `trajectory_generator.h`.

- #### Concurrency

  - The gates' movement is running in multiple threads in the `TrajectoryServer::launchThreads()` function (line 115 in `trajectory_generator.cpp`). 
  - In Gate::`updateTarget` function  A promise is transmitted to the gate thread and when the drone fly through it, the id of next gate is return to the future (line 11 in `gate_detector.cpp`).
  -  `std::unique_lock` is used to protect the pointer to Matrix3d  `target_ptr_` that is shared across multiple threads in the project code (line 13 in `gate_detector.cpp` ).
  - A `std::condition_variable` is define in class TrajectoryServer as a member variable to synchronize the startup of the execution of all the gates' thread. (line 125 in `trajectory_generator.cpp`).

