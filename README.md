# CPPND: Capstone-Autonomous FPV Simulation

This is the repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213). 

## Autonomous Drone Racing

Drone racing is a popular sport in which professional pilots fly small quadrotors through complex tracks composed of multiple gates at high speeds.  In this repo,  an autonomous fpv drone is simulated flying through multiple gates in rviz. In this repo, the drone dynamically plans the real-time trajectory according the the predicted position of the targeted gates.  The controller subscribes the trajectory topic and publishs the desired command signal to the drone.  The corresponding pose of the drone is calculated according to dynamics and also visualized in rviz. 

## Dependencies for Running Locally

* Install ROS 

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install ros-melodic-desktop
# Source ROS
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
rosdep init
rosdep update
```

- Install catkin

```
sudo apt-get install ros-melodic-catkin python-catkin-tools
```

## Install the drone race package

```
https://github.com/Lee0326/CppND-Capstone-Autonomous-FPV.git
cd CppND-Capstone-Autonomous-FPV
catkin_make
```

## Let's Race!

First, launch the simulator. There will be a quadrotor placed in the center of the world frame if the simulator is properly launched.

```
cd CppND-Capstone-Autonomous-FPV
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
```

Then run the `drone_planning` node. The fpv drone will fly through the gates. It will return back to the initial position when passes the final gate and start a new loop again.

```
cd CppND-Capstone-Autonomous-FPV
source devel/setup.bash
rosrun drone_planning drone_planning 
```

![indoor](files/rviz.gif)

