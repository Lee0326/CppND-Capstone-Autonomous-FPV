# CPPND: Capstone-Autonomous FPV Simulation

## Drone Racing

This is the repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).  Drone racing is a popular sport in which professional pilots fly small quadrotors through complex tracks at high speeds.  In this repo,  an autonomous fpv drone is simulated flying through multiple gates in rviz. 

## File and class structure

The drone dynamically plans the real-time trajectory according the the position prediction of the targeted gate.  The controller subscribes the trajectory topic and transmit the desired control signal to the dynamic model of the drone.  

## Dependencies for Running Locally

* Install ROS (melodic in ubuntu 18.04)

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

## Basic Build Instructions

#### Install the drone race package

```
git clone https://github.com/Lee0326/Drone_Racing_BIT.git
cd Drone_Racing_BIT
catkin_make
```

## Let's Race!

```
cd Drone_Racing_BIT
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
rosrun gate_visualization gate_visualization
```

The fpv drone will fly through the gates. It returns back to the initial position when passes the final gate.

![indoor](files/rviz.gif)