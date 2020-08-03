# CPPND: Capstone-Autonomous FPV Simulation

## Autonomous Drone Racing

This is the repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).  Drone racing is a popular sport in which professional pilots fly small quadrotors through complex tracks at high speeds.  In this repo,  an autonomous fpv drone is simulated flying through multiple gates in rviz. In this repo, the drone dynamically plans the real-time trajectory according the the predicted position of the targeted gates.  The controller subscribes the trajectory topic and transmit the desired command signal to the drone.  The corresponding pose of the drone is calculated according to dynamics and visualized in rviz. 

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

## Basic Build Instructions

#### Install the drone race package

```
git clone https://github.com/Lee0326/Drone_Racing_BIT.git
cd Drone_Racing_BIT
catkin_make
```

## Let's Race!

First, launch the simulator. There will be a quadrotor placed in the center of the world frame is the simulator is properly launched.

```
cd Drone_Racing_BIT
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
```

Then run the `drone_planning` node. The fpv drone will fly through the gates. It will return back to the initial position when passes the final gate and start a new loop again.

```
cd Drone_Racing_BIT
source devel/setup.bash
rosrun drone_planning drone_planning 
```

![indoor](files/rviz.gif)

The markers of the gates are also published when the node is running. It needs to be added for visualization by the left-down button in rviz. The topics are `/gate_maker${N}` where `N` is the gate number.