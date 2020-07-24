#include <Eigen/Geometry>
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator_node.cpp");
    ros::NodeHandle nh("~");
    ros::Publisher posi_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);

    //Define the trajectory starting state:
    Vec3 pos0 = Vec3(-2, 0, 0); //position
    Vec3 vel0 = Vec3(0, 0, 0);  //velocity
    Vec3 acc0 = Vec3(0, 0, 0);  //acceleration

    //define the goal state:
    Vec3 posf = Vec3(1, 0, 1); //position
    Vec3 velf = Vec3(0, 0, 1); //velocity
    Vec3 accf = Vec3(0, 0, 0); //acceleration

    //define the duration:
    double Tf = 3;

    double fmin = 5;          //[m/s**2]
    double fmax = 25;         //[m/s**2]
    double wmax = 20;         //[rad/s]
    double minTimeSec = 0.02; //[s]

    //Define how gravity lies in our coordinate system
    Vec3 gravity = Vec3(0, 0, -9.81); //[m/s**2]

    //Define the state constraints. We'll only check that we don't fly into the floor:
    Vec3 floorPos = Vec3(0, 0, 0);    //any point on the boundary
    Vec3 floorNormal = Vec3(0, 0, 1); //we want to be in this direction of the boundary

    RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    traj.SetGoalPosition(posf);
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    // Note: if you'd like to leave some states free, you can encode it like below.
    // Here we would be leaving the velocity in `x` (axis 0) free:
    //
    // traj.SetGoalVelocityInAxis(1,velf[1]);
    // traj.SetGoalVelocityInAxis(2,velf[2]);

    traj.Generate(Tf);

    
    ros::spin();
}