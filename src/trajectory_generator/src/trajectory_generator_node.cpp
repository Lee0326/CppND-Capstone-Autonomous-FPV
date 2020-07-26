#include <Eigen/Geometry>
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh("~");
    ros::Publisher posi_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

    //Define the trajectory starting state:
    Vec3 pos0 = Vec3(-2, 0, 0); //position
    Vec3 vel0 = Vec3(0, 0, 0);  //velocity
    Vec3 acc0 = Vec3(0, 0, 0);  //acceleration

    //define the goal state:
    Vec3 posf = Vec3(10, 5, 2); //position
    Vec3 velf = Vec3(0, 0, 0);  //velocity
    Vec3 accf = Vec3(0, 0, 0);  //acceleration

    //define the duration:
    double Tf = 5;

    double fmin = 5;          //[m/s**2]
    double fmax = 25;         //[m/s**2]
    double wmax = 20;         //[rad/s]
    double minTimeSec = 0.02; //[s]

    //Define how gravity lies in our coordinate system
    Vec3 gravity = Vec3(0, 0, -9.81); //[m/s**2]

    //Define the state constraints. We'll only check that we don't fly into the floor:
    Vec3 floorPos = Vec3(0, 0, 0);    //any point on the boundary
    Vec3 floorNormal = Vec3(0, 0, 1); //we want to be in this direction of the boundary

    RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    traj.SetGoalPosition(posf);
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    // Note: if you'd like to leave some states free, you can encode it like below.
    // Here we would be leaving the velocity in `x` (axis 0) free:
    //
    // traj.SetGoalVelocityInAxis(1,velf[1]);
    // traj.SetGoalVelocityInAxis(2,velf[2]);

    traj.Generate(Tf);
    Vec3 alpha(0, 0, 0);
    Vec3 beta(0, 0, 0);
    Vec3 gamma(0, 0, 0);
    for (int i = 0; i < 3; i++)
    {
        cout << "Axis #" << i << "\n";
        cout << "\talpha = " << traj.GetAxisParamAlpha(i);
        cout << "\tbeta = " << traj.GetAxisParamBeta(i);
        cout << "\tgamma = " << traj.GetAxisParamGamma(i);
        cout << "\n";
        alpha[i] = traj.GetAxisParamAlpha(i);
        beta[i] = traj.GetAxisParamBeta(i);
        gamma[i] = traj.GetAxisParamGamma(i);
    }
    auto trigger_time = ros::Time::now();
    quadrotor_msgs::PositionCommand pos_cmd;
    while (ros::ok())
    {
        double dt = (ros::Time::now() - trigger_time).toSec();
        pos_cmd.header.stamp = ros::Time::now();
        pos_cmd.header.frame_id = "world";
        Vec3 Position = traj.GetPosition(dt);
        Vec3 Velocity = traj.GetVelocity(dt);
        Vec3 Acceleration = traj.GetAcceleration(dt);
        //position
        if (dt < Tf)
        {
            pos_cmd.position.x = Position[0];
            pos_cmd.position.y = Position[1];
            pos_cmd.position.z = Position[2];
            //velocity
            pos_cmd.velocity.x = Velocity[0];
            pos_cmd.velocity.y = Velocity[1];
            pos_cmd.velocity.z = Velocity[2];
            //acceleration
            pos_cmd.acceleration.x = Acceleration[0];
            pos_cmd.acceleration.y = Acceleration[1];
            pos_cmd.acceleration.z = Acceleration[2];
            //yaw
            pos_cmd.yaw = 0;
            pos_cmd.yaw_dot = 0;
            //cout << pos_cmd.position.x << endl;
        }
        // else
        // {
        //     pos_cmd.position.x = posf[0];
        //     pos_cmd.position.y = posf[0];
        //     pos_cmd.position.z = posf[0];
        //     pos_cmd.velocity.x = velf[0];
        //     pos_cmd.velocity.y = velf[0];
        //     pos_cmd.velocity.z = velf[0];
        //     pos_cmd.acceleration.x = accf[0];
        //     pos_cmd.acceleration.y = accf[0];
        //     pos_cmd.acceleration.z = accf[0];
        //     pos_cmd.yaw = 0;
        //     pos_cmd.yaw_dot = 0;
        // }
        pos_cmd.kx = {1, 1, 1};
        pos_cmd.kv = {1, 1, 1};
        posi_cmd_pub.publish(pos_cmd);
        ros::spinOnce();
    }
}