#ifndef FRAME_H
#define FRAME_H

#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "pose_utils.h"
#include <future>
#include <memory>

using namespace Eigen;

class gate
{
private:
    std::string mesh_ = "package://gate_visualization/meshes/gate.dae";
    Vector3d position_;
    Quaterniond orientation_ = Quaterniond(1, 0, 0, 0);
    Vector3d ini_pos_;
    int id_;
    ros::Time trigger_time_;
    ros::Publisher target_pub_;
    visualization_msgs::Marker maker_;

public:
    gate(ros::Publisher &target_pub, Vector3d ini_pos, ros::Time trigger_time, int id);
    void updateState(std::promise<Matrix3d> &&prms);
    void publishMaker();
};

#endif