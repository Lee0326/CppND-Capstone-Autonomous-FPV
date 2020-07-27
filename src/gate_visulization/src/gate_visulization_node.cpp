#include <iostream>
#include <math.h>
#include <string.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "pose_utils.h"

using namespace Eigen;
class gate
{
private:
    std::string mesh_;
    Vector3d position_;
    Quaterniond orientation_ = Quaterniond(1, 0, 0, 0);
    Vector3d ini_pos_;
    int id_;
    ros::Time trigger_time_;
    visualization_msgs::Marker maker_;

public:
    gate(std::string mesh, Vector3d ini_pos, ros::Time trigger_time, int id) : mesh_(mesh), ini_pos_(ini_pos), trigger_time_(trigger_time), id_(id)
    {
        updateState();
    };
    void updateState()
    {
        double duration = (ros::Time::now() - trigger_time_).toSec();
        auto deltaPos = Vector3d(0.5 * sin(duration), 0.5 * cos(duration), 0);
        position_ = ini_pos_ + deltaPos;
        std::cout << position_[0] << std::endl;
    };
    void publishMaker(ros::Publisher &gatePub)
    {
        maker_.header.frame_id = "world";
        maker_.ns = "gate";
        maker_.id = 0;
        maker_.type = visualization_msgs::Marker::MESH_RESOURCE;
        maker_.action = visualization_msgs::Marker::ADD;
        maker_.mesh_resource = mesh_;
        while (ros::ok)
        {
            maker_.header.stamp = ros::Time();
            //set the pose of the gate
            maker_.pose.position.x = position_[0];
            maker_.pose.position.y = position_[1];
            maker_.pose.position.z = position_[2];
            maker_.pose.orientation.x = 0.0;
            maker_.pose.orientation.y = 0.0;
            maker_.pose.orientation.z = 0.0;
            maker_.pose.orientation.w = 1.0;

            //set the scale
            maker_.scale.x = 0.5;
            maker_.scale.y = 0.5;
            maker_.scale.z = 0.5;
            maker_.color.a = 1.0;
            maker_.color.r = 0.0;
            maker_.color.g = 1.0;
            maker_.color.b = 0.0;

            // publish the gate marker
            gatePub.publish(maker_);
            updateState();
            ros::spinOnce();
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle nh("~");
    ros::Publisher gatePub = nh.advertise<visualization_msgs::Marker>("gate_marker", 0);

    // define the movement of the gates
    Vector3d init_position(3, 3, 1);
    std::string mesh_resource_ = "package://gate_visualization/meshes/gate.dae";
    auto trigger_time = ros::Time::now();
    gate gate1(mesh_resource_, init_position, trigger_time, 0);
    gate1.publishMaker(gatePub);
    std::thread t = std::thread(&gate::updateState, &gate1);
    //t.join();
    return 0;
}
