#include <iostream>
#include <math.h>
#include <string.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "pose_utils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle nh("~");
    ros::Publisher gatePub = nh.advertise<visualization_msgs::Marker>("gate_marker", 0);
    // define the movement of the gates
    std::string mesh_resource_ = "package://gate_visualization/meshes/gate.dae";
    auto trigger_time = ros::Time::now();
    visualization_msgs::Marker gateROS;
    gateROS.header.frame_id = "world";
    gateROS.header.stamp = ros::Time();
    gateROS.ns = "gate";
    gateROS.id = 0;
    gateROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    gateROS.action = visualization_msgs::Marker::ADD;

    gateROS.mesh_resource = mesh_resource_;
    while (ros::ok())
    {
        double duration = (ros::Time::now() - trigger_time).toSec();

        //set the pose of the gate
        gateROS.pose.position.x = 5 + 0.5 * sin(duration);
        gateROS.pose.position.y = 5 + 0.5 * sin(duration);
        gateROS.pose.position.z = 1 + 0.2 * sin(duration);
        gateROS.pose.orientation.x = 0.0;
        gateROS.pose.orientation.y = 0.0;
        gateROS.pose.orientation.z = 0.0;
        gateROS.pose.orientation.w = 1.0;

        //set the scale
        gateROS.scale.x = 0.5;
        gateROS.scale.y = 0.5;
        gateROS.scale.z = 0.5;
        gateROS.color.a = 1.0;
        gateROS.color.r = 0.0;
        gateROS.color.g = 1.0;
        gateROS.color.b = 0.0;

        // publish the gate marker
        gatePub.publish(gateROS);

        ros::spinOnce();
    }

    return 0;
}
