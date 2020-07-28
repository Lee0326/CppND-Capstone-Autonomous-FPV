#include <iostream>
#include <math.h>
#include <string.h>
#include <gate.h>

gate::gate(ros::Publisher &target_pub, Vector3d ini_pos, ros::Time trigger_time, int id) : target_pub_(target_pub), ini_pos_(ini_pos), trigger_time_(trigger_time), id_(id){};

void gate::updateState(std::promise<Matrix3d> &&prms)
{
    int count = 0;
    Matrix3d m;
    m << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    while (ros::ok())
    {
        double duration = (ros::Time::now() - trigger_time_).toSec();
        auto deltaPos = Vector3d(0.5 * sin(duration), 0.5 * cos(duration), 0);
        position_ = ini_pos_ + deltaPos;
        // std::cout << position_[0] << std::endl;
        gate::publishMaker();
        ros::spinOnce();
        if (count == 5000000)
        {
            prms.set_value(m);
        }
        count++;
    }
};

void gate::publishMaker()
{
    maker_.header.frame_id = "world";
    maker_.ns = "gate";
    maker_.id = 0;
    maker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    maker_.action = visualization_msgs::Marker::ADD;
    maker_.mesh_resource = mesh_;

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
    target_pub_.publish(maker_);
};