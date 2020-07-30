#include <iostream>
#include <math.h>
#include <string.h>
#include <gate_detector.h>

Gate::Gate(ros::Publisher &target_pub, Vector3d ini_pos, ros::Time trigger_time, int id, std::shared_ptr<int> &segment_pt, std::shared_ptr<Matrix3d> &target_ptr) : target_pub_(target_pub), ini_pos_(ini_pos), trigger_time_(trigger_time), id_(id), segment_pt_(segment_pt), target_ptr_(target_ptr){};

void Gate::updateTarget(std::promise<Matrix3d> &&prms)
{
    std::unique_lock<std::mutex> lck(_mutex);
    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!ready_)
            cv_->wait(lck);
        updateState();
        if ((id_ == *segment_pt_) && (!is_triggered_))
        {
            Matrix3d m;
            m << 1, 2, 3,
                4, 5, 6,
                7, 8, 4;
            *target_ptr_ = m;
            prms.set_value(m);
            is_triggered_ = true;
        }
        ros::spinOnce();
    }
};

void Gate::updateState()
{

    double duration = (ros::Time::now() - trigger_time_).toSec();
    auto deltaPos = Vector3d(0.5 * sin(duration), 0.5 * cos(duration), 0);
    position_ = ini_pos_ + deltaPos;
    Gate::publishMaker();
    ros::spinOnce();
};
void Gate::setCV(std::shared_ptr<::condition_variable> &cv)
{
    cv_ = cv;
};
void Gate::publishMaker()
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