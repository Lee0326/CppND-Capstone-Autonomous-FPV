#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <gate_detector.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
class Trajectory
{
private:
    std::vector<std::shared_ptr<Gate>> gate_targets_;
    std::vector<std::promise<Matrix3d>> proms_vector_;
    std::vector<std::future<Matrix3d>> ftrs_vector_;
    std::vector<Vector3d> position_vector_;
    std::vector<ros::Publisher> publisers_;
    std::vector<std::thread> threads_;
    std::shared_ptr<std::condition_variable> cv_;
    std::shared_ptr<Matrix3d> target_ptr_;
    std::shared_ptr<int> segment_pt_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    int loop_count_ = 0;
    double hit_count_ = 0;

public:
    Trajectory(ros::NodeHandle &nh, std::vector<Vector3d> &position_vector, std::vector<std::thread> &&threads) : nh_(nh), position_vector_(position_vector), threads_(std::move(threads))
    {
        cv_ = std::make_shared<std::condition_variable>();
        segment_pt_ = std::make_shared<int>(1);
        odom_sub_ = nh_.subscribe("/visual_slam/odom", 10, &Trajectory::odom_callback, this,
                                  ros::TransportHints().tcpNoDelay());
        Matrix3d target;
        target << 1, 2, 3,
            4, 5, 6,
            7, 8, 9;
        target_ptr_ = std::make_shared<Matrix3d>(target);
    };
    ~Trajectory()
    {
        for (int i = 0; i < threads_.size(); i++)
        {
            threads_[i].join();
        }
    };
    void setGates()
    {
        auto trigger_time = ros::Time::now();
        for (int i = 0; i < position_vector_.size(); i++)
        {
            ros::Publisher targetPub = nh_.advertise<visualization_msgs::Marker>("gate_marker" + std::to_string(i), 10);
            publisers_.push_back(targetPub);
            std::promise<Matrix3d> prms;
            std::future<Matrix3d> ftr = prms.get_future();
            proms_vector_.push_back(std::move(prms));
            ftrs_vector_.push_back(std::move(ftr));
            gate_targets_.push_back(std::make_shared<Gate>(targetPub, position_vector_[i], trigger_time, (i + 1), segment_pt_, target_ptr_));
        }
    };
    void launchThreads()
    {
        setGates();

        for (int i = 0; i < gate_targets_.size(); i++)
        {
            gate_targets_[i]->setCV(cv_);
            gate_targets_[i]->ready_ = true;
            threads_.push_back(std::thread(&Gate::updateTarget, gate_targets_[i], std::move(proms_vector_[i])));
        }
        cv_->notify_all();

        Matrix3d result;
        result = ftrs_vector_[*segment_pt_].get();
        std::cout << "the changed (2,2): " << target_ptr_->coeff(2, 2) << std::endl;
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
    {

        const Vector3d position(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);
        const Vector3d velocity(odom->twist.twist.linear.x,
                                odom->twist.twist.linear.y,
                                odom->twist.twist.linear.z);
        int last_segment = *segment_pt_;
        for (int i = 0; i < position_vector_.size(); i++)
        {
            double dist_gate = (position - position_vector_[i]).norm();
            if (dist_gate < 2)
            {
                *segment_pt_ = ((i + 1) % 4) + 1;
            }
        }
        int last_loop = loop_count_;
        if (last_segment != *segment_pt_)
        {
            hit_count_ += 1;
            loop_count_ = hit_count_ / 4;
        }
        if (last_loop != loop_count_)
        {
            for (int i = 0; i < gate_targets_.size(); i++)
            {
                gate_targets_[i]->setNewLoop();
            }
        }

        // std::cout << "The drone has run " << loop_count_ << " loops" << std::endl;
        std::cout << target_ptr_->coeff(0, 0) << " " << target_ptr_->coeff(1, 0) << " " << target_ptr_->coeff(2, 0) << std::endl;

        //std::cout << "the gate on target: " << *segment_pt_ << std::endl;
    };
    void publishCMD()
    {
        quadrotor_msgs::PositionCommand pos_cmd;
        while (ros::ok())
        {
            cmd_pub_.publish(pos_cmd);
        }
    }
    void join()
    {
        for (int i = 0; i < threads_.size(); i++)
        {
            threads_[i].join();
        }
    }
};

#endif
