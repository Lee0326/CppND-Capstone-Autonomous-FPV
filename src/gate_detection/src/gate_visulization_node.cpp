#include <iostream>
#include <math.h>
#include <string.h>
#include <gate_detector.h>
#include <trajectory_generator.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    std::vector<std::shared_ptr<Gate>> gate_targets;
    std::vector<std::promise<Matrix3d>> proms_vector;
    std::vector<std::future<Matrix3d>> ftrs_vector;
    std::vector<std::thread> threads;
    std::shared_ptr<int> segment_pt = std::make_shared<int>(0);
    Vector3d init_position0(3, 3, 1);
    Vector3d init_position1(-3, 3, 1);
    Vector3d init_position2(-3, -3, 1);
    Vector3d init_position3(3, -3, 1);
    std::vector<Vector3d> position_vector;
    position_vector.push_back(init_position0);
    position_vector.push_back(init_position1);
    position_vector.push_back(init_position2);
    position_vector.push_back(init_position3);

    Trajectory traj(n, position_vector, std::move(threads));
    traj.launchThreads();
    return 0;
}
