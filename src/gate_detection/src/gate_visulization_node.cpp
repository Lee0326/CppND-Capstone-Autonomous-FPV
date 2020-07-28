#include <iostream>
#include <math.h>
#include <string.h>
#include <gate.h>

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle n;
    // initiize the gate class
    Vector3d init_position(3, 3, 1);
    std::promise<Matrix3d> prms;
    std::future<Matrix3d> ftr = prms.get_future();
    ros::Publisher targetPub = n.advertise<visualization_msgs::Marker>("gate_marker", 10);
    auto trigger_time = ros::Time::now();
    gate gate1(targetPub, init_position, trigger_time, 0);
    std::thread t(&gate::updateState, &gate1, std::move(prms));
    Matrix3d result = ftr.get();
    std::cout << "the result from the gate thread: " << result(0, 0) << std::endl;
    t.join();
    return 0;
}
