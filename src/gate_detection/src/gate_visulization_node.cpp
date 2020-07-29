#include <iostream>
#include <math.h>
#include <string.h>
#include <gate_detector.h>

std::shared_ptr<std::condition_variable> cv = std::make_shared<std::condition_variable>();
void createGateDetector(ros::NodeHandle &n, std::vector<std::shared_ptr<Gate>> &gate_targets, std::vector<std::promise<Matrix3d>> &proms_vector,
                        std::vector<std::future<Matrix3d>> &ftrs_vector, std::shared_ptr<int> &segment_pt)
{
    Matrix3d target;
    target << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    std::shared_ptr<Matrix3d> target_ptr = std::make_shared<Matrix3d>(target);

    Vector3d init_position0(3, 3, 1);
    Vector3d init_position1(-3, -3, 1);
    Vector3d init_position2(3, -3, 1);
    Vector3d init_position3(-3, 3, 1);

    ros::Publisher targetPub0 = n.advertise<visualization_msgs::Marker>("gate_marker0", 10);
    ros::Publisher targetPub1 = n.advertise<visualization_msgs::Marker>("gate_marker1", 10);
    ros::Publisher targetPub2 = n.advertise<visualization_msgs::Marker>("gate_marker2", 10);
    ros::Publisher targetPub3 = n.advertise<visualization_msgs::Marker>("gate_marker3", 10);
    auto trigger_time = ros::Time::now();

    std::promise<Matrix3d> prms0;
    std::promise<Matrix3d> prms1;
    std::promise<Matrix3d> prms2;
    std::promise<Matrix3d> prms3;

    std::future<Matrix3d> ftr0 = prms0.get_future();
    std::future<Matrix3d> ftr1 = prms1.get_future();
    std::future<Matrix3d> ftr2 = prms2.get_future();
    std::future<Matrix3d> ftr3 = prms3.get_future();

    gate_targets.push_back(std::make_shared<Gate>(targetPub0, init_position0, trigger_time, 0, segment_pt, target_ptr));
    gate_targets.push_back(std::make_shared<Gate>(targetPub1, init_position1, trigger_time, 1, segment_pt, target_ptr));
    gate_targets.push_back(std::make_shared<Gate>(targetPub2, init_position2, trigger_time, 2, segment_pt, target_ptr));
    gate_targets.push_back(std::make_shared<Gate>(targetPub3, init_position3, trigger_time, 3, segment_pt, target_ptr));
    for (int i = 0; i < gate_targets.size(); i++)
    {
        gate_targets[i]->setCV(cv);
        //gate_targets[i]->updateState();
    }

    proms_vector.push_back(std::move(prms0));
    proms_vector.push_back(std::move(prms1));
    proms_vector.push_back(std::move(prms2));
    proms_vector.push_back(std::move(prms3));

    ftrs_vector.push_back(std::move(ftr0));
    ftrs_vector.push_back(std::move(ftr1));
    ftrs_vector.push_back(std::move(ftr2));
    ftrs_vector.push_back(std::move(ftr3));
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle n;
    // initiize the gate class
    std::vector<std::shared_ptr<Gate>> gate_targets;
    std::vector<std::promise<Matrix3d>> proms_vector;
    std::vector<std::future<Matrix3d>> ftrs_vector;
    std::vector<std::thread> threads;
    std::shared_ptr<int> segment_pt = std::make_shared<int>(0);

    createGateDetector(n, gate_targets, proms_vector, ftrs_vector, segment_pt);

    // for (int i = 0; i < gate_targets.size(); i++)
    // {
    //     gate_targets[i]->notifyConditionVariavle();
    // }
    for (int i = 0; i < gate_targets.size(); i++)
    {
        gate_targets[i]->ready_ = true;
    }
    cv->notify_all();
    for (int i = 0; i < gate_targets.size(); i++)
    {
        threads.push_back(std::thread(&Gate::updateTarget, gate_targets[i], std::move(proms_vector[i])));
    }

    Matrix3d result;
    result = ftrs_vector[*segment_pt].get();

    std::cout << "the result from the gate thread: " << result(2, 2) << std::endl;

    std::cout << "the changed (2,2) is " << gate_targets[0]->target_ptr_->coeff(2, 2) << std::endl;
    std::for_each(threads.begin(), threads.end(), [&threads](std::thread &t) {
        t.join();
    });

    return 0;
}
