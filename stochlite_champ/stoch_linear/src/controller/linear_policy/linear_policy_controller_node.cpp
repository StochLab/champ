/*
 * @file linear_policy_controller_node.cpp
 *
 * Created : 24 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "stoch_linear/controller/linear_policy/linear_policy_controller_core.h"

using namespace std;
using namespace controller;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"linear_policy_controller_node");
    ros::NodeHandle n;

    LinearPolicyController linear_controller;

    
    return 0;
}
