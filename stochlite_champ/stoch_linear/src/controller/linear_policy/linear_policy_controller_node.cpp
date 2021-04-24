/*
 * @file linear_policy_controller_node.cpp
 *
 * Created : 24 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "stoch_linear/controller/linear_policy/gazebo_slope_estimator.h"
#include "stoch_linear/controller/linear_policy/linear_policy_controller_core.h"
#include "stoch_linear/controller/linear_policy/linear_policy_controller_ros.h"

using namespace std;
using namespace controller;

int main(int argc, char **argv)
{
    bool median=false;
    ros::init(argc, argv,"linear_policy_controller_node");
    // ros::Rate loop_rate(10);

    LinearPolicyRos node; 
    while(1)
    {
        node.start(0);
        // loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
