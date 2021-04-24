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

using namespace std;
using namespace controller;
using namespace stochlite;


/**
* \brief The class that contains the required classes and functions required for the ros interface 
*/
class LinearPolicyNode
{
    public:
        /**
        * \brief Callback to the Imu data and storing upto last 3 time step values
        */
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
        {
            // will store imu values as roll, pitch, yaw
            double roll,pitch,yaw;
            std::vector<double> current_imu_values(3);
            
            tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w );
            tf::Matrix3x3 eular(quat);

            eular.getRPY(roll,pitch,yaw);

            current_imu_values[0] = roll;
            current_imu_values[1] = pitch;
            current_imu_values[2] = yaw;
            
            past_imu_values = linear_controller.pastImuValues(current_imu_values,past_imu_values);
#if 1
            for(int i=0; i<past_imu_values.size(); i++)
            {
                cout << "imu values "<< past_imu_values[i] << endl;
            }
#endif
        }

        std::vector<double> past_imu_values;        // variable to store the past imu values 
        LinearPolicyController linear_controller;   // Class that contains the functions required for linear policy control
        GazeboSlopeEstimator slope_est;             // Class that contains the functions required to find the slope 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"linear_policy_controller_node");
    ros::NodeHandle n;
    LinearPolicyNode listener;

    listener.past_imu_values.resize(9);

    ros::Subscriber sub_imu = n.subscribe("/imu_1/data",1,&LinearPolicyNode::imuCallback, &listener);

    ros::spin();


    return 0;
}
