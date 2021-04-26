#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "stoch_linear/controller/linear_policy/gazebo_slope_estimator.h"
#include "stoch_linear/controller/linear_policy/linear_policy_controller_core.h"

using namespace stochlite;
namespace controller
{
    class LinearPolicyRos
    {
        private:

            sensor_msgs::Imu imu_data; // imu reading which we will store from ros topics
            ros::NodeHandle n;
            ros::Subscriber sub_imu;
            std::vector<double> past_imu_values;        // variable to store the past imu values 
            std::vector<double> state;                  // variable to store the state 
            std::vector<double> action;                 // variable to store the action
            LinearPolicyCore linear_controller;         // Class that contains the functions required for linear policy control
            GazeboSlopeEstimator slope_est;             // Class that contains the functions required to find the slope

        public:

            /**
            * \brief Constructor
            */
            LinearPolicyRos();

            /**
            * \brief Callback to the Imu data and storing upto last 3 time step values
            */
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
           
            /**
            * \brief Function to print all variables 
            */
            void printVariables(std::vector<double>& action);

            /**
            * \brief Function to start the controller 
            */           
            void linearControl(bool median,std::vector<double>& action);



    };
}


