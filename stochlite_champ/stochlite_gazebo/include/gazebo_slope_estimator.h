// written by github@aditya-shirwatkar

#ifndef __GAZEBO_SLOPE_ESTIMATOR__
#define __GAZEBO_SLOPE_ESTIMATOR__

/*
    Slope Estimator here refers to the 
    torso RPY values of the robot,
    it uses data from ToF and IMU sensor 
*/

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/connection.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include "tools/vector_op.h"
#include "tools/matrix_op.h"
#include "tools/rotation.h"
#include "io/tof/tof_coordinates.h"
#include "tools/slope_estimator.h"

using namespace std;

namespace stochlite {

    class GazeboSlopeEstimator
    {
    private:
        /* data */
        int num_imu = 1;
        int num_tof = 6;

        // std::map<int, sensor_msgs::Range> tof_data; // tof reading which we will store from ros topics, and a dictionary because 6 sensors

        sensor_msgs::Imu imu_data_0; // imu reading which we will store from ros topics

        sensor_msgs::Range tof_data_0; // tof 0 reading which we will store from ros topics
        sensor_msgs::Range tof_data_1; // tof 0 reading which we will store from ros topics
        sensor_msgs::Range tof_data_2; // tof 0 reading which we will store from ros topics
        sensor_msgs::Range tof_data_3; // tof 0 reading which we will store from ros topics
        sensor_msgs::Range tof_data_4; // tof 0 reading which we will store from ros topics
        sensor_msgs::Range tof_data_5; // tof 0 reading which we will store from ros topics
        
        std::vector<double> tof_height; // take height value from tof_data and use it as an input to stochOrocos libs
        std::vector<double> plane_angles; // output of stochOrocos libs
        // std::vector<double> slope_local; // tof reading which we will store from ros topics

        ros::NodeHandle nh;
        ros::Publisher pub_slope;

        ros::Subscriber sub_imu_0;
        
        ros::Subscriber sub_tof_0; // subscriber_function
        ros::Subscriber sub_tof_1; // subscriber_function
        ros::Subscriber sub_tof_2; // subscriber_function
        ros::Subscriber sub_tof_3; // subscriber_function
        ros::Subscriber sub_tof_4; // subscriber_function
        ros::Subscriber sub_tof_5; // subscriber_function

    public:
        GazeboSlopeEstimator(/* args */);

        void ImuCallback0(const sensor_msgs::Imu& imu_0);
        void TofCallback0(const sensor_msgs::Range& tof_0);
        void TofCallback1(const sensor_msgs::Range& tof_1);
        void TofCallback2(const sensor_msgs::Range& tof_2);
        void TofCallback3(const sensor_msgs::Range& tof_3);
        void TofCallback4(const sensor_msgs::Range& tof_4);
        void TofCallback5(const sensor_msgs::Range& tof_5);
        
    };
    
    
}

#endif // __GAZEBO_SLOPE_ESTIMATOR__