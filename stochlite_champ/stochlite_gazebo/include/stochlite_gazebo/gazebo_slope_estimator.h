// written by github@aditya-shirwatkar

#ifndef __GAZEBO_SLOPE_ESTIMATOR__
#define __GAZEBO_SLOPE_ESTIMATOR__
/*
    Slope Estimator here refers to the 
    torso RPY values of the robot,
    it uses data from ToF and IMU sensor 
*/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>


#include "stochlite_gazebo/tools/vector_op.h"
#include "stochlite_gazebo/tools/matrix_op.h"

#include <vector>
#include <queue>

using namespace std;

namespace stochlite {
    /*
        A Class to handle the ros nodes, subscribe to imu and tof sensor topics.
        The Class also has functions to call the "slope_estimator" functions from stochOrocos, 
        and few other helper functions
    */
    class GazeboSlopeEstimator
    {
    private:
        /* data */
        const int num_imu = 1;
        const int num_tof = 6;

        /* 
            A queue for storing the slope values obtained by calling slope_estimator.
            The queues are used to get a median of past "n" slope values for robustness
        */
        const int slope_queue_size = 5; 
        std::queue<double> slope_roll_queue;
        std::queue<double> slope_pitch_queue;
        std::queue<double> slope_yaw_queue;
        double roll_median; // median of past "n" values of slope values
        double yaw_median;
        double pitch_median;

        /*
            variables for storing raw sensor data (raw means values obtained directly from gazebo simulations)
        */
        sensor_msgs::Imu imu_data_1; // imu reading which we will store from ros topics        
        sensor_msgs::Range tof_data_1; // tof 1 reading which we will store from ros topics
        sensor_msgs::Range tof_data_2; // tof 2 reading which we will store from ros topics
        sensor_msgs::Range tof_data_3; // tof 3 reading which we will store from ros topics
        sensor_msgs::Range tof_data_4; // tof 4 reading which we will store from ros topics
        sensor_msgs::Range tof_data_5; // tof 5 reading which we will store from ros topics
        sensor_msgs::Range tof_data_6; // tof 6 reading which we will store from ros topics
        

        std::vector<double> tof_height; // take height value from tof_data and use it as an input to stochOrocos libs (slope_estimator function)
        std::vector<double> plane_angles; // output of stochOrocos libs
        std::vector<double> slope_local; // extra variable for ease of functioning while calling slope_estimator
        std::vector<double> imu_slope_esti; // imu data for argument of slope_estimator function
        std::vector<double> imu_rpy; // variable for getting RPY values from quaternion of imu raw data

        ros::NodeHandle nh; // standard ros node handle
        ros::Publisher pub_slope; // publisher variable for publishing the final slope values (To do!)

        ros::Subscriber sub_imu_1; // subscriber variable for subscribing to imu raw data
        
        ros::Subscriber sub_tof_1; // subscriber variable for raw tof data 
        ros::Subscriber sub_tof_2; // subscriber variable for raw tof data
        ros::Subscriber sub_tof_3; // subscriber variable for raw tof data
        ros::Subscriber sub_tof_4; // subscriber variable for raw tof data
        ros::Subscriber sub_tof_5; // subscriber variable for raw tof data
        ros::Subscriber sub_tof_6; // subscriber variable for raw tof data

    public:
        // Constructor of the class
        GazeboSlopeEstimator(/* args */);

        // callback function of imu raw data
        void imu_callback_1(const sensor_msgs::Imu& imu_1);

        // callback function of raw tof data 
        void tof_callback_1(const sensor_msgs::Range& tof_1); 
        void tof_callback_2(const sensor_msgs::Range& tof_2); 
        void tof_callback_3(const sensor_msgs::Range& tof_3); 
        void tof_callback_4(const sensor_msgs::Range& tof_4); 
        void tof_callback_5(const sensor_msgs::Range& tof_5); 
        void tof_callback_6(const sensor_msgs::Range& tof_6); 

        /* 
            main function that calls the slope_estimator() from stoch Orocos libs,
            which does all the heavy lifting of performing calculations

            return -> a vector of size 3 containing instantaneous slope values in RPY
        */
        std::vector<double> estimator();

        /* 
            function for resizing the vectors to their appropriate size,
            and defines the various subscribers and publishers to their respective callbacks
        */
        void configure_variables();

        /*
            Takes in the instantanous slope values in the form of a vector - rpy,
            Makes a queues of past "n" readings for each roll, pitch and yaw, 
            and returns the median of those queues.
            This ensures stable values are sensed.

            input -> rpy: a vector containing instantanous slope values
            output -> roll_median, pitch_median, and yaw_median
        */
        void filter(std::vector<double> rpy, double* roll_median, double* pitch_median, double* yaw_median);

        // A function that prints variable for debugging purposes
        void print_info();

        // A function for printing vectors in cpp
        void print_vectors(std::vector<double> const &input);
    };
    
    double PI = 3.14707;

}

#endif // __GAZEBO_SLOPE_ESTIMATOR__