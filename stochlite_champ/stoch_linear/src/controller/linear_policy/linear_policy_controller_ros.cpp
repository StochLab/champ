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
using namespace stochlite;

namespace controller
{

    void LinearPolicyRos::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        imu_data = *msg; 
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
#if 0
        for(int i=0; i<past_imu_values.size(); i++)
        {
            cout << "imu values "<< past_imu_values[i] << endl;
        }
#endif
    }

    void LinearPolicyRos::printVariables(std::vector<double>& action)
    {
        slope_est.print_info(0);

        for(int i=0; i<action.size() ; i++ )
        {
            printf("action at: %d is: %f \n",i,action[i]);
        }

        cout << "############" << endl;
    }

    void LinearPolicyRos::linearControl(bool median, std::vector<double>& action)
    {
        slope_est.allFunctions(median);

        state  = linear_controller.settingState(past_imu_values,slope_est.plane_angles);
        action = linear_controller.linearPolicy(state,1); // The joystick value has been forced to 5m/s but need to create a subscriber to the joystick topic of the framework
        
        // need to publish the action to the trajectory node for now printing    
        printVariables(action);        
    }

    LinearPolicyRos::LinearPolicyRos()       
    {
        bool median=false;
        past_imu_values.resize(9);
        sub_imu = n.subscribe("/imu_1/data",1,&LinearPolicyRos::imuCallback, this);

        ros::Rate loop_rate(10);

        // while(ros::ok())
        // {
        //     ros::spinOnce();
        //     linearControl(median,action);
        //     loop_rate.sleep();
        // }
        
    }


}
