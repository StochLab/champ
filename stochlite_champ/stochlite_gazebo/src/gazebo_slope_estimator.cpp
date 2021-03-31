// #include <ros/ros.h>
#include "gazebo_slope_estimator.h"
// #include <cstdio>
// #include <sensor_msgs/Range.h>
// #include <sensor_msgs/Imu.h>
// #include "tools/vector_op.h"
// #include "tools/matrix_op.h"
// #include "tools/rotation.h"
// #include "io/tof/tof_coordinates.h"
// #include "tools/slope_estimator.h"

using namespace std;

namespace stochlite {

    void GazeboSlopeEstimator::ImuCallback0(const sensor_msgs::Imu& imu_0){
        imu_data_0 = imu_0;
        // std::cout << imu_data_0;
    }

    void GazeboSlopeEstimator::TofCallback0(const sensor_msgs::Range& tof_0){
        tof_data_0 = tof_0;
    }


    void GazeboSlopeEstimator::TofCallback1(const sensor_msgs::Range& tof_1){
        tof_data_1 = tof_1;
    }

    void GazeboSlopeEstimator::TofCallback2(const sensor_msgs::Range& tof_2){
        tof_data_2 = tof_2;
    }

    void GazeboSlopeEstimator::TofCallback3(const sensor_msgs::Range& tof_3){
        tof_data_3 = tof_3;
    }

    void GazeboSlopeEstimator::TofCallback4(const sensor_msgs::Range& tof_4){
        tof_data_4 = tof_4;
    }

    void GazeboSlopeEstimator::TofCallback5(const sensor_msgs::Range& tof_5){
        tof_data_5 = tof_5;
    }

    GazeboSlopeEstimator::GazeboSlopeEstimator(/* args */)
    {
        // sub_imu_0 = nh.subscribe("/imu_0/data", 1000, &GazeboSlopeEstimator::ImuCallback0, this);
        // sub_tof_0 = nh.subscribe("/gazebo/range_tof_0", 1000, &GazeboSlopeEstimator::TofCallback0, this);

        sub_imu_0 = nh.subscribe("/imu_0/data", 1, &GazeboSlopeEstimator::ImuCallback0, this);
        // message_filters::TimeSynchronizer<sensor_msgs::Imu> sync_imu(sub_imu_0, 10);

        sub_tof_0 = nh.subscribe("/gazebo/range_tof_0", 1, &GazeboSlopeEstimator::TofCallback0, this);
        sub_tof_1 = nh.subscribe("/gazebo/range_tof_1", 1, &GazeboSlopeEstimator::TofCallback1, this);
        sub_tof_2 = nh.subscribe("/gazebo/range_tof_2", 1, &GazeboSlopeEstimator::TofCallback2, this);
        sub_tof_3 = nh.subscribe("/gazebo/range_tof_3", 1, &GazeboSlopeEstimator::TofCallback3, this);
        sub_tof_4 = nh.subscribe("/gazebo/range_tof_4", 1, &GazeboSlopeEstimator::TofCallback4, this);
        sub_tof_5 = nh.subscribe("/gazebo/range_tof_5", 1, &GazeboSlopeEstimator::TofCallback5, this);

        // message_filters::TimeSynchronizer<sensor_msgs::Imu,
        //                                   sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range,
        //                                   sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> 
        //                                   sync_tof(sub_imu_0,
        //                                            sub_tof_0, sub_tof_1, sub_tof_2,
        //                                            sub_tof_3, sub_tof_4, sub_tof_5, 7);

        // sync_imu.registerCallback(&GazeboSlopeEstimator::ImuCallback, this);

        // sync_tof.registerCallback(&GazeboSlopeEstimator::TofCallback, this);

        // char format_string[100];
        // int retVal_format_string;

        // for (int i = 0; i < 6; i++)
        // {
        //     /* code */
        //     retVal = sprintf(format_string, "/gazebo/range_tof_%d", i);
        //     sub_tof[i] = nh.subscribe(format_string, &GazeboSlopeEstimator::TofCallback, this);

        // }

        ros::Rate loop_rate(10);
    
        while (ros::ok())
        {
        
            ros::spinOnce();

            loop_rate.sleep();
        
        }
    }
}