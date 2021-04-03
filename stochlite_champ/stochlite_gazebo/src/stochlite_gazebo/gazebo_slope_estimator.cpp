// written by github@aditya-shirwatkar

#include "stochlite_gazebo/gazebo_slope_estimator.h"
#include "stochlite_gazebo/tools/slope_estimator.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace stochlite {
    
    void GazeboSlopeEstimator::print_vectors(std::vector<double> const &input)
    {
        std::cout.precision(3);
        for (auto const& i: input) {
            std::cout << i << endl;
        }
    }
 
    void GazeboSlopeEstimator::print_info(){

        cout << "############" << endl;
        cout << "IMU roll, pitch and yaw in degrees" << endl;
        cout << "imu roll -> " << imu_rpy[0] * 180/PI << endl;
        cout << "imu pitch -> " << imu_rpy[1] * 180/PI << endl;
        cout << "imu yaw -> " << imu_rpy[2] * 180/PI << endl;

        cout << "------------" << endl;
        cout << "Slope Values in degrees" << endl;
        cout << "slope roll -> " << roll_median << endl;
        cout << "slope pitch -> " << pitch_median << endl;
        cout << "slope yaw -> " << yaw_median << endl;
        cout << "############" << endl;

    }

    void GazeboSlopeEstimator::configure_variables(){
        slope_local.resize(2);
        tof_height.resize(12);
        plane_angles.resize(3);
        imu_slope_esti.resize(2);
        imu_rpy.resize(3);
        imu_slope_esti[0] = 0;
        imu_slope_esti[1] = 0;

        /*
            Reference: http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
        */
        sub_imu_1 = nh.subscribe("/imu_1/data", 1, &GazeboSlopeEstimator::imu_callback_1, this);
        sub_tof_6 = nh.subscribe("/gazebo/range_tof_6", 1, &GazeboSlopeEstimator::tof_callback_6, this);
        sub_tof_1 = nh.subscribe("/gazebo/range_tof_1", 1, &GazeboSlopeEstimator::tof_callback_1, this);
        sub_tof_2 = nh.subscribe("/gazebo/range_tof_2", 1, &GazeboSlopeEstimator::tof_callback_2, this);
        sub_tof_3 = nh.subscribe("/gazebo/range_tof_3", 1, &GazeboSlopeEstimator::tof_callback_3, this);
        sub_tof_4 = nh.subscribe("/gazebo/range_tof_4", 1, &GazeboSlopeEstimator::tof_callback_4, this);
        sub_tof_5 = nh.subscribe("/gazebo/range_tof_5", 1, &GazeboSlopeEstimator::tof_callback_5, this);
    }

    void GazeboSlopeEstimator::imu_callback_1(const sensor_msgs::Imu& imu_1){
        imu_data_1 = imu_1;

        /*
            Convert a an imu quaternion to imu rpy using tf2 package
            Reference: https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
        */
        tf2::Quaternion quat;
        tf2::convert(imu_data_1.orientation , quat);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        imu_rpy[0] = roll;
        imu_rpy[1] = pitch;
        imu_rpy[2] = yaw;
    }

    void GazeboSlopeEstimator::tof_callback_1(const sensor_msgs::Range& tof_1){
        tof_data_1 = tof_1;
        tof_height[0] = (double) 1;                     // id of the sensor
        tof_height[1] = (double) -1 * tof_data_1.range; // -1 because the rays are projected downwards
    }

    void GazeboSlopeEstimator::tof_callback_2(const sensor_msgs::Range& tof_2){
        tof_data_2 = tof_2;
        tof_height[2] = (double) 2;
        tof_height[3] = (double) -1 * tof_data_2.range;

    }

    void GazeboSlopeEstimator::tof_callback_3(const sensor_msgs::Range& tof_3){
        tof_data_3 = tof_3;
        tof_height[4] = (double) 3;
        tof_height[5] = (double) -1 * tof_data_3.range;

    }

    void GazeboSlopeEstimator::tof_callback_4(const sensor_msgs::Range& tof_4){
        tof_data_4 = tof_4;
        tof_height[6] = (double) 4;
        tof_height[7] = (double) -1*tof_data_4.range;
    }

    void GazeboSlopeEstimator::tof_callback_5(const sensor_msgs::Range& tof_5){
        tof_data_5 = tof_5;
        tof_height[8] = (double) 5;
        tof_height[9] = (double) -1*tof_data_5.range;
    }

    void GazeboSlopeEstimator::tof_callback_6(const sensor_msgs::Range& tof_6){
        tof_data_6 = tof_6;
        tof_height[10] = (double) 6;
        tof_height[11] = (double) -1*tof_data_6.range;
    }

    std::vector<double> GazeboSlopeEstimator::estimator(){
        std::vector<double> slope_;
        slope_.resize(3);
        Vector3f slope; slope = slope_estimator(tof_height, imu_slope_esti); // call the sotchOrocos libs

        for (int i = 0; i < 3; i++) {
            slope_.at(i) = slope[i]; // convert a Vector3f to std::vector
        }

        slope_local = slope_; // store the values in the class variable

        return slope_;

    }

    void GazeboSlopeEstimator::filter(std::vector<double> rpy, double* roll_median, double* pitch_median, double* yaw_median){
        
        if (slope_roll_queue.size() <= slope_queue_size && 
            slope_pitch_queue.size() <= slope_queue_size && 
            slope_yaw_queue.size() <= slope_queue_size) 
        {
            slope_roll_queue.push(rpy[0]);
            slope_pitch_queue.push(rpy[1]);
            slope_yaw_queue.push(rpy[2]);
        }
        else { // pop out the values and update the queue
            slope_roll_queue.pop();
            slope_pitch_queue.pop();
            slope_yaw_queue.pop();
        }

        /*
            A copy of queue variables to keep original queues intact, 
            since conversion to vectors is required as median values cannot be directly obtained
        */
        queue<double> r = slope_roll_queue; 
        queue<double> y = slope_yaw_queue;
        queue<double> p = slope_pitch_queue;

        /*
            vectors that will store the queue values
        */
        std::vector<double> roll_vec;
        std::vector<double> pitch_vec;
        std::vector<double> yaw_vec;

        // perform conversion
        while (!r.empty() && !y.empty() && !p.empty()) {
            roll_vec.push_back(r.front());
            r.pop();
            yaw_vec.push_back(y.front());
            y.pop();
            pitch_vec.push_back(p.front());
            p.pop();
        }

        // used for median calculations
        size_t size_roll = roll_vec.size();
        size_t size_yaw = yaw_vec.size();
        size_t size_pitch = pitch_vec.size();

        if (size_roll == 0 || size_yaw == 0 || size_pitch == 0)
        {   
            // Undefinded
            *roll_median = 0;
            *pitch_median = 0;
            *yaw_median = 0;  
        }
        else
        {   
            // First sort the vector
            sort(roll_vec.begin(), roll_vec.end());
            sort(yaw_vec.begin(), yaw_vec.end());
            sort(pitch_vec.begin(), pitch_vec.end());

            // if even
            if (size_roll % 2 == 0)
            {
                *roll_median = (roll_vec[size_roll / 2 - 1] + roll_vec[size_roll / 2]) / 2;
            }
            else  // if odd
            {
                *roll_median = roll_vec[size_roll / 2];
            }

            if (size_yaw % 2 == 0)
            {
                *yaw_median = (yaw_vec[size_yaw / 2 - 1] + yaw_vec[size_yaw / 2]) / 2;
            }
            else 
            {
                *yaw_median = yaw_vec[size_yaw / 2];
            }

            if (size_pitch % 2 == 0)
            {
                *pitch_median = (pitch_vec[size_pitch / 2 - 1] + pitch_vec[size_pitch / 2]) / 2;
            }
            else 
            {
                *pitch_median = pitch_vec[size_pitch / 2];
            }

        }


    }

    GazeboSlopeEstimator::GazeboSlopeEstimator()
    {    
        configure_variables();
        
        ros::Rate loop_rate(10); // running at low rates to percieve the slope values in terminal

        while (ros::ok())
        {
            ros::spinOnce();
            
            imu_slope_esti[0] = imu_rpy[0];
            imu_slope_esti[1] = imu_rpy[1];

            plane_angles = estimator();

            // change to degrees
		    plane_angles[0] *= 180/PI;
		    plane_angles[1] *= 180/PI;
		    plane_angles[2] *= 180/PI;

            filter(plane_angles, &roll_median, &pitch_median, &yaw_median);

            print_info();

            loop_rate.sleep();

            /* 
                for clearing screen, helps in visualization of output
                Reference: https://stackoverflow.com/a/32008479
            */
            cout << "\033[2J\033[1;1H";
        }
    }
}