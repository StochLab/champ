/*
 * @file linear_policy_control_core.h
 *
 * Created : 24 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#ifndef __LINEAR_POLICY_CONTROLLER__
#define __LINEAR_POLICY_CONTROLLER__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vector>

using namespace std;

namespace controller 
{
    class LinearPolicyCore
    {
        public:

            /**
             * \brief Function to Store the past 3 imu values
             */
            std::vector<double> pastImuValues(std::vector<double> current_imu_readings, std::vector<double> past_imu_readings);

            /**
             * \brief Function setting the robot state using imu and slope values
             */
            std::vector<double> settingState(std::vector<double> past_imu_readings, std::vector<double> current_slope_readings);

            /**
             * \brief Function to find the action transformation
             */
            std::vector<double> linearPolicy(std::vector<double> state,double command_velocity);

            /**
             * \brief Function to apply transforms to the action
             */
            std::vector<double> actionTransform(std::vector<double> action, double command_velocity,std::vector<double> slope_values);

            /**
             * \brief We include the trained policy here
             */
            // #include "stoch_linear/controller/trot/policy_slopes_flatg.h" // Most stable on flat grounds only
            #include "stoch_linear/controller/trot/policy_slopes_joystick3.h" // Most stable but steers off
            // #include "stoch_linear/controller/trot/policy_18_26F3.h" // Least jittery
            // #include "stoch_linear/controller/trot/policy_42_26F3.h" // Not performing well
            // #include "stoch_linear/controller/trot/policy_60_26F3.h" // Worst of the 3
    };
}
#endif // __LINEAR_POLICY_CONTROLLER__
