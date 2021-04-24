#include "stoch_linear/controller/linear_policy/linear_policy_controller_core.h"

using namespace std;

namespace controller 
{
    std::vector<double> LinearPolicyController::pastImuValues(std::vector<double> current_imu_readings, std::vector<double> &past_imu_readings)
    {
        for(int i=0 ; i < past_imu_readings.size()-current_imu_readings.size() ; i++)
        {
          past_imu_readings[i] = past_imu_readings[i+3];
        }

        for(int i=0 ; i < current_imu_readings.size() ; i++)
        {
          past_imu_readings[i+6] = current_imu_readings[i];
        }
#if 0
        for(int i=0; i < past_imu_readings.size() ; i++)
        {
          printf("imu value: %f",past_imu_readings[i]*180/M_PI);
        }	

        printf("\n");
#endif

        return past_imu_readings;
    }

    std::vector<double> LinearPolicyController::settingState(std::vector<double> past_imu_readings, std::vector<double> current_slope_readings)
    {

        std::vector<double> local_state;
        local_state.resize(11);

        local_state.at(0)  = past_imu_readings.at(0);
        local_state.at(1)  = past_imu_readings.at(1);
        local_state.at(2)  = past_imu_readings.at(2);
        local_state.at(3)  = past_imu_readings.at(3);
        local_state.at(4)  = past_imu_readings.at(4);
        local_state.at(5)  = past_imu_readings.at(5);
        local_state.at(6)  = past_imu_readings.at(6);
        local_state.at(7)  = past_imu_readings.at(7);
        local_state.at(8)  = past_imu_readings.at(8);
        local_state.at(9)  = current_slope_readings.at(0);
        local_state.at(10) = current_slope_readings.at(1);

        return local_state;
    }

    std::vector<double> LinearPolicyController::linearPolicy(std::vector<double> state, double command_velocity)
    {
        std::vector<double> action;
        std::vector<double> action_transformed;
        std::vector<double> slope_values;

        slope_values.resize(2);
        action.resize(20);

        // Sending Slope Values to Action Transform 
        slope_values[0] = state[9];
        slope_values[1] = state[10];

        double robot_param;

        for(int i=0; i<20 ; i++)
        {
          robot_param = 0; 

          for(int j=0; j<11; j++)
          {
            robot_param += policy[i][j] * state[j];
          }

          action[i] = robot_param; 
        }

#if 0
        //step length
        action[0] = 0.3;
        action[1] = 0.3;
        action[2] = 0.3;
        action[3] = 0.3;

        //drive angle
        action[4] = 0;
        action[5] = 0;
        action[6] = 0;
        action[7] = 0;

        //x-shift
        action[8] = 0.05;
        action[9] = 0.05;
        action[10] = 0.05;     
        action[11] = 0.05;

        //y-shift
        action[12] = 0.2;
        action[13] = 0.2;
        action[14] = 0.2;
        action[15] = 0.2;

        //z-shift
        action[16] = 0;
        action[17] = 0;
        action[18] = 0;
        action[19] = 0;
#endif            

        action_transformed = actionTransform(action, command_velocity, slope_values);

#if 0
        for(int i =0 ; i < action.size(); i++)
        {
          printf("position i: %d action: %f\n",i,action_transformed.at(i));
        }
          printf("\n");
#endif

        return action_transformed;        
    }

    std::vector<double> LinearPolicyController::actionTransform(std::vector<double> action, double command_velocity,std::vector<double> slope_values)
    {
        std::vector<double> transformed_action;
        transformed_action.resize(20);

        double step_length_offset;
        double sp_pitch_offset;
        double fwd_vel_offset;

        /* here we are giving the values as fl, fr, bl, br as this is the 
         * convention for the motor angles
         */

        step_length_offset = 0.3*command_velocity;
        sp_pitch_offset    = 0*0.075*slope_values.at(1)*180/M_PI/15; //0.075
        fwd_vel_offset     = -0.02*command_velocity;

        //step length FL,FR,BL,BR
        transformed_action.at(0) = 1*((action.at(0)+1)/2*0.08+step_length_offset)/*+step_length_offset*/;
        transformed_action.at(1) = 1*((action.at(1)+1)/2*0.08+step_length_offset)/*+step_length_offset*/;
        transformed_action.at(2) = 1*((action.at(2)+1)/2*0.08+step_length_offset)/*+step_length_offset*/;
        transformed_action.at(3) = 1*((action.at(3)+1)/2*0.08+step_length_offset)/*+step_length_offset*/;

        //drive angle  FL,FR,BL,BR
        transformed_action.at(4) = 1*action.at(4);
        transformed_action.at(5) = 1*action.at(5);
        transformed_action.at(6) = 1*action.at(6);
        transformed_action.at(7) = 1*action.at(7);

        //x-shift FL,FR,BL,BR
        transformed_action.at(8)  = 1*((action.at(8)  + sp_pitch_offset)*1 + fwd_vel_offset);
        transformed_action.at(9)  = 1*((action.at(9)  + sp_pitch_offset)*1 + fwd_vel_offset);
        transformed_action.at(10) = 1*((action.at(10) + sp_pitch_offset)*1 + fwd_vel_offset);
        transformed_action.at(11) = 1*((action.at(11) + sp_pitch_offset)*1 + fwd_vel_offset);

        //y-shift FL,FR,BL,BR
        transformed_action.at(12) = 1*-((action.at(12)*0.14 + 0.05))*1/*-0.05*/;
        transformed_action.at(13) = 1*(  action.at(13)*0.14 + 0.05 )*1/*+0.05*/;
        transformed_action.at(14) = 1*-((action.at(14)*0.14 + 0.05))*1/*-0.05*/;
        transformed_action.at(15) = 1*(  action.at(15)*0.14 + 0.05 )*1/*+0.05*/;

        //z-shift FL,FR,BL,BR
        transformed_action.at(16) = 1*0.1*action.at(16);
        transformed_action.at(17) = 1*0.1*action.at(17);
        transformed_action.at(18) = 1*0.1*action.at(18);
        transformed_action.at(19) = 1*0.1*action.at(19);

        return transformed_action;
    }
}