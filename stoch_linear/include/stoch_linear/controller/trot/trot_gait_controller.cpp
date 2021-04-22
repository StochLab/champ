#include <stoch_linear/controller/trot/trot_gait_controller.cpp>
#include <stoch_linear/ik/robot_kinematics.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <stoch_linear/ik/serial3r_kinematics.h>

#define STOCHLITE_ABD_LENGTH 0.096

using namespace std;

namespace controller
{
    void TrotGaitController::setGaitConfig(double Walking_h, double Swing_h, double Leg_length, double No_of_points, double phase[], double s1)
    {
        wh_ = Walking_h;
        sh_ = Swing_h;
        leg_length_ = Leg_length;
        no_of_points_ = No_of_points;
        leg_phase_.f_r_phase = phase[FR];
        leg_phase_.f_l_phase = phase[FL];
        leg_phase_.b_r_phase = phase[BR];
        leg_phase_.b_l_phase = phase[BL];
        step_length_ = s1;

        printf(" %f , %f, %f, %f , %f , %f , %f ,%f \n", wh_, sh_, leg_length_, no_of_points_,leg_phase_.f_r_phase,leg_phase_.f_l_phase, leg_phase_.b_r_phase, leg_phase_.b_l_phase);
        return;
    }

    void TrotGaitController::getGaitConfig(double &Walking_h, double &Swing_h, double &Leg_length, double &No_of_points, double phase[]){
        Walking_h = wh_;
        Swing_h = sh_;
        Leg_length = leg_length_;
        No_of_points = no_of_points_;
        phase[FR] = leg_phase_.f_r_phase;
        phase[FL] = leg_phase_.f_l_phase;
        phase[BR] = leg_phase_.b_r_phase;
        phase[BL] = leg_phase_.b_l_phase;
    }

    double TrotGaitController::constrainTheta(double theta)
    {
        double theta1 = theta;
        theta1 = fmod(theta1, 2*no_of_points_);

        if(theta1<0)
            theta1 = theta1 + 2*no_of_points_;
        return theta1;
    }
    void TrotGaitController::updateLegTheta( double theta)
    {
        stoch2_5_.front_left.theta = constrainTheta(theta + leg_phase_.f_r_phase);
        stoch2_5_.front_right.phi = constrainTheta(theta + leg_phase_.f_r_phase);
        stoch2_5_.back_left.phi = constrainTheta(theta + leg_phase_.f_r_phase);
        stoch2_5_.back_right.phi = constrainTheta(theta + leg_phase_.f_r_phase);
    }
    void TrotGaitController::updateLegPhiVal( double *leg_phi)
    {
        stoch2_5_.front_left.phi = leg_phi[FL];
        stoch2_5_.front_right.phi = leg_phi[FR];
        stoch2_5_.back_left.phi = leg_phi[BL];
        stoch2_5_.back_right.phi = leg_phi[BR];
    }

    void TrotGaitController::updateLegStepLengthVal(double *leg_s1)
    {
        stoch2_5_.front_left.step_length = leg_s1[FL];
        stoch2_5_.front_right.step_length = leg_s1[FR];
        stoch2_5_.back_left.step_length = leg_s1[BL];
        stoch2_5_.back_right.step_length = leg_s1[BR];
    }
    void TrotGaitController::initializeElipseShift(double *xshift, double *yshift, double *zshift)
    {
        stoch2_5_.front_left.x_shift = xshift[FL];
        stoch2_5_.front_right.x_shift = xshift[FR];
        stoch2_5_.back_left.x_shift = xshift[BL];
        stoch2_5_.back_right.x_shift = xshift[BR];

        stoch2_5_.front_left.y_shift = yshift[FL];
        stoch2_5_.front_right.y_shift = yshift[FR];
        stoch2_5_.back_left.y_shift = yshift[BL];
        stoch2_5_.back_right.y_shift = yshift[BR];
        
        stoch2_5_.front_left.z_shift = zshift[FL];
        stoch2_5_.front_right.z_shift = zshift[FR];
        stoch2_5_.back_left.z_shift = zshift[BL];
        stoch2_5_.back_right.z_shift = zshift[BR];
    }

    void TrotGaitController::initializeLegState(double theta, std::vector<double> action)
    {
        double leg_s1[4], leg_phi[4], xshift[4], yshift[4], zshift[4];

        stoch2_5_.front_left.name = "f1";
        stoch2_5_.front_right.name = "fr";
        stoch2_5_.back_left.name = "bl";
        stoch2_5_.back_right.name = "br";

        updateLegTheta(theta);

        for(int i=0; i< 4; i++){
            leg_s1[i] = action.at(i);
            leg_phi[i] = action.at(4+i);
            xshift[i] = action.at(8+i);
            yshift[i] = action.at(12+i);
            zshift[i] = action.at(16+i);

        }

        updateLegPhiVal(leg_phi);

        updateLegStepLengthVal(leg_s1);

        initializeElipseShift(xshift, yshift, zshift);
    }

    void TrotGaitController::runEllipticalTrajStoch2_5(std::vector<double> action, double theta, int value, double final_bot_joint_angles[4][3])
    {
        robot::Kinematics t;
        Serial3RKinematics serial3r({0.096, 0.146, 0.172});

        std::vector<double> foot_position(3);
        std::vector<double> joint_angles(3);

        initializeLegState(theta, action);

        for(int i=0;i<4;i++){
            value = i;
            if(value==FR)
                leg = &stoch2_5_.front_right;
            else if(value==FL)
                leg = &stoch2_5_.front_left;
            else if(value==BR)
                leg = &stoch2_5_.back_right;
            else if(value==BL)
                leg = &stoch2_5_.back_left;

            double leg_theta, leg_r, x, y, z;
            int flag;

            leg_theta = (leg->theta/ (2*no_of_points_)) * 2 * M_PI;

            leg_r = leg->step_length / 2;

            x = -leg_r * cos(leg_theta) + leg->x_shift;

            if(leg_theta > M_PI){
                flag = 0;
            }
            else{ 
                flag = 1;
            }
            z = sh_*sin(leg_theta)*flag + wh_ + leg->z_shift;

            leg->x = x*cos(leg->phi);
            leg->z = z;
            leg->y = x* - sin(leg->phi);

            if(i==FR or i==BR)
                leg->y = leg->y - STOCHLITE_ABD_LENGTH + leg->y_shift;
            else
                leg->y = leg->y + STOCHLITE_ABD_LENGTH + leg->y_shift;
            
            foot_position[0] = leg->x;
            foot_position[1] = leg->y;
            foot_position[2] = leg->z;

            serial3r.inverseKinematics(leg->name, foot_position, '>', joint_angles);

            final_bot_joint_angles[i][0] = joint_angles[0];
            final_bot_joint_angles[i][1] = joint_angles[1];
            final_bot_joint_angles[i][2] = joint_angles[2];


        }
    }

    void TrotGaitController::runEllipticalTrajLeg(std::vector<double> action, double theta, int value, double *final_leg_joint_angles)
    {
        robot::Kinematics t;

        Serial3RKinematics serial3r({0.096, 0.146, 0.172});

        std::vector<double> foot_position(3);
        std::vector<double> joint_angles(3);

        initializeLegState(theta, action);

        if(value == FR)
            leg = &stoch2_5_.front_right;
        else if(value == FL)
            leg = &stoch2_5_.front_left;
        else if(value == BR)
            leg = &stoch2_5_.back_right;
        else if(value == BL)
            leg = &stoch2_5_.back_left;
        
        double leg_theta, leg_r, x, y, z;
        int flag;

        leg_theta = (leg->theta / (2*no_of_points_)) * 2 * M_PI;
        leg_r = leg->step_length / 2;

        x = -leg_r * cos(leg_theta);

        if(leg_theta > M_PI)
            flag = 0;
        else 
            flag = 1;

        z = sh_*sin(leg_theta) * flag + wh_ + leg->z_shift;

        leg->x = x*cos(leg->phi);
        leg->z = z;
        leg->y = x*-sin(leg->phi);

        if(value == FR or value == BR)
            leg->y = leg->y - STOCHLITE_ABD_LENGTH + leg->y_shift;
        else
            leg->y = leg->y + STOCHLITE_ABD_LENGTH + leg->y_shift;

        
        serial3r.inverseKinematics(leg->name, foot_position, '>', joint_angles);

        final_leg_joint_angles[0] = joint_angles[0];
        final_leg_joint_angles[1] = joint_angles[1];
        final_leg_joint_angles[2] = joint_angles[2];
        
    }

}