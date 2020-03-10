#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

class TrajectoryPlanner
{
    QuadrupedLeg *leg_;

    float swing_height_;
    float step_length_;
    float stance_depth_;
    unsigned int total_control_points_;

    Transformation prev_foot_position_;

    float factorial_[13];
    float ref_control_points_x_[12];
    float ref_control_points_y_[12];
    float control_points_x_[12];
    float control_points_y_[12];
    
    float height_ratio_;
    float length_ratio_;

    // float getGaitCycleCount(float target_velocity);
    void updateControlPointsHeight(float swing_height)
    {
        float new_height_ratio = swing_height / 0.15;
        
        if(height_ratio_ != new_height_ratio)
        {
            height_ratio_ = new_height_ratio;
            for(unsigned int i = 0; i < 12; i++)
            {
                control_points_y_[i] = -((ref_control_points_y_[i] * height_ratio_) + (0.5 * height_ratio_));
            }    
        }
    }

    void updateControlPointsLength(float step_length)
    {
        float new_length_ratio = step_length / 0.4;
        
        if(length_ratio_ != new_length_ratio)
        {
            length_ratio_ = new_length_ratio;
            for(unsigned int i = 0; i < 12; i++)
            {
                if(i == 0)
                    control_points_x_[i] = -step_length / 2.0;
                else if(i == 11)
                    control_points_x_[i] = step_length / 2.0;
                else
                    control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;   
            }
        }
    }

    public:
        TrajectoryPlanner(QuadrupedLeg *leg, float swing_height, float stance_depth):
            leg_(leg),
            swing_height_(swing_height),
            stance_depth_(stance_depth),
            total_control_points_(12),
            factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
            ref_control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15},
            ref_control_points_y_{-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5},
            height_ratio_(0),
            length_ratio_(0)
        {
            updateControlPointsHeight(swing_height_);
            updateControlPointsLength(step_length_);
        }

        void generate(Transformation &foot_position, float step_length, float rotation, float swing_phase_signal, float stance_phase_signal)
        {    
            updateControlPointsLength(step_length);
            leg_->gait_phase(1);

            int n = total_control_points_ - 1;
            float x = 0.0;
            float y = 0.0;

            if(stance_phase_signal > swing_phase_signal && step_length > 0)
            {
                x = (step_length / 2) * (1 - (2 * stance_phase_signal));
                y = -stance_depth_ * cosf((3.1416 * x) / step_length);
            }
            else if(stance_phase_signal < swing_phase_signal && step_length > 0)
            {
                leg_->gait_phase(0);

                for(unsigned int i = 0; i < total_control_points_ ; i++)
                {
                    float coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);

                    x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
                    y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
                }
            }
            else
            {
                x = 0;
                y = 0;
            }

            foot_position.X() = foot_position.X() + (x * cosf(rotation));
            foot_position.Y() = foot_position.Y() + (x * sinf(rotation));
            foot_position.Z() = foot_position.Z() + y;

            if((!swing_phase_signal && !stance_phase_signal) && step_length > 0)
            {
                foot_position = prev_foot_position_;
            }

            prev_foot_position_ = foot_position;
        }
};

#endif