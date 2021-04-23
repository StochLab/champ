#include <stoch_linear/controller/trot/trot_gait_controller.h>
#include <stoch_linear/ik/robot_kinematics.h>

namespace controller
{
    class Trot{
        protected:
            std::vector<double> id;
            std::vector<double> Ref;
            std::vector<double> twist_input;
            std::vector<double> twist_input_local;
            std::vector<double> state;
            std::vector<double> imu_readings;
            std::vector<double> slope_readings;
            std::vector<double> imu_readings_to_slope;
            std::vector<double> motor_command;
            std::vector<double> slope_reading_local;
            std::vector<double> imu_readings_local;
            std::vector<double> past_three_imu_readings;

            double theta;
            double dt;
            double freq;
            double phase[4];
            double wh;
            double sh;
            double leg_length;
            double step_length_;
            double count;
            double omega;
            double step_length_1;
            double body_length;
            double body_width;
            double button;
            double button_local;
            double zero_offset[12];


            int no_of_points;


            TrotGaitController trot;

            void setGaitCon(double walking_h,double swing_h,double step_len)
            {

                trot.setGaitConfig(walking_h, swing_h, this->leg_length, this->no_of_points, phase, step_len);
                return ;

            }

            double constrainAngle(double theta0)
            {

                double theta1;
                double omega;

                omega = 2 * no_of_points * freq;
                theta1 = trot.constrainTheta( (omega * dt) + theta0);

                return theta1;

            }


            void initiate_action(){
                // put it in the linear policy

                    //step length
                action.push_back(0.3);
                action.push_back( 0.3);
                action.push_back( 0.3);
                action.push_back(0.3);

                    //drive angle
                action.push_back(0);
                action.push_back(0);
                action.push_back(0);
                action.push_back(0);

                    //x-shift
                action.push_back(0.05);
                action.push_back(0.05);
                action.push_back(0.05);     
                action.push_back(0.05);

                    //y-shift
                action.push_back(0.2);
                action.push_back(0.2);
                action.push_back(0.2);
                action.push_back(0.2);

                    //z-shift
                action.push_back(0);
                action.push_back(0);
                action.push_back(0);
                action.push_back(0);
            }
            
            bool init_kinematics()
            {

                //forwardKinematics2_5 = peer2 -> getOperation("forwardKinematics2_5");
       
                action.resize(20);
                phase[0] = no_of_points;
                phase[1] = 0;
                phase[2] = 0;
                phase[3] = no_of_points;

                trot.setGaitConfig(wh, sh, leg_length, no_of_points, phase, step_length_);

                //dt = this -> getPeriod();
                return true;

            }


        public:


            std::vector<double> set_pos;
            std::vector<double> action;
            bool new_act;

            Trot()
                : theta(1.0),freq(2*M_PI/2.51), no_of_points(250),wh(-0.25),sh(0.06),leg_length(0.15),step_length_(0.1),dt(0.007)
            {
                new_act = false;
                initiate_action();
            }  

            std::vector<double> sampleJoints(double theta, std::vector<double> action){
                // Previously doSimulation
                double final_bot_joint_angles[4][3];
                std::vector<double> joint_angles;

                trot.runEllipticalTrajStoch2_5(action, theta, 0, final_bot_joint_angles);

                for(int j = 0; j < 4; j++)
                {

                    for( int i =0 ; i<3 ; i++)
                    {
                        joint_angles.push_back(final_bot_joint_angles[j][i]);
                    }

                }
                return joint_angles;
            }


            void stepRun(){
                if(new_act){
                    new_act = false;
                    set_pos.clear();
                    set_pos = sampleJoints(theta, action);
                    omega = 2 * no_of_points * freq;
                    theta = trot.constrainTheta( (omega * dt) + theta);
                }
            }

    };

}