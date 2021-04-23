#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vector>

#define FL 0
#define FR 1
#define BL 2
#define BR 3

using namespace std;

namespace controller
{
    class TrotGaitController
    {   
        public:
             /**
             * \brief Setting the Gait Configuration
             */
            void setGaitConfig( double Walking_h, double Swing_h, double Leg_length, double No_of_points, double phase[], double sl);

            /**
             * \brief Get the current Gait Configuration position of the joint
             */
            void getGaitConfig(double &Walking_h, double &Swing_h, double &Leg_length, double &No_of_points, double *phase);

            /**
             * \brief restricting the theta within a value
             */
            double constrainTheta(double theta);

            /**
             * \brief We are updating the theta value of the each
             */
            void updateLegTheta(double theta);

            /**
             * \brief We are updating the Phi value of the leg
             */
            void updateLegPhiVal(double *leg_phi);

            /**
             * \brief We are updating the step length for each leg 
             */    
            void updateLegStepLengthVal(double *leg_sl);

            /**
             * \brief Here we are shifting the elipse that needs to be traversed by the leg 
             */
            void initializeElipseShift(double *yshift, double *xshift,double *zshift);

            /**
             * \brief We initialise the legs current state 
             */
            void initializeLegState(double theta, std::vector < double > action);
            
            /**
             * \brief This is the main function that calls the other functions 
             */
            void runEllipticalTrajStoch2_5( std::vector < double > action, double theta, int value, double final_bot_joint_angles[4][3]);
            
            /**
             * \brief This is the main function that runs the elliptical trajectory for a single leg 
             */            
            void runEllipticalTrajLeg( std::vector< double > action, double theta, int value, double *final_leg_joint_angles);

            double wh_, sh_, leg_length_, step_length_, theta_, no_of_points_;   

            struct leg_data
            {
                std::string name;
                float motor_hip; 
                float motor_knee;
                float motor_abduction;
                float x;
                float y;
                float z;
                float theta;
                float phi;
                float b;
                float step_length;
                float x_shift;
                float y_shift;
                float z_shift;
            }*leg;

            struct robot_data
            {
                struct  leg_data front_right;
                struct leg_data front_left;
                struct leg_data back_right;
                struct leg_data back_left;

            }stoch2_5_;

            struct leg_phase
            {
                float f_r_phase;
                float f_l_phase;
                float b_r_phase;
                float b_l_phase;
            }leg_phase_;

    };

}
