#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string>

using namespace std;

namespace robot
{
    class Kinematics
    {   
        public:
            /**
             * \brief Here we are finding the angle required for the joints 
             */
            void inverseKinematicsStoch2_5(double x, double y, double z);

            /**
             * \brief A simple IK
             */ 
            void inverseKinematics(double x, double z, const char branch);

            /**
             * \brief cosineRule
             */            
            float cosineRule(float a, float b, float c);

            /**
             * \brief FK of the bot 
             */ 
            void forwardKinematicsStoch2_5(float theta1, float theta2, float x, float y);

            double f_theta[3];
    };

}
