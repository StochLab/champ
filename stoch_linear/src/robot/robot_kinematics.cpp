#include <stoch_linear/ik/robot_kinematics.h>
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <string>

using namespace std;
namespace robot
{
	//Assuming there is no lateral movement for the robot
	void Kinematics::inverseKinematicsStoch2_5(double x, double y, double z)
	{
		double theta, new_cords[3];
		int br;

		br = 0;
		theta = atan2(y,-z); 
		new_cords[0] = x;
		new_cords[1] = z/cos(theta);
		new_cords[2] = y;

		f_theta[0] = theta; //abd
		//printf("\n\nAbd: %f \n\n", f_theta[0]);
		//printf("x value: %f\n",x);
		//printf("y value: %f\n",y);

		//Inverse2D(new_cords[0],new_cords[1], br);
		inverseKinematics(new_cords[0],new_cords[1],'<');

		return;
	}



	void Kinematics::inverseKinematics(double x, double z, const char branch)
	{

		double r;
		double theta,theta1,theta2;
		double link_1 = 0.15;
		double link_2 = 0.175;

		r = sqrtf(x*x + z*z);
		theta = atan2f(z, x); // Reference with the x axis when we publish to the legs we need to take care of the fact that the motors take angles to -y axis so an offset of pi/2 need to be given to the hip

		theta1 = cosineRule(link_2, r, link_1);
		theta2 = -(theta1 + cosineRule(link_1, link_2, r));

		if(branch == '<')
		{
			theta1 *= -1;
			theta2 *= -1;
		}

		theta1 += theta;
		
		//printf("theta1 value: %f\n",theta1);
		//printf("theta2 value: %f\n",theta2);

		f_theta[1] = theta1; //hip 
		f_theta[2] = theta2; //knee

		//printf("theta1 value: %f\n",theta1);
		//printf("theta2 value: %f\n",theta2);

		return;
	}

	void Kinematics::forwardKinematicsStoch2_5(float theta1, float theta2, float x, float z)
	{
		float z1 = -(0.15*sin(theta1) + 0.17 * (cos(theta2) * sin(theta1) + sin(theta2) * cos(theta1)) );
		float x1 = -0.15 * cos(theta1) - 0.17 * cos(theta2) * cos(theta1) + 0.17 * sin(theta2) * sin(theta1);
		printf("x: %f x1: %f  \n",x,x1);
		printf("y: %f y1: %f  \n",z,z1);

		return;        
	}

	float Kinematics::cosineRule(float a, float b, float c)
	{
		return acosf((c*c + b*b - a*a)/(2*b*c));
	}
}
