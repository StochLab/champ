#include <stoch_linear/controller/trot/trot_gait_controller.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "stoch_linear/Action_step.h"
using namespace std;


            std::vector<double> id;
            std::vector<double> set_pos;
            std::vector<double> Ref;
            std::vector<double> twist_input;
            std::vector<double> twist_input_local;
            std::vector<double> action;
            std::vector<double> state;
            std::vector<double> imu_readings;
            std::vector<double> slope_readings;
            std::vector<double> imu_readings_to_slope;
            std::vector<double> motor_command;
            std::vector<double> slope_reading_local;
            std::vector<double> imu_readings_local;
            std::vector<double> past_three_imu_readings;

            double theta=1.0;
            double dt = 0.007;
            double freq = 2*M_PI/2.51;
            double phase[4];
            double wh = -0.25;
            double sh = 0.06;
            double leg_length = 0.15;
            double step_length_ = 0.1;
            double count;
            double omega;
            double step_length_1;
            double body_length;
            double body_width;
            double button;
            double button_local;
            double zero_offset[12];

            int no_of_points = 250;

bool new_act= true;

trajectory_msgs::JointTrajectory traj;

controller::TrotGaitController trot;
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

void initiate_action(){
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

void initTraj(){
    //traj.header.stamp = ros::Time::now();
    

    traj.joint_names.push_back("fl_abd_joint");
    traj.joint_names.push_back("fl_hip_joint");
    traj.joint_names.push_back("fl_knee_joint");
    traj.joint_names.push_back("fr_abd_joint");
    traj.joint_names.push_back("fr_hip_joint");
    traj.joint_names.push_back("fr_knee_joint");
    traj.joint_names.push_back("bl_abd_joint");
    traj.joint_names.push_back("bl_hip_joint");
    traj.joint_names.push_back("bl_knee_joint");
    traj.joint_names.push_back("br_abd_joint");
    traj.joint_names.push_back("br_hip_joint");
    traj.joint_names.push_back("br_knee_joint");

    vector<double> pos;
    pos.push_back(-0.2107);
    pos.push_back(0.90580);
    pos.push_back( -1.653);
    pos.push_back( 0.2107);
    pos.push_back( 0.9058);
    pos.push_back( -1.653);
    pos.push_back( -0.2107);
    pos.push_back( -0.9058);
    pos.push_back( 1.65333);
    pos.push_back( 0.2107);
    pos.push_back( -0.9058);
    pos.push_back( 1.653);
    trajectory_msgs::JointTrajectoryPoint temp_point;
    temp_point.positions = pos;
    traj.points.push_back(temp_point);

    
}

void updateTraj(ros::Time t_init){
    for(int i = 0; i<12;i++){
        traj.points.at(0).positions.at(i) = set_pos.at(i);
    }
    //traj.header.seq++;
    //traj.header.stamp = ros::Time::now();
    // /traj.header.stamp = ros::Time::now() - t_init;
}

bool init_id()
{
        id.push_back(11);
        id.push_back(12);
        id.push_back(13);
        id.push_back(21);
        id.push_back(22);
        id.push_back(23);
        id.push_back(31);
        id.push_back(32);
        id.push_back(33);
        id.push_back(41);
        id.push_back(42);
        id.push_back(43);

        return true;

}


std::vector<double> doSimulation( double theta, std::vector<double> action)
      {
        double final_bot_joint_angles[4][3];
        std::vector<double> joint_angles;

        //trot.setGaitConfig(this->wh,this->sh,this->leg_length,this->no_of_points,this->phase,this->step_length_);

        trot.runEllipticalTrajStoch2_5(action, theta, 0, final_bot_joint_angles);

        for(int j = 0; j < 4; j++)
        {

          for( int i =0 ; i<3 ; i++)
          {
            joint_angles.push_back(final_bot_joint_angles[j][i]);
          }

        }
        //printf("The size :%ld\n",list.size());

        return joint_angles;
      }

void setGaitCon(double walking_h,double swing_h,double step_len)
{

        trot.setGaitConfig(walking_h, swing_h, leg_length, no_of_points, phase, step_len);
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


void get_actions(const stoch_linear::Action_step &msg){
    for(int i = 0; i < 20; i++){
        action.at(i) = msg.actions.at(i);
    }
    new_act = true;
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

int main(int argc, char **argv){

    ros::init(argc, argv,"Elliptical_trajectory_node");
    initiate_action();
    init_id();
   
    init_kinematics();

    ros::NodeHandle n;
    ros::Time t_init = ros::Time::now();
    initTraj();
    ros::Publisher jointControl_pub = n.advertise<trajectory_msgs::JointTrajectory>("/joint_elliptical_trajectory", 1000);
    ros::Subscriber sub = n.subscribe("/action_ml",1000, get_actions);
       


    ros::Rate loop_rate(200);
    while(ros::ok()){
        if(true){
            new_act = false;
            set_pos.clear();
            set_pos = doSimulation(theta,action);
            omega = 2 * no_of_points * freq;
            theta = trot.constrainTheta( (omega * dt) + theta);

            updateTraj(t_init);
            // test trajectory
            jointControl_pub.publish(traj);
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
      
    return 0;
}