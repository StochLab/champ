#include "trajectory_core.cpp"
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;






trajectory_msgs::JointTrajectory traj;

controller::Trot trot;




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
        traj.points.at(0).positions.at(i) = trot.set_pos.at(i);
    }
    //traj.header.seq++;
    //traj.header.stamp = ros::Time::now();
    //traj.header.stamp = ros::Time::now() - t_init;
}



int main(int argc, char **argv){

    ros::init(argc, argv,"Elliptical_trajectory_node");
    ros::NodeHandle n;
    ros::Time t_init = ros::Time::now();
    initTraj();
    ros::Publisher jointControl_pub = n.advertise<trajectory_msgs::JointTrajectory>("/joint_elliptical_trajectory", 1000); 


    ros::Rate loop_rate(200);
    while(ros::ok()){
        if(trot.new_act){
            
            trot.stepRun();
            updateTraj(ros::Time::now());
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
      
    return 0;
}
