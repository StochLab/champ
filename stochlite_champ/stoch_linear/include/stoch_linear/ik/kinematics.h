/*
 * kinematics.h
 *
 * Created : 15 Feb, 2021
 * Author  : Aditya Sagi
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "serial3r_kinematics.h"

#define FL  0
#define FR  1
#define BL  2
#define BR  3

#define X 0
#define Y 1
#define Z 2


#define NLEGS 4 // Number of legs
#define NM_PER_LEG 3 // Number of motors per leg

namespace stochlite
{
  using namespace RTT;

  class Kinematics : public TaskContext 
  {
  protected:
    const double BODY_LENGTH = 0.38;  // hip-to-hip length of the body (in metres)
    const double BODY_WIDTH  = 0.24;  // hip-to-hip width of the body (in metres) 

    const double ABD_LEN    = 0.096; // length of abduction link (metres)
    const double THIGH_LEN  = 0.146; // length of thigh link (metres)
    const double SHANK_LEN  = 0.172 ; // length of shank link (metres)

    char branch_fl, branch_fr, branch_bl, branch_br;

    Serial3RKinematics *serial_3r;
    
    bool inverseKinematics(geometry_msgs::PoseArray msg, sensor_msgs::JointState& joint_state );
    bool inverseKinematicsLeg(std::string frame, std::vector<double> msg, char branch, sensor_msgs::JointState& joint_state );

    bool forwardKinematics(sensor_msgs::JointState joint_statei, geometry_msgs::PoseArray& msg);
    bool forwardKinematicsLeg(std::string frame, sensor_msgs::JointState joint_state, geometry_msgs::Pose& msg);
    int  findString(std::vector<std::string> str_array, std::string str_element);

  public:
    Kinematics(const std::string& name) :
      TaskContext(name)
    {
      // trigger the update hook when data is received on the input port (addEventPort)
      this->ports()->addEventPort("foot_pos_in", foot_pos_in_).doc("Input of the foot position to the kinematics module.");
      this->ports()->addEventPort("joint_pos_in", joint_pos_in_).doc("Input of the joint position to the kinematics module.");

      this->ports()->addPort("joint_pos_out", joint_pos_out_).doc("Ouput of joint positions.");
      this->ports()->addPort("foot_pos_out", foot_pos_out_).doc("Ouput of foot positions.");

      branch_fl = '>';
      branch_fr = '>';
      branch_bl = '>';
      branch_br = '>';

      serial_3r = new Serial3RKinematics({ABD_LEN, THIGH_LEN, SHANK_LEN});

    }

    void updateHook() 
    {
      log(Info) << "Kinematics:: Update hook." << endlog();

      geometry_msgs::PoseArray	foot_pos;
      sensor_msgs::JointState	  joint_state;
      bool                      valid_ik=false, valid_fk=false;

      if(foot_pos_in_.read(foot_pos) == NewData)
      {
        if (foot_pos.header.frame_id == "base_link")
          valid_ik = inverseKinematics(foot_pos, joint_state);
        else if (foot_pos.header.frame_id == "fl")
          valid_ik = inverseKinematicsLeg(std::string("fl"), foot_pos.poses[0], branch_fl, joint_state);
        else if (foot_pos.header.frame_id == "fr")
          valid_ik = inverseKinematicsLeg(std::string("fr"), foot_pos.poses[0], branch_fr, joint_state);
        else if (foot_pos.header.frame_id == "bl")
          valid_ik = inverseKinematicsLeg(std::string("bl"), foot_pos.poses[0], branch_bl, joint_state);
        else if (foot_pos.header.frame_id == "br")
          valid_ik = inverseKinematicsLeg(std::string("br"), foot_pos.poses[0], branch_br, joint_state);

        if(valid_ik)
          joint_pos_out_.write(joint_state);	
      }
      else if (joint_pos_in_.read(joint_state) == NewData)
      {
          valid_fk = forwardKinematics(joint_state, foot_pos);

          if(valid_fk)
            foot_pos_out_.write(foot_pos);

      }
    }

  };

}

#endif /* _KINEMATICS_H_ */
