/*
 * kinematics.cpp
 *
 * Created : 15 Feb, 2021
 * Author  : Aditya Sagi
 */

#include <stoch_linear/ik/kinematics.h>

namespace stochlite
{
  /* Inverse kinematics for the leg of StochLite.
   *
   * Args:
   * \param[in] leg_name: Name of the leg. Possible values = "fl", "fr", "bl", "br"
   *
   * \param[in] branch: Branch of the solution for the inverse kinematics. Can be ">" or "<".
   *
   * \param[in] foot_pos: Position vector of the foot (x, y, z) in metres.
   *
   * \param[out] joint_state: Joint angles corresponding to the foot position (abd, hip, knee) in radians.
   *
   *
   * \return bool: true if IK solution is valid, false otherwise.
   */
  bool Kinematics::inverseKinematicsLeg(std::string leg_name, std::vector<double> foot_pos, char branch, std::vector<double>& joint_state)
  {
    int                 valid_ik;
    std::vector<double> foot_position;
    std::vector<double> joint_angles;

    foot_position.resize(3);
    joint_angles.resize(NM_PER_LEG);

    foot_position[0] = foot_pos.at(X);
    foot_position[1] = foot_pos.at(Y);
    foot_position[2] = foot_pos.at(Z);

    if( (leg_name == "fl") || (leg_name == "fr") || \
        (leg_name == "bl") || (leg_name == "br") )
    {
      valid_ik = serial_3r->inverseKinematics(leg_name, foot_position, branch, joint_angles); 
    }
    else 
    {
      // Invalid frame ID
      return false;
    }

    if(valid_ik < 0)
    {
      return false;
    }

    joint_state.resize(NM_PER_LEG);

    joint_state = joint_angles;

    return true;
  }


  /* Forward kinematics for the legs of StochLite.
   *
   *
   * Args:
   * \param[in] frame: ID of the frame in which the position is required.
   *                   Possible values are "fl", "fr", "bl", "br".
   *
   * \param[in] joint_state: Joint angles for the abduction hip and knee
   *
   * \param[out] foot_pos: Position of the foot in the respective leg frame of reference.
   *
   *
   * \return bool: False if there is invalid input, true otherwise.
   * 
   */
  bool Kinematics::forwardKinematicsLeg(std::string frame, std::vector<double> joint_state, std::vector<double>& foot_pos)
  {
    std::vector<double> joint_state_3r;
    std::vector<double> foot_pos_3r;
    double              abd, hip, knee;
    int                 index;


      serial_3r->forwardKinematics(frame, joint_state_3r, foot_pos_3r);

      foot_pos[X] = foot_pos_3r[X];
      foot_pos[Y] = foot_pos_3r[Y];
      foot_pos[Z] = foot_pos_3r[Z];

    return true;
  }
