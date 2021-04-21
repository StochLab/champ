/*
 * Copyright 2021, StochLab
 *
 * Author: Aditya Sagi
 * Date:   Feb, 2021
 */

#ifndef __SERIAL3R_KINEMATICS_H__
#define __SERIAL3R_KINEMATICS_H__

#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>

#include "serial2r_kinematics.h"

/* Class for the kinematics of a serial-3R chain.
 *
 * This class implements the kinematics (forward and inverse) of a spatial serial-3R chain.
 */
class Serial3RKinematics {

  private:
    std::vector<double> link_lengths_;
    bool                safety_; // safety check turned on

    bool                right_leg_; // leg on the right side

    Serial2RKinematics* serial_2r;

    /* 3D rotation about the X-axis.
     *
     * This function performs a rotation on the input vector about the 
     * x-axis by a specified angle.
     *
     * \param[in] angle : Angle of rotation about the x-axis.
     *
     * \param[in] vec_in : 3D input vector.
     *
     * \return : Returns the 3D vector which is rotated about the x-axis.
     */
    std::vector<double> rotX(double angle, std::vector<double> vec_in)
    {
      assert(vec_in.size() == 3);

      std::vector<double> vec_out;
      vec_out.resize(3);

      vec_out[0] = vec_in[0];
      vec_out[1] = cos(angle) * vec_in[1] - sin(angle) * vec_in[2];
      vec_out[2] = sin(angle) * vec_in[1] + cos(angle) * vec_in[2];

      return vec_out;
    }

    /* Function to determine the length of a vector using the Euclidean norm.
     *
     * The vector is defined by the position vectors of its end points.
     *
     * \param[in] p1 : vector 1
     *
     * \param[in] p2: vector 2
     *
     * \return double : Returns the length of the vector.
     */
    double vectorLength(std::vector<double> p1, std::vector<double> p2)
    {
      assert(p1.size() == p2.size());

      double  r, r_sqr;
      int     size;

      size = p1.size();
      for(int i = 0; i < size; i++)
      {
        r_sqr += pow((p2[i] -p1[i]), 2);
      }
      r = sqrt(r_sqr);

      return r;
    }

    /* Mid-point of a vector.
     *
     * Mid-point of the vector defined by its end points.
     *
     * \param[in] p1 : vector 1
     *
     * \param[in] p2 : vector 2
     *
     * \return vector: The position vector of the mid-point between p1 and p2.
     */
    std::vector<double> midPoint(std::vector<double> p1, std::vector<double> p2)
    {
      assert(p1.size() == p2.size());
      std::vector<double> p;

      p.resize(p1.size());

      for(int i=0; i<p1.size(); i++)
      {
        p[i] = 0.5*(p1[i] + p2[i]);
      }

      return p;
    }

  public:
    Serial3RKinematics(std::vector<double> link_lengths) :
      link_lengths_(link_lengths), safety_(true)
  {
    serial_2r = new Serial2RKinematics({link_lengths[1], link_lengths[2]});
  }

    Serial3RKinematics(): link_lengths_({1.0, 1.0, 1.0}), safety_(true)
    {
      serial_2r = new Serial2RKinematics({link_lengths_[1], link_lengths_[2]});
      right_leg_= false;
    }

    ~Serial3RKinematics()
    {
      if(serial_2r)
        delete serial_2r;
    }

    void turnOnSafety()
    { 
      safety_=true;
      return;
    }

    void turnOffSafety()
    {
      safety_ = false;
      return;
    }

    bool getSafety()
    {
      return safety_;
    }


    /* Inverse kinematics for a serial-3R chain.
     *
     * This function implements the inverse kinematics for a planar serial-3R chain. 
     * It is assumed that the chain lies in the x-z plane with the first joint angle
     * measured w.r.t. the negative z axis (vertical axis) and the second joint
     * angle measured w.r.t the first link. This function can provide one of two
     * solutions to the inverse kinematics problem based on the branch that is chosen.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br"
     *
     * \param[in] ee_pos : End-effector position in the Cartesian space (x, y, z).
     *
     * \param[in] branch : Specifies branch of the inverse kinematics solution.
     *                     It can take values '<' or '>' for the branch of the 
     *                     serial-2R chain formed by the last two links.
     *
     * \param[out] joint_state : The joint angle obtained as a solution are provided 
     *                           in this vector. 
     *
     *
     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists.
     *                Returns 1 if the input is modified for safety (and solution exists)
     *                Returns -1 if solution does not exis.
     */     
    int inverseKinematics(std::string leg_name, std::vector<double> ee_pos, char branch, std::vector<double>& joint_angles)
    {
      assert(ee_pos.size() == 3);
      joint_angles.resize(3);

      double  t1, t2;
      double  abd_angle, hip_angle, knee_angle;
      double  h, r;
      double  x, y, z;
      int     valid_ik=0;
      double  l1;

      std::vector<double> foot_pos_2r(2);
      std::vector<double> joint_angles_2r(2);
      std::vector<double> pos(3);

      // Zero position is invalid
      if( (fabs(ee_pos[0]) < 0.0001 ) && (fabs(ee_pos[1]) < 0.0001) && (fabs(ee_pos[0]) < 0.0001 ) )
        return -1;

      // If not in workspace, then find the point in workspace
      // closest to the desired end-effector position. Return
      // false if such a point is not found.
      if (!inWorkspace(ee_pos))
      {
        if(safety_)
        {
          pos = searchSafePosition(ee_pos);
          valid_ik = 1;
        }
        else
        {
          return -1;
        }
      }
      else
      {
        pos = ee_pos;
      }

      if((leg_name == "fr") || (leg_name == "br"))
        right_leg_ = true;
      else
        right_leg_ = false;

      x = pos[0];
      y = pos[1];
      z = pos[2];

      l1 = link_lengths_[0];

      r = sqrt(y*y + z*z);

      h = sqrt(r*r - l1*l1);

      t1 = atan2(h, l1);
      t2 = atan2(y, -z);

      if(right_leg_)
        abd_angle = M_PI/2 - t1 + t2;
      else
        abd_angle = t1 + t2 - M_PI/2;
        
      foot_pos_2r[0] = x;
      foot_pos_2r[1] = -h;

      serial_2r->inverseKinematics(foot_pos_2r, branch, joint_angles_2r);

      hip_angle = joint_angles_2r[0];
      knee_angle = joint_angles_2r[1];

      joint_angles[0] = abd_angle;
      joint_angles[1] = hip_angle;
      joint_angles[2] = knee_angle;

      return valid_ik;
    }

    /* Forward kinematics of a serial-3R chain.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br".
     * 
     * \param[in] joint_angles : A three element vector of the joint angles.
     *
     * \param[out] ee_pos : End-effector position (x, y, z).
     *
     */
    void forwardKinematics(std::string leg_name, std::vector<double> joint_angles, std::vector<double>& ee_pos)
    {
      assert(joint_angles.size() == 3);
      ee_pos.resize(3);

      double abd_angle, hip_angle, knee_angle;
      double l1, l2, l3;
      std::vector<double> foot_pos_2r, foot_pos_3r;
      std::vector<double> joint_angles_2r;

      if((leg_name == "fr") || (leg_name == "br"))
        right_leg_ = true;
      else
        right_leg_ = false;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];
      l3 = link_lengths_[2];

      abd_angle  = joint_angles[0];
      hip_angle  = joint_angles[1]; 
      knee_angle = joint_angles[2]; 

      joint_angles_2r.resize(2);
      joint_angles_2r[0] = hip_angle;
      joint_angles_2r[1] = knee_angle;

      serial_2r->forwardKinematics(joint_angles_2r, foot_pos_2r);

      // Foot position without considering abduction movement
      foot_pos_3r.resize(3);
      foot_pos_3r[0] = foot_pos_2r[0];          // x
      foot_pos_3r[1] = right_leg_ ? -l1 : l1;   // y
      foot_pos_3r[2] = foot_pos_2r[1];          // z

      // Rotate about x-axis to account for the abduction angle.
      ee_pos = rotX(abd_angle, foot_pos_3r); 

      return;
    }

    /* Function to check if the specified point lies in the workspace.
     *
     * \param[in] ee_pos: End-effector position (vector of dimension 3)
     *
     * \return bool: true if the point lies in the workspace, false otherwise.
     */
    bool inWorkspace(std::vector<double> ee_pos)
    {
      double              x, y, z;
      double              l1;
      std::vector<double> foot_pos_2r(2);
      bool                in_workspace;
      double              h;
      double              r;


      x = ee_pos[0];
      y = ee_pos[1];
      z = ee_pos[2];

      l1 = link_lengths_[0];

      r = sqrt(y*y + z*z);

      // Out of workspace
      if( r < l1)
        return false;

      h = sqrt(r*r - l1*l1);

      foot_pos_2r[0] = x;
      foot_pos_2r[1] = -h;

      in_workspace = serial_2r->inWorkspace(foot_pos_2r);

      return in_workspace;
    }

    /* Function to search for the closest end-effector point within the workspace.
     *
     * This uses the bisection method to search for a feasible point on the boundary
     * of the workspace.
     *
     * \param[in] desired_pos: desired position of the end-effector
     *
     * \return array: Valid position inside the workspace
     */                        
    std::vector<double> searchSafePosition(std::vector<double> desired_pos)
    {
      const double  RADIAL_DISTANCE=0.2; // radial distance for a point in workspace
      const double  DELTA_MAX = 0.001; 
      const int     MAX_ITERATIONS = 20;

      std::vector<double> p_in(3), p_out(3), p(3);
      std::vector<double> unit_vector(3);
      double r; // radial distance;
      int n=0;

      // If the input is valid, then there is no need to search
      if(inWorkspace(desired_pos))
      {
        return desired_pos;
      }

      // p_out is always an invalid point (lies outside the workspace)
      p_out= desired_pos;

      r = vectorLength(p_out, {0,0,0});
      unit_vector[0] = p_out[0]/r;
      unit_vector[1] = p_out[1]/r;
      unit_vector[2] = p_out[2]/r;

      // p_in is always a valid point (lies inside the workspace)
      p_in[0] = RADIAL_DISTANCE*unit_vector[0];
      p_in[1] = RADIAL_DISTANCE*unit_vector[1];
      p_in[2] = RADIAL_DISTANCE*unit_vector[2];

      r = vectorLength(p_in, p_out);

      while( (r > DELTA_MAX) && (n < MAX_ITERATIONS))
      {
        p = midPoint(p_in, p_out);
        if(inWorkspace(p))
          p_in = p;
        else
          p_out = p;
        r = vectorLength(p_in, p_out);
        n++;
      }

      return p_in;
    }

};

#endif //__SERIAL3R_KINEMATICS_H__

