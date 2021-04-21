/*
 * Copyright 2021, StochLab
 *
 * Author: Aditya Sagi
 * Date:   Feb, 2021
 */

#ifndef __SERIAL2R_KINEMATICS_H__
#define __SERIAL2R_KINEMATICS_H__

#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>

/* Class for the kinematics of a serial-2R chain.
 *
 * This class implements the kinematics (forward and inverse) of a planar serial-2R chain.
 * It assumes that the chain lies in the x-z plane with x pointing forward and z pointing 
 * upward. The joints rotate about the y-axis.
 */
class Serial2RKinematics {

  private:
    std::vector<double> link_lengths_;
    bool                safety_; // safety check turned on

    /* Give the three sides of a triangle, find the angle opposite to side A
     * i.e., angle between links B and C
     *
     * Note: This function does not perform any checks on the inputs. It assumes
     * that valid inputs are provided.
     */
    double cosineRule(double a, double b, double c)
    {
      return acosf((c*c + b*b - a*a)/(2*b*c));
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
    Serial2RKinematics(std::vector<double> link_lengths) :
      link_lengths_(link_lengths), safety_(true)
  {}

    Serial2RKinematics(): link_lengths_({1.0, 1.0}), safety_(true) {}

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


    /* Invserse kinematics for a serial-2R chain.
     *
     * This function implements the inverse kinematics for a planar serial-2R chain. 
     * It is assumed that the chain lies in the x-z plane with the first joint angle
     * measured w.r.t. the negative z axis (vertical axis) and the second joint
     * angle measured w.r.t the first link. This function can provide one of two
     * solutions to the inverse kinematics problem based on the branch that is chosen.
     *
     * The inverse kinematics is solved in two stages. First, the angle made by the radial
     * line (line joining origin to the end-effector) w.r.t the reference is determined. Second,
     * the angle made by the link l1 w.r.t. the radial line is determined. The hip angle then 
     * becomes the sum of the two angles. 
     *
     * \param[in] ee_pos : End-effector position in the Cartesian space (x, z).
     *
     * \param[in] branch : Specifies branch of the inverse kinematics solution.
     *                     It can take values '<' or '>'.
     *
     * \param[out] joint_state : The joint angle obtained as a solution are provided 
     *                           in this vector. 
     *
     *
     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists for given input.
     *                Returns 1 if a new point is found within the workspace and the solution exists for it.
     *                Returns -1 if no solution exists.
     */     
    int inverseKinematics(std::vector<double> ee_pos, char branch, std::vector<double>& joint_angles)
    {
      assert(ee_pos.size() == 2);
      joint_angles.resize(2);

      double  l1, l2;
      double  theta1, theta2;
      double  t1, t2;
      double  r_sqr, r;
      double  x, z;
      double  r_theta; 
      bool    valid_pt=false;
      int     valid_ik=0;
      std::vector<double> pos(2);

      // Zero position is invalid
      if( (fabs(ee_pos[0]) < 0.0001 ) && (fabs(ee_pos[1]) < 0.0001))
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

      x = pos[0]; 
      z = pos[1];

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      // Square of the radial distance from the origin.
      r_sqr = x*x + z*z;
      r = sqrt(r_sqr);

      r_theta = atan2(-x, -z); // angle made by radial line w.r.t the reference

      // Ensure the output lies in the range [-PI , PI]
      if(r_theta > M_PI)
        r_theta = r_theta - 2*M_PI;
      else if (r_theta < -M_PI)
        r_theta = r_theta + 2*M_PI;

      t1 = cosineRule(l2, r, l1);  // internal angle opposite to l2
      t2 = cosineRule(r, l1, l2);  // internal angle opposite to radial line

      theta2 = -(M_PI - t2);

      if(branch == '<') {
        t1 = -t1;
        theta2 = -theta2;
      }

      theta1 = r_theta + t1;

      joint_angles.resize(2);
      joint_angles[0] = theta1;
      joint_angles[1] = theta2;

      return valid_ik;
    }

    /* Forward kinematics of a serial-2R chain.
     *
     * This function provides the forward kinematics soultion for a planar serial-2R chain.
     * It is assumed that the chain lies in the x-z plane. The first joint angle is measured
     * w.r.t the negative z-axis and the second joint angle is measured w.r.t the first link.
     * The axis of rotation is the positive y-axis and the direction is determined using 
     * the right-hand rule with the thumb pointing in the positive y direction. 
     *
     * \param[in] joint_angles : A two element vector of the joint angles.
     *
     * \param[out] ee_pos : End-effector position (x, z).
     */
    void forwardKinematics(std::vector<double> joint_angles, std::vector<double>& ee_pos)
    {
      assert(joint_angles.size() == 2);
      ee_pos.resize(2);

      double theta1, theta2;
      double l1, l2;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      theta1 = joint_angles[0];
      theta2 = joint_angles[1]; 

      ee_pos[0] = -(l1*sin(theta1) + l2*sin(theta1+theta2));
      ee_pos[1] = -(l1*cos(theta1) + l2*cos(theta1+theta2));

      return;
    }



    /* Function to check if the provided point is within the workspace of the serial-2R.
     *
     * \param[in] ee_pos: end-effector position
     *
     * \return bool: true if the point lies in the workspace, false otherwise     
     */
    bool inWorkspace(std::vector<double> ee_pos)
    {
      double y, z;
      double l1, l2;
      double r_sqr;

      y = ee_pos[0];
      z = ee_pos[1];

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      r_sqr = y*y + z*z;

      if ( r_sqr > (l1 + l2)*(l1+l2))
        return false;

      if ( r_sqr < (l1 - l2)*(l1 - l2))
        return false;

      return true;
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

      std::vector<double> p_in(2), p_out(2), p(2);
      std::vector<double> unit_vector(2);
      double r; // radial distance;
      int n=0;

      // If the input is valid, then there is no need to search
      if(inWorkspace(desired_pos))
      {
        return desired_pos;
      }

      // p_out is always an invalid point (lies outside the workspace)
      p_out= desired_pos;

      r = vectorLength(p_out, {0,0});
      unit_vector[0] = p_out[0]/r;
      unit_vector[1] = p_out[1]/r;

      // p_in is always a valid point (lies inside the workspace)
      p_in[0] = RADIAL_DISTANCE*unit_vector[0];
      p_in[1] = RADIAL_DISTANCE*unit_vector[1];

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

#endif //__SERIAL2R_KINEMATICS_H__
