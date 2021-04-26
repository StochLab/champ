#include <stdio.h>
#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "stoch_linear/utils/vector_op.h"
#include "stoch_linear/utils/matrix_op.h"
#include "stoch_linear/utils/rotation.h"
#include "stoch_linear/utils/tof_coordinates.h"
#include "stoch_linear/utils/slope_estimator.h"
#include <vector>
#include <bits/stdc++.h>

using namespace std;
using namespace mat;
Matrix3f transpose(Vector3f x,Vector3f y, Vector3f z)
{
	Matrix3f r;
	r[0][0]=x[0];
	r[1][0]=x[1];
	r[2][0]=x[2];
	r[0][1]=y[0];
	r[1][1]=y[1];
	r[2][1]=y[2];
	r[0][2]=z[0];
	r[1][2]=z[1];
	r[2][2]=z[2];

	return r;
}

Matrix3f matrix(Vector3f x,Vector3f y, Vector3f z)
{
	Matrix3f r;
	r[0][0]=x[0];
	r[0][1]=x[1];
	r[0][2]=x[2];
	r[1][0]=y[0];
	r[1][1]=y[1];
	r[1][2]=y[2];
	r[2][0]=z[0];
	r[2][1]=z[1];
	r[2][2]=z[2];

	return r;
}

double giveMedian(double a[4], int n)
{
	// First we sort the array
	sort(a, a + n);

	// check for even case
	if (n % 2 != 0)
		return (double)a[n / 2];

	return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;

}

Vector3f slope_estimator(std::vector<double> z, std::vector<double> imu_r_p) {

	double	    n1=0;
	double			n2=0;
	double			n3=0;
	double			n4=0;
	double	    n5=0;
	double    	n6=0;

	Vector3f tof1, tof2, tof3, tof4, tof5, tof6;
	Vector3f plane_normal1, plane_normal2, plane_normal3, plane_normal4, plane_normal5, plane_normal6;
	Vector3f diag52, diag62, diag42, diag53, diag51, diag31;
	Vector3f rpy1, rpy2, rpy3, rpy4, rpy5, rpy6, rpy_angles;
	double r,p;
	// X-axis pointing forward
	// Y-axis pointing to the left
	// Z-axis pointing upward

	tof1 = {tof_coor[0][0], tof_coor[0][1], z.at(1)};
	tof2 = {tof_coor[1][0], tof_coor[1][1], z.at(3)};
	tof3 = {tof_coor[2][0], tof_coor[2][1], z.at(5)};
	tof4 = {tof_coor[3][0], tof_coor[3][1], z.at(7)};
	tof5 = {tof_coor[4][0], tof_coor[4][1], z.at(9)};
	tof6 = {tof_coor[5][0], tof_coor[5][1], z.at(11)};

	// Vectors along the diagonals
	diag52 = tof5 - tof2;
	diag62 = tof6 - tof2;
	diag42 = tof4 - tof2;
	diag53 = tof5 - tof3;
	diag51 = tof5 - tof1;
	diag31 = tof3 - tof1;
	// Normal to the plane
	plane_normal1 = cross(diag52, diag62);
	plane_normal2 = cross(diag42, diag62);
	plane_normal3 = cross(diag52, diag53);
	plane_normal4 = cross(diag51, diag52);
	plane_normal5 = cross(diag31, diag42);
	plane_normal6 = cross(diag42, diag62);


	n1 = norm(plane_normal1);
	n2 = norm(plane_normal2);
	n3 = norm(plane_normal3);
	n4 = norm(plane_normal4);
	n5 = norm(plane_normal5);
	n6 = norm(plane_normal6);

	rpy1 = normalToEuler(plane_normal1, n1, imu_r_p);
	rpy2 = normalToEuler(plane_normal2, n2, imu_r_p);
	rpy3 = normalToEuler(plane_normal3, n3, imu_r_p);
	rpy4 = normalToEuler(plane_normal4, n4, imu_r_p);
	rpy5 = normalToEuler(plane_normal5, n5, imu_r_p);
	rpy6 = normalToEuler(plane_normal6, n6, imu_r_p);
	//printf("R 1:%f, 2:%f, 3:%f, 4:%f, 5:%f, 6:%f\n",rpy1[0]*180/M_PI,rpy2[0]*180/M_PI,rpy3[0]*180/M_PI,rpy4[0]*180/M_PI,rpy5[0]*180/M_PI,rpy6[0]*180/M_PI);
	//printf("P 1:%f, 2:%f, 3:%f, 4:%f, 5:%f, 6:%f\n",rpy1[1]*180/M_PI,rpy2[1]*180/M_PI,rpy3[1]*180/M_PI,rpy4[1]*180/M_PI,rpy5[1]*180/M_PI,rpy6[1]*180/M_PI);
	double roll[6]  = {rpy1[0],rpy2[0],rpy3[0],rpy4[0],rpy5[0],rpy6[0]};
	double pitch[6] = {rpy1[1],rpy2[1],rpy3[1],rpy4[1],rpy5[1],rpy6[1]};
	r = giveMedian(roll, 6);
	p = giveMedian(pitch, 6);
	rpy_angles = {r, p, 0};
	return rpy_angles;
}


Vector3f normalToEuler(Vector3f plane_normal, double n, std::vector<double> imu_r_p)
{
	Vector3f rpy_angles;
	if(n < 1e-6) {
		rpy_angles[0] = 0;
		rpy_angles[1] = 0;
		rpy_angles[2] = 0;
		return rpy_angles;
	}
	plane_normal = plane_normal/n;

	Vector3f vx{1,0,0};
	Vector3f vx_world{0,0,0};
	Vector3f plane_normal_world{0,0,0};

#if 1	
	double imu_rot[3][3]= {{	cos(imu_r_p.at(1)),     sin(imu_r_p.at(1))*sin(imu_r_p.at(0)),  sin(imu_r_p.at(1))*cos(imu_r_p.at(0))},
		{ 			     		 0,   			   cos(imu_r_p.at(0)),  		  -sin(imu_r_p.at(0))},
		{ 		       -sin(imu_r_p.at(1)),     cos(imu_r_p.at(1))*sin(imu_r_p.at(0)),  cos(imu_r_p.at(0))*cos(imu_r_p.at(1))}};
	
#endif
#if 0
	double imu_rot[3][3]= {{			1,     0,  0},
		{ 			     		0,     1,  0},
		{ 		       			0,     0,  1}};
	
#endif	
	for(int j = 0; j < 3; j++)
	{
		for(int k = 0; k < 3; k++)
		{
			vx_world[j] += imu_rot[j][k] * vx[k];
			plane_normal_world[j] += imu_rot[j][k] * plane_normal[k];
		}
	}
	Vector3f y_est; y_est = cross(plane_normal_world,vx_world);
	Vector3f x_est; x_est = cross(y_est,plane_normal_world);

	Matrix3f rot_mat_support_plane; rot_mat_support_plane = transpose(x_est,y_est,plane_normal_world);
	rpy_angles = rotation_to_euler(rot_mat_support_plane);

	return rpy_angles;
}


Vector3f planeNormalThreePoint(Vector3f pt_a,Vector3f pt_b,Vector3f pt_c){

	Vector3f vect_A,vect_B,cross_P;
	double vec_norm;

	vect_A = pt_b - pt_a;
	vect_B = pt_c - pt_a;

	cross_P = cross(vect_A, vect_B);

	// Unit vector
	vec_norm = norm(cross_P);
	if(vec_norm != 0)
		cross_P = cross_P/vec_norm;

	return cross_P;
}

Vector3f planeNormalFourPoint(Vector3f pt_a, Vector3f pt_b, Vector3f pt_c, Vector3f pt_d){

	Vector3f vect_A,vect_B,cross_P;
	double	vec_norm;

	vect_A = pt_a - pt_b;
	vect_B = pt_c - pt_d;

	cross_P = cross(vect_A, vect_B);

	// Unit vector
	vec_norm = norm(cross_P);
	if(vec_norm != 0)
		cross_P = cross_P/vec_norm;

	return cross_P;
}


void estimateSupportPlaneRollPitch(Matrix3f rot,Vector3f plane_normal, double *plane_roll, double *plane_pitch)
{
	Vector3f vx{1,0,0};
	Vector3f y_cap_of_support_plane = cross(plane_normal,transformation(rot,vx));
	Vector3f x_cap_of_support_plane = cross(y_cap_of_support_plane,plane_normal);

	Matrix3f rot_mat_support_plane = transpose(x_cap_of_support_plane,y_cap_of_support_plane,plane_normal);

	Vector3f euler_angles_of_support_plane = rotation_to_euler(rot_mat_support_plane);

	*plane_roll = euler_angles_of_support_plane[0];
	*plane_pitch = euler_angles_of_support_plane[1];
}


#define FL 0
#define FR 1
#define BL 2
#define BR 3

Vector3f getTerrainNormal(std::array<Vector3f, 4> foot_pos){
	return planeNormalFourPoint(foot_pos[FL], foot_pos[BR], foot_pos[FR], foot_pos[BL]);
}
