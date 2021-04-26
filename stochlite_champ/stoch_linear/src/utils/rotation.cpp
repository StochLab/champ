#include "stoch_linear/utils/rotation.h"
#include "math.h"

using namespace mat;

Matrix3f euler_to_rotation(double roll, double pitch, double yaw){

	Matrix3f Rx {1, 0, 0,
				 0, cos(roll), -sin(roll),
				 0, sin(roll), cos(roll)};

	Matrix3f Ry {cos(pitch), 0, sin(pitch),
				 0, 1, 0,
				 -sin(pitch), 0, cos(pitch)};

	Matrix3f Rz {cos(yaw), -sin(yaw), 0,
				 sin(yaw), cos(yaw), 0,
				 0, 0, 1};

	Matrix3f R{0};

	R = Rz*Ry*Rx;

	return R;


}

Vector3f rotation_to_euler(Matrix3f R){

	double sy = sqrt(R[0][0] * R[0][0] +  R[1][0] * R[1][0]);
	Vector3f ea;
	int singular = sy < 1e-6?1:0;

	if(!singular)
	{
		ea[0] = atan2(R[2][1], R[2][2]);
		ea[1] = atan2(-R[2][0], sy);
		ea[2] = atan2(R[1][0], R[0][0]);
	}
	else
	{
		ea[0] = atan2(R[0][1], R[1][1]);
		ea[1] = 3.1415/2.0f;
		ea[2] = 0;
	}
	return ea;
}

Vector3f transformation(Matrix3f rot, Vector3f x)
{
	Vector3f x_out;

	for(int i=0;i<3;i++)
	{
		x_out[i] = 0;
		for (int j = 0; j < 3; ++j)
		{
			x_out[i]+=rot[i][j]*x[j];
		}
	}

	return x_out;
}
