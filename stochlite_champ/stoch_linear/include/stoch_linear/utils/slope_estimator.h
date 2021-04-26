#ifndef __SLOPE_ESTIMATOR_H__
#define __SLOPE_ESTIMATOR_H__

#include "stoch_linear/utils/vector_op.h"
#include "stoch_linear/utils/matrix_op.h"
#include <vector>

using namespace std;
using namespace mat;

Vector3f slope_estimator(std::vector<double> z, std::vector<double> imu_r_p);

Vector3f normalToEuler(Vector3f plane_normal, double n, std::vector<double> imu_r_p);

void estimateSupportPlaneRollPitch(Matrix3f rot,Vector3f plane_normal, double *plane_roll, double *plane_pitch);

Vector3f getTerrainNormal(std::array<Vector3f, 4> foot_pos);

#endif // __SLOPE_ESTIMATOR_H__
