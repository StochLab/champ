#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "stoch_linear/utils/vector_op.h"
#include "stoch_linear/utils/matrix_op.h"

#include <math.h>

Matrix3f euler_to_rotation(double roll, double pitch, double yaw);
Vector3f rotation_to_euler(Matrix3f R);
Vector3f transformation(Matrix3f rot, Vector3f x);


#endif // __ROTATION_H__
