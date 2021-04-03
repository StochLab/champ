#ifndef __ROTATION__
#define __ROTATION__

#include "stochlite_gazebo/tools/vector_op.h"
#include "stochlite_gazebo/tools/matrix_op.h"
#include <math.h>

namespace stochlite {
    Matrix3f euler_to_rotation(double roll, double pitch, double yaw);
    Vector3f rotation_to_euler(Matrix3f R);
    Vector3f transformation(Matrix3f rot, Vector3f x);
}

#endif // __ROTATION__
