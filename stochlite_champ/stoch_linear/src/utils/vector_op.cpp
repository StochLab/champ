#include "stoch_linear/utils/vector_op.h"
#include <math.h>


Vector3f cross(Vector3f a, Vector3f b){

	Vector3f c;

	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];

	return c;
}

double norm(Vector3f a) {
	return (sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]));
}
