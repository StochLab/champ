#include "stochlite_gazebo/tools/matrix_op.h"

namespace stochlite {	
	// Multiplication of 2x2 matrices
	int mat_mul(const double m1[][2], const double m2[][2], double mres[][2]){

		double m11, m12, m21, m22;
		double n11, n12, n21, n22;

		m11 = m1[0][0];
		m12 = m1[0][1];
		m21 = m1[1][0];
		m22 = m1[1][1];

		n11 = m2[0][0];
		n12 = m2[0][1];
		n21 = m2[1][0];
		n22 = m2[1][1];

		mres[0][0]=m11*n11 + m12*n21;
		mres[0][1]=m11*n12 + m12*n22;
		mres[1][0]=m21*n11 + m22*n21;
		mres[1][1]=m21*n12 + m22*n22;

		return 0;
	}

	// Addition of 2x2 matrices
	int mat_add(const double m1[][2], const double m2[][2], double mres[][2]){

		double m11, m12, m21, m22;
		double n11, n12, n21, n22;

		m11 = m1[0][0];
		m12 = m1[0][1];
		m21 = m1[1][0];
		m22 = m1[1][1];

		n11 = m2[0][0];
		n12 = m2[0][1];
		n21 = m2[1][0];
		n22 = m2[1][1];

		mres[0][0]=m11+n11;
		mres[0][1]=m12+n12;
		mres[1][0]=m21+n21;
		mres[1][1]=m22+n22;

		return 0;
	}


	// Inversion of a 2x2 matrix
	int mat_inv(const double m1[][2], double mres[][2]) {

		double det=0;
		double m11, m12, m21, m22;

		m11 = m1[0][0];
		m12 = m1[0][1];
		m21 = m1[1][0];
		m22 = m1[1][1];

		det = m11*m22 - m12*m21;

		mres[0][0]=(1/det)*m22;
		mres[0][1]=(1/det)*(-m12);
		mres[1][0]=(1/det)*(-m21);
		mres[1][1]=(1/det)*m11;

		return 0;
	}


	int mat_transpose(const double m1[][2], double mres[][2]) {

		mres[0][0]=m1[0][0];
		mres[0][1]=m1[1][0];
		mres[1][0]=m1[0][1];
		mres[1][1]=m1[1][1];

		return 0;
	}


	int mat_scalar_mul(const double m1[][2], double scalar, double mres[][2]) {

		mres[0][0]=scalar*m1[0][0];
		mres[0][1]=scalar*m1[0][1];
		mres[1][0]=scalar*m1[1][0];
		mres[1][1]=scalar*m1[1][1];

		return 0;
	}
}