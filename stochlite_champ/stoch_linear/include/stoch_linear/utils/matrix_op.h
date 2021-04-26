
#ifndef MATRIX_OP_H
#define MATRIX_OP_H

#include <array>
#include "stoch_linear/utils/vector_op.h"

namespace mat
{
int mat_mul(const double m1[][2], const double m2[][2], double mres[][2]);
int mat_add(const double m1[][2], const double m2[][2], double mres[][2]);
int mat_inv(const double m1[][2], double mres[][2]); 
int mat_transpose(const double m1[][2], double mres[][2]);
int mat_scalar_mul(const double m1[][2], double scalar, double mres[][2]);



template<typename T, std::size_t S>
using Matrix = std::array<std::array<T, S>, S>;

using Matrix3f = std::array<std::array<double, 3>, 3>;


template<typename T, std::size_t S>
auto operator*(const Matrix<T,S>& a, const Matrix<T,S>& b)->Matrix<T,S>{

	int i=0, j=0, k=0;
	Matrix<T,S> res{0};

	for(i=0; i<S; i++){
		for(j=0; j<S; j++){
			for(k=0; k<S; k++){
				res[i][j] += a[i][k]*b[k][j];
			}
		}
	}
	return res;
}


template<typename T, std::size_t S>
auto operator*(const Matrix<T,S>& m, const Vector<T,S>& v)->Vector<T,S>{
	int i=0, j=0;
	Vector<T,S> res{0};

	for(i=0;i<S;i++){
		for(j=0;j<S;j++){
			res[i] += m[i][j]*v[j];
		}
	}

	return res;
}
}

#endif // MATRIX_OP_H
