#ifndef __VECTOR_OP_H__
#define __VECTOR_OP_H__

#include <array>
#include <iostream>

template <typename T, std::size_t S>
using Vector = std::array<T, S>;

using Vector3f = std::array<double, 3>;


Vector3f cross(Vector3f a, Vector3f b);
double norm(Vector3f a);

template<typename T, std::size_t S>
auto operator+(const Vector<T,S>& a, const Vector<T,S>& b)->Vector<T,S>{
	int 	i;
	std::array<T, S> out; 
	for(i=0;i<S;i++)
		out[i] = a[i] + b[i];
	return out;
}

template<typename T, std::size_t S>
auto operator-(const std::array<T, S>& a, const std::array<T, S>& b)->std::array<T,S>{
	int 	i;
	std::array<T, S> out; 
	for(i=0;i<S;i++)
		out[i] = a[i] - b[i];
	return out;
}

template<typename T, std::size_t S>
auto operator/(const std::array<T, S>& a, const T& b)->std::array<T,S>{
	int 	i;
	std::array<T, S> out; 
	for(i=0;i<S;i++)
		out[i] = a[i]/b;
	return out;
}


template<typename T, std::size_t S>
std::ostream & operator<<(std::ostream& os, const std::array<T,S>& vec){

	for(int i=0;i<S;i++)
		os << vec[i] << " ";
	os << std::endl;
	return os;
}

#endif // __VECTOR_OP_H__
