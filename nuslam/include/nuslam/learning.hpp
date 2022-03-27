#ifndef LEARNING_INCLUDE_GUARD_HPP
#define LEARNING_INCLUDE_GUARD_HPP
/// \file
/// \brief process laser data by supervised learning

#include <vector>
#include "turtlelib/rigid2d.hpp"

namespace nuslam
{
	/// \brief structure to represent a circle on a plane
	struct Circle
	{
		double x = 0.0;
		double y = 0.0;
		double r = 1.0;
	};	
	
	/// \brief array of laser ranges
	/// \param ranges - array of range data
	/// \param thresh - threshold 
	/// \param min_n - minimum number of data
	/// \return a vector of vectors
	std::vector<std::vector<int>> cluster_range(const std::vector<float> & ranges, double thresh, int min_n);
	
	/// \brief classifies a bunch of points
	/// \param ranges - range datas
	/// \param cluster - indices
	/// \param dtheta - angular increments
	/// \param min_mean - minimum value
	/// \param max_mean - maximum value
	/// \param max_std - standard deviation max
	/// \return true if Circle, false if Not Circle
	bool is_circle(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta, float min_mean, float max_mean, float max_std);
	
	/// \brief computes the mean of a vector
	/// \param v - a vector
	/// \return mean of v
	template <typename T>
	T mean(const std::vector<T> & v);
	
	/// \brief standard deviation
	/// \param v - given vector
	/// \return standard deviation
	template <typename T>
	T stdev(const std::vector<T> & v);
	
	/// \brief converts range data in a cluster into relative xy coordinates
	/// \param ranges - range datas
	/// \param cluster - indices
	/// \param dtheta - angular increment
	/// \return vector representing relative xy coors
	std::vector<turtlelib::Vector2D> range2xy(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta);
	
	/// \brief fits circle
	/// \param pts - xy coors
	/// \return fitted result
	Circle fit_circle(const std::vector<turtlelib::Vector2D> & pts);
}

#endif
