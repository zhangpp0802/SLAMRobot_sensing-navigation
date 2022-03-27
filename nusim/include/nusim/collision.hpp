#ifndef COLLISION_INCLUDE_GUARD_HPP
#define COLLISION_INCLUDE_GUARD_HPP

#include "turtlelib/rigid2d.hpp"
#include <armadillo>
#include <random>
#include <vector>

namespace circles
{
	/// \brief detection for whether a collision happen
	struct Collision
	{
		/// if true, an Collision exists
		bool intersected = false;
		/// x coordinate of the Collision
		double x = 0.0;
		/// y coordinate of the Collision
		double y = 0.0;
	}; 

	/// \brief returns a Collision type
	/// \param p1 - point1
	/// \param p2 - point2
	/// \param radius -radius
	/// \return whether collide
	Collision compute_Collision(turtlelib::Vector2D p1, turtlelib::Vector2D p2, turtlelib::Vector2D p_circle, double radius);
	
	/// \brief helper fcn for comput_Collision
	int sgn(double x);
}

/// \brief Gaussian implementation
class Multivar
{
private:
	// distribution variance matrix
	arma::Mat<double> Q; 
	// variance matrix Q 
	arma::Mat<double> L; 
	// anormal distribution
	std::vector<std::normal_distribution<>> dist_vec;
	
	std::mt19937 & get_random();

public:
	/// \brief construct Gaussain
	/// \param cov - covariance matrix
	Multivar(arma::Mat<double> & cov);
	
	/// \brief draws the vector with Gaussian
	/// \return a Gaussian noise
	std::vector<double> draw();
};

#endif
