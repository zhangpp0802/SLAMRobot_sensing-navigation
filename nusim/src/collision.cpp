#include "nusim/collision.hpp"
#include "turtlelib/rigid2d.hpp"
#include <armadillo>
#include <cmath>

namespace circles
{
	Collision compute_Collision(turtlelib::Vector2D p1, turtlelib::Vector2D p2, turtlelib::Vector2D p_circle, double radius)
	{
		using namespace turtlelib;
		Vector2D p1_p2(p2.x-p1.x, p2.y-p1.y);
		p1.x -= p_circle.x;
		p2.x -= p_circle.x;
		p1.y -= p_circle.y;
		p2.y -= p_circle.y;

		double dx = p2.x - p1.x;
		double dy = p2.y - p1.y;
		double dr = sqrt(dx*dx + dy*dy);
		double D = p1.x*p2.y - p2.x*p1.y;
		double discrim = radius*radius*dr*dr - D*D;
		Collision c;  
		Collision cc; 
		if (discrim < 0)
		{
			c.intersected = false;
			return c;
		}
		else
		{	
			c.x = (D*dy+sgn(dy)*dx*sqrt(discrim))/(dr*dr);
			cc.x = (D*dy-sgn(dy)*dx*sqrt(discrim))/(dr*dr);
			c.y = (-D*dx+std::abs(dy)*sqrt(discrim))/(dr*dr);
			cc.y = (-D*dx-std::abs(dy)*sqrt(discrim))/(dr*dr);
			Vector2D p1_c(c.x-p1.x, c.y-p1.y);
			Vector2D p1_cc(cc.x-p1.x, cc.y-p1.y);
			if ((p1_c.x*p1_p2.x + p1_c.y*p1_p2.y) < 0) 
			{
				c.intersected = false;
				return c;
			}
			else
			{
				c.intersected = true;
				cc.intersected = true;
				if (magnitude(p1_c) < magnitude(p1_cc))
				{
					// translate
					c.x += p_circle.x;
					c.y += p_circle.y;
					return c;
				}
				else
				{
					// translate
					cc.x += p_circle.x;
					cc.y += p_circle.y;
					return cc;
				}
			}
		}
	}
	
	int sgn(double x)
	{
		if (x < 0)
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}
}

Multivar::Multivar(arma::Mat<double> & cov)
{
	Q = cov;
	L = arma::chol(Q, "lower");
	for (std::size_t i = 0; i < cov.n_rows; ++i)
	{
		std::normal_distribution<> d(0.0, cov(i,i));
		dist_vec.push_back(d);
	}
}

// reference from lecture note
std::mt19937 & Multivar::get_random()
{
	static std::random_device rd{}; 
	static std::mt19937 mt{rd()};
	return mt;
}

std::vector<double> Multivar::draw()
{
	arma::Mat<double> u(Q.n_rows, 1);
	for (std::size_t i = 0; i < Q.n_rows; ++i)
	{
		u[i] = dist_vec[i](get_random());
	}
	return (arma::conv_to<std::vector<double>>::from(L*u));
}
