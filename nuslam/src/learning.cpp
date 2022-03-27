#include <vector>
#include <cmath>
#include <armadillo>
#include "nuslam/learning.hpp"
#include "turtlelib/rigid2d.hpp"

namespace nuslam 
{	
	std::vector<std::vector<int>> cluster_range(const std::vector<float> & ranges, double thresh, int min_n)
	{
		std::vector<std::vector<int>> clusters_arr;
		std::vector<int> cluster;
		for (long unsigned int i=0; i<ranges.size(); ++i)
		{	
			if (cluster.empty())
			{
				cluster.push_back(i);
			}
			else
			{
				if (std::abs(ranges[cluster.back()] - ranges[i]) < thresh)
				{
					cluster.push_back(i);
				}
				else
				{
					clusters_arr.push_back(cluster);
					cluster.clear();
					cluster.push_back(i);
				}
			}
			if (i == ranges.size() - 1) 
			{
				if (std::abs(ranges[0] - ranges[i]) < thresh)
				{
					for (std::size_t j=0; j<cluster.size(); ++j)
					{
						clusters_arr[0].push_back(cluster[j]);
					}
				}
				else
				{
					clusters_arr.push_back(cluster);
				}
			}
		}
		
		for (std::size_t k=0; k<clusters_arr.size(); ++k)
		{
			if ((int)clusters_arr[k].size() < min_n)
			{
				clusters_arr.erase(clusters_arr.begin()+k);
				k -= 1;
			}
		}
		return clusters_arr;
	}
	
	bool is_circle(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta, float min_mean, float max_mean, float max_std)
	{
		using namespace turtlelib;
		
		float r1 = ranges[cluster[0]]; 
		float r2 = ranges[cluster.back()]; 
		int cluster_size = cluster.size();
		Vector2D p1(r1, 0);  
		Vector2D p2(r2*cos(dtheta*(cluster_size-1)), r2*sin(dtheta*(cluster_size-1)));
		std::vector<float> angles;
		for (int i=1; i<(cluster_size-2); ++i)
		{
			float ri = ranges[cluster[i]];
			Vector2D pi(ri*cos(dtheta*i), ri*sin(dtheta*i));
			Vector2D i1 = p1 - pi;
			Vector2D i2 = p2 - pi;
			angles.push_back(normalize_angular_difference(angle(i2), angle(i1)));
		}
		float angle_avg = mean(angles);
		float angle_std = stdev(angles);
		if ((angle_std <= max_std) && (angle_avg >= min_mean) && (angle_avg <= max_mean))
		{
			return true;
		}
		else
		{
			return false;
		}
	}	
	
	template <typename T>
	T mean(const std::vector<T> & v)
	{
		T sum = 0.0;
		for (std::size_t i=0; i<v.size(); ++i)
		{
			sum += v[i];
		}
		return (sum/v.size());
	}
	
	template <typename T>
	T stdev(const std::vector<T> & v)
	{
		T avg = mean(v);
		T ssq = 0.0;
		for (std::size_t i=0; i<v.size(); ++i)
		{
			ssq += ((v[i] - avg)*(v[i] - avg));
		}
		return (sqrt(ssq)/(v.size()-1));
	}
	
	std::vector<turtlelib::Vector2D> range2xy(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta)
	{
		using namespace turtlelib;
		std::vector<Vector2D> v_xy;
		for (std::size_t i=0; i<cluster.size(); ++i)
		{
			Vector2D v(ranges[cluster[i]]*cos(dtheta*cluster[i]), ranges[cluster[i]]*sin(dtheta*cluster[i]));
			v_xy.push_back(v);
		}
		return v_xy;
	}
	
	Circle fit_circle(const std::vector<turtlelib::Vector2D> & pts)
	{
		using namespace turtlelib;
		std::size_t n = pts.size();
		
		Vector2D avg_pt;
		for (std::size_t i=0; i<n; ++i)
		{
			avg_pt += pts[i];
		}
		avg_pt *= (1.0/n);
		
		std::vector<double> x;
		std::vector<double> y;
		std::vector<double> z;
		for (std::size_t i=0; i<n; ++i)
		{
			double xi = pts[i].x-avg_pt.x;
			double yi = pts[i].y-avg_pt.y;
			double zi = xi*xi + yi*yi;
			x.push_back(xi);
			y.push_back(yi);
			z.push_back(zi);
		}	
		double z_avg = mean(z);
		
		arma::Col<double> xcol(x);
		arma::Col<double> ycol(y);
		arma::Col<double> zcol(z);
		arma::Mat<double> Z = arma::join_rows(zcol,xcol,ycol,arma::ones(n,1));
		arma::Mat<double> M = (1.0/n)*Z.t()*Z;
		arma::Mat<double> H = {	{8*z_avg, 0, 0, 2},
								{0,	1, 0, 0},
								{0, 0, 1, 0},
								{2, 0, 0, 0}};
		arma::Mat<double> Hinv = {	{0, 0, 0, 0.5},
									{0, 1, 0, 0},
									{0, 0, 1, 0},
									{0.5, 0, 0, -2*z_avg}};
		arma::Mat<double> U;
		arma::Mat<double> V;
		arma::Col<double> s;
		arma::svd(U,s,V,Z);
		arma::Mat<double> A;
		if (s(3) > 1.0e-12)
		{
			arma::Mat<double> Y = V*arma::diagmat(s)*V.t();
			arma::Mat<double> Q = Y*Hinv*Y;
			arma::Col<std::complex<double>> eigval;
			arma::Mat<std::complex<double>> eigvec;
			arma::eig_gen(eigval, eigvec, Q);
			int min_eig_ind = 0;
			float min_eig = 1000;
			for (int i=0; i<(int)eigval.size(); ++i)
			{
				float current_eig = std::real(eigval(i));
				if ((current_eig < min_eig) && (current_eig > 0))
				{
					min_eig = current_eig;
					min_eig_ind = i;
				}
			}
			arma::Col<std::complex<double>> Astar_c = eigvec.col(min_eig_ind);
			arma::Col<double> Astar((int)Astar_c.size());
			for (int i=0; i<(int)Astar_c.size(); ++i)
			{
				Astar(i) = std::real(Astar_c(i));
			}
			A = arma::inv(Y)*Astar;
		}
		else
		{
			A = V.col(3);
		}
		Circle c;
		c.x = -A(1)/(2*A(0)) + avg_pt.x;
		c.y = -A(2)/(2*A(0)) + avg_pt.y;
		c.r = sqrt((A(1)*A(1) + A(2)*A(2) - 4*(A(0)*A(3)))/(4*A(0)*A(0)));
		return c;
	}
}
