#include "nuslam/filter.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <cmath>

namespace nuslam
{
	arma::Mat<double> A_mat(turtlelib::DiffDrive & dd, turtlelib::Transform2D & t_mb, double l_phi_wheel_new, double r_phi_wheel_new, int n)
	{
		using namespace turtlelib;
		
		Twist2D t = dd.wheelsToTwist(l_phi_wheel_new, r_phi_wheel_new, 1.0);
		arma::Mat<double> A_sub(3, 3, arma::fill::zeros);
		double dx = t.getVx();
		double dth = t.getW();
		double theta = t_mb.getTheta();
		if (almost_equal(dth, 0.0))  // case 1: d(theta)_t == 0
		{
			A_sub(1,0) = -dx*sin(theta);
			A_sub(2,0) = dx*cos(theta);
		}
		else  // case 2: d(theta)_t != 0
		{
			A_sub(1,0) = (cos(theta + dth) - cos(theta))*dx/dth;
			A_sub(2,0) = (sin(theta + dth) - sin(theta))*dx/dth;
		}
		arma::Mat<double> A = arma::join_cols(arma::join_rows(A_sub, arma::zeros(3,2*n)), arma::join_rows(arma::zeros(2*n,3),arma::zeros(2*n,2*n)));
		A += arma::eye(3+2*n,3+2*n);
		return A;
	}


	arma::Mat<double> H_mat(double dx, double dy, int n, int j)
	{
		double d = dx*dx + dy*dy;
		arma::Mat<double> h1 = {{0, -dx/sqrt(d), -dy/sqrt(d)},
								{-1, dy/d, -dx/d}};
		arma::Mat<double> h2 = arma::zeros(2,2*(j-1));
		arma::Mat<double> h3 = {{dx/sqrt(d), dy/sqrt(d)},
								{-dy/d, dx/d}};
		arma::Mat<double> h4 = arma::zeros(2,2*n-2*j);
		arma::Mat<double> H = join_rows(h1,h2,h3,h4);
		return H;
	}
	
	Landmark::Landmark(double r_arg, double phi_arg, int id_arg)
		: r(r_arg)
		, phi(phi_arg)
		, id(id_arg)
	{
	}
	
	void initialize_landmark(turtlelib::Transform2D & t_mb, Landmark & lm, arma::Mat<double> & M)
	{
		double mx = t_mb.getX() + lm.r*cos(lm.phi + t_mb.getTheta());
		double my = t_mb.getY() + lm.r*sin(lm.phi + t_mb.getTheta());
		M(2*(lm.id-1),0) = mx;
		M(2*(lm.id-1)+1,0) = my;
	}
	
	turtlelib::Vector2D inv_meas(turtlelib::Transform2D & t_mb, Landmark & lm)
	{
		using namespace turtlelib;
		double mx = t_mb.getX() + lm.r*cos(lm.phi + t_mb.getTheta());
		double my = t_mb.getY() + lm.r*sin(lm.phi + t_mb.getTheta());
		Vector2D v(mx,my);
		return v;
	}
	
	arma::Col<double> measureLandmark(turtlelib::Transform2D & t_mb, arma::Mat<double> & M, double id)
	{
		using namespace turtlelib;
		double mx = M(2*(id-1),0);
		double my = M(2*(id-1)+1,0);
		double r = sqrt((mx-t_mb.getX())*(mx-t_mb.getX()) + (my-t_mb.getY())*(my-t_mb.getY()));
		double phi = normalize_angular_difference(atan2(my-t_mb.getY(), mx-t_mb.getX()), t_mb.getTheta());
		arma::Col<double> zh = {r, phi};
		return zh;
	}
	
	arma::Mat<double> initialize_Qbar(std::vector<double> & q_cov, double n_lm)
	{
		arma::Mat<double> Q = {	{q_cov[0], q_cov[3], q_cov[4]},
								{q_cov[3], q_cov[1], q_cov[5]},
								{q_cov[4], q_cov[5], q_cov[2]}};
								
		arma::Mat<double> Q_bar = arma::join_cols(arma::join_rows(Q, arma::zeros(3,2*n_lm)), arma::join_rows(arma::zeros(2*n_lm,3), arma::zeros(2*n_lm,	2*n_lm)));
		return Q_bar;
	}
	
	arma::Mat<double> initialize_S(double n_lm)
	{								
		arma::Mat<double> S = {arma::join_cols(arma::join_rows(arma::zeros(3,3), arma::zeros(3,2*n_lm)), arma::join_rows(arma::zeros(2*n_lm,3), 1.0e20*arma::eye(2*n_lm,2*n_lm)))};
		return S;
	}
	
	arma::Mat<double> initialize_R(std::vector<double> & r_cov)
	{
		arma::Mat<double> R = {	{r_cov[0], r_cov[2]},
								{r_cov[2], r_cov[1]}};
		return R;
	}	
}
