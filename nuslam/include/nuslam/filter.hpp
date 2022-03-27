#ifndef FILTER_INCLUDE_GUARD_HPP
#define FILTER_INCLUDE_GUARD_HPP
/// \file
/// \brief helper functions to perform calculations for slam

#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <cmath>

namespace nuslam 
{
	/// \brief compute the matrix A (derivative of state update with respect to the state)
	/// \param dd - the diff drive robot odometry at t-1
	/// \param t_mb - the map to base_footprint transform at t-1
	/// \param l_phi_wheel_new - the angle of the robot left wheel at time t
	/// \param r_phi_wheel_new - the angle of the robot right wheel at time t
	/// \param n - the number of landmarks
	/// \return the matrix of A as an armadillo matrix
	arma::Mat<double> A_mat(turtlelib::DiffDrive & dd, turtlelib::Transform2D & t_mb, double l_phi_wheel_new, double r_phi_wheel_new, int n);
	
	/// \brief compute the matrix H_j
	/// \param dx - estimated relative x distance
	/// \param dy - estimated relative y distance
	/// \param n - the number of landmarks
	/// \param j - the current landmark index (1 - n)
	arma::Mat<double> H_mat(double dx, double dy, int n, int j);
	
	/// \brief landmark information
    struct Landmark
    {
        double r;
        double phi;  
        int id;
        
        /// \brief a landmark
        /// \param r_arg - range 
        /// \param phi_arg - bearing
        /// \param id_arg - landmark id
        Landmark(double r_arg, double phi_arg, int id_arg);
    };
    
    /// \brief initialize landmark in state estimate vector of the map
    /// \param t_mb - the map to base_footprint transform at t
    /// \param lm - the landmark to initialize
    /// \param M - the state estimate vector of the map at time t-1
    void initialize_landmark(turtlelib::Transform2D & t_mb, Landmark & lm, arma::Mat<double> & M);
    
    /// \brief invert range-bearing measurement to extract coordinates of landmark
    /// \param t_mb - the map to base_footprint transform at t
    /// \param lm - the landmark
    /// \return a vector containing landmark coordinates from measurement inversion
    turtlelib::Vector2D inv_meas(turtlelib::Transform2D & t_mb, Landmark & lm);
    
    /// \brief compute theoretical measurement for a landmark
    /// \param t_mb - the map to base_footprint transform at t
    /// \param M - the state estimate vector of the map at time t-1
    /// \param id - the landmark id
    /// \return theoretical measurement vector [r, phi]^T
    arma::Col<double> measureLandmark(turtlelib::Transform2D & t_mb, arma::Mat<double> & M, double id);
    
    /// \brief initialize the matrix Q_bar, used in SLAM algorithm
    /// \param q_cov - the vector of process noise covariances in the format [sigma_theta, sigma_x, sigma_y, sigma_xtheta, sigma_ytheta, sigma_xy]
    /// \param n_lm - the number of landmarks to track
    /// \return the (2*n_lm+3)x(2*n_lm+3) matrix Q_bar
    arma::Mat<double> initialize_Qbar(std::vector<double> & q_cov, double n_lm);
    
    /// \brief the matrix sigma
    /// \param n_lm - the number of landmarks
    /// \return the (2*n_lm+3)x(2*n_lm+3) form matrix Sigma
    arma::Mat<double> initialize_S(double n_lm);
    
    /// \brief the matrix R
    /// \param r_cov - sensor noise covariances which is a 4 param vector
    /// \return the 2x2 matrix R
    arma::Mat<double> initialize_R(std::vector<double> & r_cov);
}

#endif
