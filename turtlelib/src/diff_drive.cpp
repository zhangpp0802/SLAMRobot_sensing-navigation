#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
	DiffDrive::DiffDrive(double wheel_base, double wheel_radius, double l_wheel_phi, double r_wheel_phi)
		: m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_l_wheel_phi(l_wheel_phi)
		, m_r_wheel_phi(r_wheel_phi)
	{
	}
	
	DiffDrive::DiffDrive(double wheel_base, double wheel_radius)
		: m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_l_wheel_phi(0.0)
		, m_r_wheel_phi(0.0)
	{
	}
	
	DiffDrive::DiffDrive()
		: m_l_wheel_phi(0.0)
		, m_r_wheel_phi(0.0)
	{
	}
	
	void DiffDrive::setPhysicalParams(double wheel_base, double wheel_radius)
	{
		m_wheel_base = wheel_base;
		m_wheel_radius = wheel_radius;
	}
	
	void DiffDrive::updatePoseWithSlip(double l_wheel_phi_new, double r_wheel_phi_new, double l_slip, double r_slip)
	{
		// Determine wheel angle change
		double d_phi_l = normalize_angular_difference(normalize_angle(l_wheel_phi_new), m_l_wheel_phi);
		double d_phi_r = normalize_angular_difference(normalize_angle(r_wheel_phi_new), m_r_wheel_phi);
		// Update absolute wheel angles
		m_l_wheel_phi += d_phi_l;
		m_r_wheel_phi += d_phi_r;
		m_l_wheel_phi = normalize_angle(m_l_wheel_phi);
		m_r_wheel_phi = normalize_angle(m_r_wheel_phi);
		// Compute body twist (unit time)
		double d_theta_b = (d_phi_r*(1-r_slip) - d_phi_l*(1-l_slip)) * m_wheel_radius / m_wheel_base;  // Equations 2-8
		double d_x_b = (d_phi_r*(1-r_slip) + d_phi_l*(1-l_slip)) * m_wheel_radius / 2;  // Equation 1 
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b);  
		Vector2D v;  
		Transform2D t_a(v, w_pos.theta);
		Twist2D d_q = t_a.change_twist_frame(d_q_b);
		// Update pose
		w_pos.theta += d_q.getW();
		normalize_angle(w_pos.theta);
		w_pos.x += d_q.getVx();
		w_pos.y += d_q.getVy();
	}
	
	void DiffDrive::translatePose(WheelPos q)
	{
		w_pos.x += q.x;
		w_pos.y += q.y;
		w_pos.theta += q.theta; 
		normalize_angle(w_pos.theta);
	}
	
	void DiffDrive::setPose(WheelPos q)
	{
		w_pos = q;
	}
	
	Twist2D DiffDrive::wheelsToTwist(double l_wheel_phi_new, double r_wheel_phi_new, double dt) const
	{
		// Determine wheel angle change
		double d_phi_l = l_wheel_phi_new - m_l_wheel_phi;
		double d_phi_r = r_wheel_phi_new - m_r_wheel_phi;
		// Compute body twist (unit time)
		double d_theta_b = ((d_phi_r - d_phi_l) * m_wheel_radius / m_wheel_base)/dt;  // Equations 2-8 (separating changes in position and time)
		double d_x_b = ((d_phi_r + d_phi_l) * m_wheel_radius / 2)/dt;  // Equation 1 (separating changes in position and time)
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b); 
		return d_q_b;
	}
	
	WheelVel DiffDrive::twist2Wheel(const Twist2D & tw) const
	{
		WheelVel wv;
		wv.left = (2*tw.getVx() - tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);  // Equations 13-14
		wv.right = (2*tw.getVx() + tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);  // Equations 9-12
		return wv;
	}
	
	double DiffDrive::getTheta() const
	{
		return w_pos.theta;
	}
		
	double DiffDrive::getX() const
	{
		return w_pos.x;
	}
	
	double DiffDrive::getY() const
	{
		return w_pos.y;
	}
	
	double DiffDrive::getrWheelPhi() const
	{
		return m_r_wheel_phi;
	}
	
	double DiffDrive::getlWheelPhi() const
	{
		return m_l_wheel_phi;
	}
}
