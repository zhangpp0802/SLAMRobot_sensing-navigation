#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief model the kinematics of a differential drive robot

#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{	
	struct WheelPos
	{
		double theta = 0.0;
		double x = 0.0;
		double y = 0.0;
	};
	
	struct WheelVel
	{
		double left = 0.0;
		double right = 0.0;
	};
	
	class DiffDrive
	{
	private:
		WheelPos w_pos; 
		double m_wheel_base; 
		double m_wheel_radius;  
		double m_l_wheel_phi;  
		double m_r_wheel_phi;  
	public:

		DiffDrive(double wheel_base, double wheel_radius, double l_wheel_phi, double r_wheel_phi);
		
		DiffDrive(double wheel_base, double wheel_radius);
		
		DiffDrive();
		
		void setPhysicalParams(double wheel_base, double wheel_radius);
		
		void updatePoseWithSlip(double l_wheel_phi_new, double r_wheel_phi_new, double l_slip=0.0, double r_slip=0.0);
		
		void translatePose(WheelPos q);
		
		void setPose(WheelPos q);
		
		Twist2D wheelsToTwist(double l_wheel_phi_new, double r_wheel_phi_new, double dt) const;
		
		WheelVel twist2Wheel(const Twist2D & tw) const;
		
		double getTheta() const;
		double getX() const;
		double getY() const;
		double getlWheelPhi() const;
		double getrWheelPhi() const;
	};
}		
#endif
