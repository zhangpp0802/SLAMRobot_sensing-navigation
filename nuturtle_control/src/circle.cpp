/// \file   lets the robot drive in a circle 
/// \brief
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs/Twist): Command twist
///
/// SERVICES:
///   control (nuturtle_robot/control) - the direction of the robot
///   stop (nuturtle_robot/stop) - stop motion
///   reverse (nuturtle_robot/reverse) - reverse direction

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <nuturtle_control/control.h>
#include <nuturtle_control/reverse.h>
#include <nuturtle_control/stop.h>

static ros::ServiceServer ControlServices;
static ros::ServiceServer ReverseServices;
static ros::ServiceServer StopServices;
geometry_msgs::Twist twist;


bool serviceCallback(nuturtle_control::control::Request &req, nuturtle_control::control::Response &res)
{
    twist.linear.x = req.velocity;
    if (turtlelib::almost_equal(req.radius,0.0))
    {
        throw std::logic_error("radius should not be zero.");
    }
    twist.angular.z = req.velocity/req.radius;
    return true;
}

bool reverseCallback(nuturtle_control::reverse::Request &req ,nuturtle_control::reverse::Response &)
{
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;
    return true;
}

bool stopCallback(nuturtle_control::stop::Request &req ,nuturtle_control::stop::Response &)
{
    twist.angular.z = 0.0;
    return true;
}

int main(int argc, char** argv)
{
	using namespace turtlelib;
	
	ros::init(argc, argv, "circle");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	double circle_radius = 0.15;
	double speed = 0.1;
	
	np.getParam("circle_radius", circle_radius);
	np.getParam("speed", speed);
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	ControlServices = n.advertiseService("control",serviceCallback);
    ReverseServices = n.advertiseService("reverse",reverseCallback);
    StopServices = n.advertiseService("stop",stopCallback);
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		pub.publish(twist);
		ros::spinOnce(); 
		r.sleep();
	}
	
	return 0;
}
	
