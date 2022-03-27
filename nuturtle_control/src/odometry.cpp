/// \file
/// \brief Main: Publishes Odometry messages for diff drive robot based on wheel joint states
///
/// PUBLISHES:
///   odom_pub (nav_msgs::Odometry): publishes odometry messages like position and twist
/// SUBSCRIBES:
///   js_sub (sensor_msgs::JointState), which records joint states of the robot
///

#include <string>
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/set_pose.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

static turtlelib::DiffDrive dd;
static nav_msgs::Odometry odom;
static geometry_msgs::TransformStamped trans;
static bool started(false);

/// \brief Updates internal odometry state, publishes a ROS odometry message, broadcast the transform between odometry and body frame on tf
/// \param msg - a pointer to the sensor_msg/JointState message with angles and angular velocities of the robot wheels
void jsCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
	using namespace turtlelib;
	
	static ros::NodeHandle nh;
	static tf2_ros::TransformBroadcaster br;
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	// update position in robot
	double l_phi_wheel_new = msg -> position[0];
	double r_phi_wheel_new = msg -> position[1];

	if (started)
	{
		double dt = (current_time - last_time).toSec();
		Twist2D tw = dd.wheelsToTwist(l_phi_wheel_new, r_phi_wheel_new, dt);
		odom.twist.twist.linear.x = tw.getVx();
		odom.twist.twist.linear.y = tw.getVy();
		odom.twist.twist.angular.z = tw.getW();
	}
	else
	{
		odom.twist.twist.linear.x = 0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0;
	}

	dd.updatePoseWithSlip(l_phi_wheel_new, r_phi_wheel_new);

	// start referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)	
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = dd.getX();
	odom.pose.pose.position.y = dd.getY();
	odom.pose.pose.position.z = 0.0;
	
	//geometry_msgs::Quaternion odow_posuat = tf::createQuaternionMsgFromYaw(dd.getTheta());
	tf2::Quaternion q;
	q.setRPY(0, 0, dd.getTheta());
	
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();
	
	static ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	pub.publish(odom);
	
	trans.header.stamp = current_time;
	trans.transform.translation.x = dd.getX();
	trans.transform.translation.y = dd.getY();
	trans.transform.translation.z = 0.0;
	trans.transform.rotation.x = q.x();
	trans.transform.rotation.y = q.y();
	trans.transform.rotation.z = q.z();
	trans.transform.rotation.w = q.w();
	br.sendTransform(trans);
		
	last_time = current_time;
	started = true;
	// end referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)
}

/// \brief following a set_pose service, reset location of odometry to match requested configuration
/// \param req - service request package of type nuturtle_control::set_pose::Request (containing robot pose)
/// \param res - service response package of type nuturtle_control::set_pose::Response (empty)
/// \return true if services were successfully called, else false
bool set_pose_method(nuturtle_control::set_pose::Request & req, nuturtle_control::set_pose::Response &)
{
	using namespace turtlelib;
	using namespace nuturtle_control;
	
	WheelPos q;
	q.theta = req.theta;
	q.x = req.x;
	q.y = req.y;
	dd.setPose(q);
	return true;
}

int main(int argc, char** argv)
{
	using namespace turtlelib;
	using namespace nuturtle_control;
	
	ros::init(argc, argv, "odometry");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joint_states", 10, jsCallback);
	
	double wheel_base;
	double wheel_radius;
	std::string odom_frame_id;
	std::string body_frame_id;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("odom_frame_id", odom_frame_id);
	n.getParam("body_frame_id", body_frame_id);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint);
	
	odom.header.frame_id = odom_frame_id;
	odom.child_frame_id = body_frame_id;
	trans.header.frame_id = odom_frame_id;
	trans.child_frame_id = body_frame_id;
	
	dd.setPhysicalParams(wheel_base, wheel_radius);
	ros::ServiceServer srv = n.advertiseService("set_pose", set_pose_method);
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
	}
	
	return 0;
}
