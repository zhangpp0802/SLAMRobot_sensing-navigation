#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <sensor_msgs/JointState.h>
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <iterator>

static bool called_trans = false;
static bool called_rot = false;
static bool called_js = false; 

void callback_trans(const nuturtlebot_msgs::WheelCommands::ConstPtr & msg)
{
	if (((msg -> left_velocity !=0) && (msg -> right_velocity != 0)) || called_trans)
	{
		called_trans = true;
		CHECK(msg -> left_velocity == 42);
		CHECK(msg -> right_velocity == 42);
	}
	else
	{
		CHECK(msg -> left_velocity == 0);
		CHECK(msg -> right_velocity == 0);
	}
}

void callback_rot(const nuturtlebot_msgs::WheelCommands::ConstPtr & msg)
{
	if (((msg -> left_velocity !=0) && (msg -> right_velocity != 0)) || called_rot)
	{
		called_rot = true;
		CHECK(msg -> left_velocity == -207);
		CHECK(msg -> right_velocity == 207);
	}
	else
	{
		CHECK(msg -> left_velocity == 0);
		CHECK(msg -> right_velocity == 0);
	}
}

void callback_js(const sensor_msgs::JointState::ConstPtr & msg)
{
	using namespace turtlelib;
	
	if ((std::size(msg -> position) !=0) || called_js)
	{
		called_js = true;
		CHECK(almost_equal(msg -> position[0], 200*PI/4096));
		CHECK(almost_equal(msg -> position[1], 100*PI/4096));
	}
	else
	{
		CHECK(std::size(msg -> position) == 0);
	}
}

TEST_CASE("pure translation cmd_vel", "[cmd_vel]")
{
	ros::NodeHandle nh;
	const auto sub = nh.subscribe("wheel_cmd", 1000, callback_trans);
	const auto pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, true);
	geometry_msgs::Twist tw;
	tw.linear.x = 0.033;
	tw.linear.y = 0.0;
	tw.angular.z = 0.0;
	pub.publish(tw);
	ros::Rate r(100.0);
	for(int i = 0; ros::ok() && i != 200; ++i)
	{
		ros::spinOnce();
		r.sleep();
	}
	CHECK(called_trans);
}

TEST_CASE("pure rotation cmd_vel", "[cmd_vel]")
{
	ros::NodeHandle nh;
	const auto sub = nh.subscribe("wheel_cmd", 1000, callback_rot);
	const auto pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, true);
	geometry_msgs::Twist tw;
	tw.linear.x = 0.0;
	tw.linear.y = 0.0;
	tw.angular.z = 2.0;
	pub.publish(tw);
	ros::Rate r(100.0);
	for(int i = 0; ros::ok() && i != 200; ++i)
	{
		ros::spinOnce();
		r.sleep();
	}
	CHECK(called_rot);
}

TEST_CASE("encoder data to joint_states", "[sensor_data]")
{
	ros::NodeHandle nh;
	const auto sub = nh.subscribe("joint_states", 1000, callback_js);
	const auto pub = nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 1000, true);
	nuturtlebot_msgs::SensorData sd;
	sd.left_encoder = 100;
	sd.right_encoder = 50;
	pub.publish(sd);
	ros::Rate r(100.0);
	for(int i = 0; ros::ok() && i != 200; ++i)
	{
		ros::spinOnce();
		r.sleep();
	}
	CHECK(called_js);
}

