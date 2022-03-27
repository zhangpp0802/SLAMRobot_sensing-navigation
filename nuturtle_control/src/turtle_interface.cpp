/// \file
/// \brief low-level control and sensor routines
///
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs/WheelCommands): makes the turtlebot follow the specified twist
///		joint_states (sensor_msgs/JointState): provides angle (rad) and velocity (rad/s) of the wheels based on encoder data
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded twist
///		sensor_data (nuturtlebot_msgs/SensorData): wheel angles encoder data

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <string>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
// #include "nuturtle_control/set_pose.h"

static turtlelib::DiffDrive dd(0.16, 0.033);  // using the turtlebot3 burger wheel base and wheel radius
static bool started(false);
static bool publish_cmd_vel(false);
static bool publish_sensor_data(false);
static nuturtlebot_msgs::WheelCommands wheel_cmds;
static sensor_msgs::JointState joint;


/// \brief converts an angular wheel speed to a command value in range [-256, 256]
/// \param ang_speed - the wheel angular speed in rad/s
/// \return a motor command value in range [-256, 256] proportional to the commanded angular wheel speed
int speedCommand(double ang_speed)
{
	using namespace turtlelib;
	const double max_speed = 57.0*2.0*PI/60.0;
	int command = 256 * ang_speed/max_speed;
	if (command > 256)
	{
		command = 256;
	}
	if (command < -256)
	{
		command = -256;
	}
	return command;
}

/// \brief cmd_vel subscriber callback that will make the turtlebot3 follow the specified twist
///
/// \param msg (geometry_msgs/Twist): gives linear and angular velocity
/// \ publishes  wheel velocites (rigid2d::WheelVelocities --> nuturtlebot:WheelCommands)
void callback_vel(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace turtlelib;
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	WheelVel wv = dd.twist2Wheel(tw);
	wheel_cmds.left_velocity = speedCommand(wv.left);
	wheel_cmds.right_velocity = speedCommand(wv.right);
	publish_cmd_vel = true;
}

/// \brief normalize the encoder value
double encoder2rad(int encoder)
{
	using namespace turtlelib;
	const int max_count = 4096;
	double rad = normalize_angle(encoder*2.0*PI/max_count);
	return rad;
}

/// \brief sns subscriber callback. Records left and right wheel angles
///
/// \param sns (nuturtlebot/SensorData): the left and right wheel joint sensor data 
/// wheel_angle and wheelVel_measured (turtlelib::WheelVelocities): measured wheel angles and velocities respct.
void callback_sensor(const nuturtlebot_msgs::SensorData::ConstPtr & msg)
{
	using namespace turtlelib;
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	joint.name = {"wheel_left_joint", "wheel_right_joint"};
	
	static double left_rad = 0.0;
	left_rad = encoder2rad(msg -> left_encoder);
	static double right_rad = 0.0;
	right_rad = encoder2rad(msg -> right_encoder);
	static double left_rad_prev = 0.0;
	static double right_rad_prev = 0.0;
	
	dd.updatePoseWithSlip(left_rad, right_rad);
	joint.header.stamp = current_time;
	joint.position = {left_rad, right_rad};
	
	if (!started)
	{
		joint.velocity[0] = 0.0;
		joint.velocity[1] = 0.0;
		started = true;
	}
	else
	{
		double dt = (current_time - last_time).toSec();
		joint.velocity = {normalize_angular_difference(left_rad, left_rad_prev)/dt, normalize_angular_difference(right_rad, right_rad_prev)/dt};
	}
	
	last_time = current_time;
	left_rad_prev = left_rad;
	right_rad_prev = right_rad;
	publish_sensor_data = true;
}	

int main(int argc, char** argv)
{
	using namespace turtlelib;
	
	ros::init(argc, argv, "turtle_interface");
	ros::NodeHandle n;
	ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 10, callback_vel);
	ros::Subscriber sub_sensor_data = n.subscribe("sensor_data", 10, callback_sensor);	
	ros::Publisher pub_cmd_vel = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 10);
	ros::Publisher pub_sensor_data = n.advertise<sensor_msgs::JointState>("joint_states", 10);
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
		if (publish_cmd_vel)
		{
			pub_cmd_vel.publish(wheel_cmds);
			publish_cmd_vel = false;
		}
		if (publish_sensor_data)
		{
			pub_sensor_data.publish(joint);
			publish_sensor_data = false;
		}
	}
	
	return 0;
}
