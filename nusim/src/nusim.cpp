/// \file
/// \brief A kinematic simulation of a differential drive robot subject to sensor noise and wheel slippage, showing the location of obstacles in rviz
///
/// PARAMETERS:
///		wheel_base (double): the distance between the robot wheels
///		wheel_radius (double): the radius of the wheels
///		left_wheel_joint (string): the name of the left wheel joint
///		right_wheel_joint (string): the name of the right wheel joint
///		vx_noise (double): Gaussian noise on the commanded linear twist
///		w_noise (double): Gaussian noise on the commanded rotational twist
///		obs_var (std::vector<double>): covariance matrix of x and y relative sensor noise (x, y, xy)
///		ox (std::vector<double>): vector containing x coordinates of the obstacles
///		oy (std::vector<double>): vector containing y coordinates of the obstacles
///		obs_rad (double): radius of the obstacles
///		num_tubes (double): maximum distance beyond which obs are not visible
///		robot_radius (double): collision radius of the robot
///		lidar_min_range (double): minimum object to lidar distance that is sensed
///		lidar_max_range (double): maximum object to lidar distance that is sensed
///		lidar_dtheta (double): angle increment of lidar scan in degrees
///		lidar_n_samples (int): number of samples per lidar scan
///		lidar_resolution (double): resolution of lidar distance measurements
///		lidar_noise (double): Gaussian noise on the lidar distance measurements
///		wall_width (double): width of the simulated border
///		wall_height (double): height of the simulated border
/// PUBLISHES:
///		obstacles (visualization_msgs/MarkerArray): true location of the obs in the environment
///		real_path (nav_msgs/Path): trajectory of the robot
///		fake_sensor (visualization_msgs/MarkerArray): measured position of the obs relative to the robot
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels
///		scan (sensor_msgs/LaserScan): simulated lidar data
///		walls (visualization_msgs/Marker): simulated border lines
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): input twist that causes motion of robot wheels
#include <armadillo>
#include <string>
#include <random>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nusim/collision.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>

static turtlelib::DiffDrive dd;	
static turtlelib::WheelVel wv;
// Noise parameters
static double vx_noise = 0.0001;
static double w_noise = 0.0001;


std::mt19937 & get_random() 
{
	static std::random_device rd{}; 
	static std::mt19937 mt{rd()};
	return mt;
}

/// \brief Updates wheel angular velocities when a Twist message arrives
/// \param msg - a pointer to the geometry_msg/Twist message describing the commanded twist
void cmd_callback(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace turtlelib;
	// normal distributions
	static std::normal_distribution<> vx_gauss(0.0, vx_noise);
	static std::normal_distribution<> w_gauss(0.0, w_noise);
	// add noise to the linear twist
	if ((almost_equal(msg -> linear.x, 0.0, 1e-4)) && (almost_equal(msg -> linear.y, 0.0, 1e-4)) && (almost_equal(msg -> angular.z, 0.0, 1e-4)))  // if all twist entries are zero, do not add noise
	{
		Vector2D v(msg -> linear.x, msg -> linear.y);
		// add noise to the rotational twist
		Twist2D tw(v, msg -> angular.z);
		// update wv
		wv.left = (dd.twist2Wheel(tw)).left;
		wv.right = (dd.twist2Wheel(tw)).right;
	}
	else
	{
		Vector2D v(msg -> linear.x + vx_gauss(get_random()), msg -> linear.y);
		// add noise to the rotational twist
		Twist2D tw(v, msg -> angular.z + w_gauss(get_random()));
		// update wv
		wv.left = (dd.twist2Wheel(tw)).left;
		wv.right = (dd.twist2Wheel(tw)).right;
	}
	// update wv
	// wv.left = (dd.twist2Wheel(tw)).left;
	// wv.right = (dd.twist2Wheel(tw)).right;
	
}



int main(int argc, char** argv)
{
	using namespace turtlelib;
	using namespace circles;
	
	ros::init(argc, argv, "obsnusim");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
	ros::Publisher pub_obs = n.advertise<visualization_msgs::MarkerArray>("obstacles", 10, true);
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("real_path", 10);
	ros::Publisher pub_sensor = n.advertise<visualization_msgs::MarkerArray>("fake_sensor", 10);
	ros::Publisher pub_laser = n.advertise<sensor_msgs::LaserScan>("scan", 10);
	ros::Publisher pub_walls = n.advertise<visualization_msgs::Marker>("walls", 10, true);
	tf2_ros::TransformBroadcaster br;
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmd_callback);
	
	// define variables to store parameters and retrieve parameters
	double wheel_base;
	double wheel_radius;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint); 
	n.getParam("vx_noise", vx_noise);
	n.getParam("w_noise", w_noise);

	dd.setPhysicalParams(wheel_base, wheel_radius);
	
	std::vector<double> ox;
	std::vector<double> oy;
	std::vector<double> obs_var = {0.01, 0.01, 0.0};
	double obs_rad = 0.01;
	double num_tubes = 1.0;
	double slip_max = 0.5;
	double robot_radius = 0.1;
	double lidar_min_range = 0.12;
	double lidar_max_range = 3.5;
	double lidar_dtheta = 1.0;
	int lidar_n_samples = 360;
	double lidar_resolution = 0.015;
	double lidar_noise = 0.01;
	double wall_width = 4.0;
	double wall_height = 4.0;
	n.getParam("ox", ox);
	n.getParam("oy", oy);
	n.getParam("obs_var", obs_var);
	n.getParam("obs_rad", obs_rad);
	n.getParam("num_tubes", num_tubes);
	n.getParam("slip_max", slip_max);
	n.getParam("robot_radius", robot_radius);
	n.getParam("lidar_min_range", lidar_min_range);
	n.getParam("lidar_max_range", lidar_max_range);
	n.getParam("lidar_dtheta", lidar_dtheta);
	n.getParam("lidar_n_samples", lidar_n_samples);
	n.getParam("lidar_resolution", lidar_resolution);
	n.getParam("lidar_noise", lidar_noise);
	n.getParam("wall_width", wall_width);
	n.getParam("wall_height", wall_height);
	
	// critical distance for collision
	double dis_crit = obs_rad + robot_radius;
	
	// define gaussian distributions given ros parameters
	std::uniform_real_distribution<> slip_unif(0, slip_max);
	std::normal_distribution<> laser_gauss(0.0, lidar_noise);
	
	// obtain matrix for x-y sensor noise bivariate Gaussian
	arma::Mat<double> Q = { {obs_var[0], obs_var[2]},
							{obs_var[2], obs_var[1]} };
	Multivar mv_xy(Q);
	
	ros::Rate r(100);
	sensor_msgs::JointState js;
	js.name = {left_wheel_joint, right_wheel_joint};
	
	auto current_time = ros::Time::now();
	auto last_time = ros::Time::now();
	
	// define message to publish transformation between world frame and turtle frame
	geometry_msgs::TransformStamped trans;
	trans.header.frame_id = "world";
	trans.child_frame_id = "odo";
	
	// define message to publish path of turtle frame
	nav_msgs::Path path;
	path.header.frame_id = "world";
	
	// contains true position of the markers, expressed in terms of the world frame
	visualization_msgs::MarkerArray real_marker_arr;
	for(std::size_t i = 0; i < ox.size(); ++i)
	{
		visualization_msgs::Marker m;  // initialize marker to add to the array
		m.header.stamp = current_time;
		m.header.frame_id = "world";  // relative to the world (fixed) frame
		m.ns = "real";
		m.id = i+1;  // unique id under namespace "real"
		m.type = visualization_msgs::Marker::CYLINDER;
		m.action = visualization_msgs::Marker::ADD;
		// assign x and y positions of the marker as specified by the ros parameter
		m.pose.position.x = ox[i]; 
		m.pose.position.y = oy[i];
		m.pose.position.z = 0;
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = 0;
		m.pose.orientation.w = 1;
		// scale rviz visualization with obs_rad
		m.scale.x = obs_rad;
		m.scale.y = obs_rad;
		m.scale.z = 0.2;
		// red marker, not transparent
		m.color.r = 0.0;
		m.color.b = 0.0;
		m.color.g = 1.0;
		m.color.a = 1.0;
		real_marker_arr.markers.push_back(m);  // add marker to the array
	}
	pub_obs.publish(real_marker_arr);  // publish once MarkerArray
	
	visualization_msgs::Marker sidewall;
	sidewall.header.stamp = current_time;
	sidewall.header.frame_id = "world";
	sidewall.ns = "lines";
	sidewall.id = 1;
	sidewall.type = visualization_msgs::Marker::LINE_STRIP;
	sidewall.action = visualization_msgs::Marker::ADD;
	sidewall.pose.orientation.x = 0.0;
	sidewall.pose.orientation.y = 0.0;
	sidewall.pose.orientation.z = 0.0;
	sidewall.pose.orientation.w = 1.0;
	sidewall.scale.x = 0.05;
	sidewall.color.r = 0.0;
	sidewall.color.g = 0.0;
	sidewall.color.b = 0.0;
	sidewall.color.a = 1.0;

	geometry_msgs::Point point;
	point.x = -wall_width/2;
	point.y = -wall_height/2;
	point.z = 0.0;
	sidewall.points.push_back(point);
	point.x = wall_width/2;
	point.y = -wall_height/2;
	point.z = 0.0;		
	sidewall.points.push_back(point);
	point.x = wall_width/2;
	point.y = wall_height/2;
	point.z = 0.0;
	sidewall.points.push_back(point);
	point.x = -wall_width/2;
	point.y = wall_height/2;
	point.z = 0.0;
	sidewall.points.push_back(point);
	point.x = -wall_width/2;
	point.y = -wall_height/2;
	point.z = 0.0;
	sidewall.points.push_back(point);
	pub_walls.publish(sidewall);
	
	// update robo pos
	Vector2D robot_pos(dd.getX(), dd.getY());
	Transform2D t(robot_pos, dd.getTheta());

	int count = 0;
	int laser_count = 0;
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		// update pose 
		dd.updatePoseWithSlip(dd.getlWheelPhi()+dt*wv.left, dd.getrWheelPhi()+dt*wv.right, slip_unif(get_random()), slip_unif(get_random()));
		// check collision
		for(std::size_t i = 0; i < ox.size(); ++i)
		{	
			Vector2D pos_abs(ox[i], oy[i]);
			Vector2D robot_pos(dd.getX(), dd.getY());
			Vector2D pos_diff = (pos_abs - robot_pos);
			double d_current = magnitude(pos_diff);
			
			if (d_current < dis_crit)
			{
				pos_diff.normalize();		
				Vector2D dp = -(dis_crit - d_current)*pos_diff;
				robot_pos.x += dp.x;
				robot_pos.y += dp.y;
				// new pose 
				WheelPos rp{dd.getTheta(), robot_pos.x, robot_pos.y};
				// set pose
				dd.setPose(rp);
			}
		}
		js.header.stamp = current_time;
		js.position = {dd.getlWheelPhi(), dd.getrWheelPhi()};
		js.velocity = {wv.left, wv.right};
		pub.publish(js);
		

		trans.header.stamp = current_time;
		trans.transform.translation.x = dd.getX();
		trans.transform.translation.y = dd.getY();
		trans.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, dd.getTheta());
		trans.transform.rotation.x = q.x();
		trans.transform.rotation.y = q.y();
		trans.transform.rotation.z = q.z();
		trans.transform.rotation.w = q.w();
		br.sendTransform(trans);	
		

		path.header.stamp = current_time;
		geometry_msgs::PoseStamped pos;
		pos.header.stamp = current_time;
		pos.pose.position.x = dd.getX();
		pos.pose.position.y = dd.getY();
		pos.pose.position.z = 0.0;
		pos.pose.orientation.x = q.x();
		pos.pose.orientation.y = q.y();
		pos.pose.orientation.z = q.z();
		pos.pose.orientation.w = q.w();
		path.poses.push_back(pos);
		pub_path.publish(path);
		
		if (count == 10)
		{
			Vector2D robot_pos(dd.getX(), dd.getY());
			Transform2D t(robot_pos, dd.getTheta());
			Transform2D tinv = t.inv();
			visualization_msgs::MarkerArray relative_marker_arr;
			for(std::size_t i = 0; i < ox.size(); ++i)
			{
				visualization_msgs::Marker m;  
				m.header.stamp = current_time; 
				m.header.frame_id = "odo"; 
				m.ns = "relative";
				m.id = i+1;  
				m.type = visualization_msgs::Marker::CYLINDER;
				
				Vector2D pos_abs(ox[i], oy[i]); 
				std::vector<double> noise_vec = mv_xy.draw();
				Vector2D p_noise((double)noise_vec[0], (double)noise_vec[1]);
				
				Vector2D p_rel = tinv(pos_abs) + p_noise;  
				double mag = magnitude(p_rel);  
				
				if (mag <= num_tubes)
				{
					m.action = visualization_msgs::Marker::MODIFY;
				}
				else
				{
					m.action = visualization_msgs::Marker::DELETE;
				}
				m.pose.position.x = p_rel.x;
				m.pose.position.y = p_rel.y;
				m.pose.position.z = 0;
				m.pose.orientation.x = 0;
				m.pose.orientation.y = 0;
				m.pose.orientation.z = 0;
				m.pose.orientation.w = 1;
				m.scale.x = obs_rad;
				m.scale.y = obs_rad;
				m.scale.z = 0.2;
				m.color.r = 1.0;
				m.color.b = 0.0;
				m.color.g = 0.0;
				m.color.a = 1.0;
				relative_marker_arr.markers.push_back(m);
			}
			pub_sensor.publish(relative_marker_arr); 
			count = 0;
		}
		else
		{
			++count;
		}
		
		if (laser_count == 20) 
		{
			sensor_msgs::LaserScan s_laser;
			s_laser.header.stamp = current_time;
			s_laser.header.frame_id = "odo";
			s_laser.angle_min = 0;
			s_laser.angle_increment = deg2rad(lidar_dtheta);
			s_laser.range_min = lidar_min_range;
			s_laser.range_max = lidar_max_range;
			s_laser.angle_max = deg2rad(lidar_dtheta)*(lidar_n_samples-1);
			for (int i=0; i<lidar_n_samples; ++i)
			{
				double relative_scan_angle = i*s_laser.angle_increment;
				double absolute_scan_angle = normalize_angle(relative_scan_angle + dd.getTheta()); 
				Vector2D p1(dd.getX(), dd.getY()); 
				Vector2D p2; 
				double a = atan((0.5*wall_height-dd.getY())/(0.5*wall_width-dd.getX()));
				double b = atan((0.5*wall_height+dd.getY())/(0.5*wall_width-dd.getX()));
				double c = atan((0.5*wall_height+dd.getY())/(0.5*wall_width+dd.getX()));
				double d = atan((0.5*wall_height-dd.getY())/(0.5*wall_width+dd.getX()));
				if ((absolute_scan_angle >= -b) && (absolute_scan_angle <= a))  // west wall
				{
					p2.x = wall_width/2;
					p2.y = p1.y + tan(absolute_scan_angle)*(p2.x - p1.x);
				}
				else if ((absolute_scan_angle > a) && (absolute_scan_angle < PI-d)) // north wall
				{
					p2.y = wall_height/2;
					p2.x = p1.x + (p2.y - p1.y)/tan(absolute_scan_angle);					
				}
				else if ((absolute_scan_angle >= PI-d) || (absolute_scan_angle <= -PI+c)) // east wall
				{
					p2.x = -wall_width/2;
					p2.y = p1.y + tan(absolute_scan_angle)*(p2.x - p1.x);
				}
				else if ((absolute_scan_angle > -PI+c) && (absolute_scan_angle < -b)) // south wall
				{
					p2.y = -wall_height/2;
					p2.x = p1.x + (p2.y - p1.y)/tan(absolute_scan_angle);
				}
				double range = magnitude(p1-p2);
				Collision it;
				for(std::size_t i = 0; i < ox.size(); ++i)
				{
					Vector2D p_circle(ox[i], oy[i]);
					Collision collTemp = compute_Collision(p1, p2, p_circle, obs_rad);
					if (collTemp.intersected)
					{
						if (!it.intersected)
						{
							Vector2D p1_new(p1.x-collTemp.x, p1.y-collTemp.y);
							range = magnitude(p1_new);
							it = collTemp;
						}
						else
						{
							Vector2D p1_new(p1.x-collTemp.x, p1.y-collTemp.y);
							if (magnitude(p1_new) < range)  
							{
								it = collTemp;
								range = magnitude(p1_new);
							}
						}
					}
				}
				range += laser_gauss(get_random()); 
				int num_resolutions = range/lidar_resolution;  
				range = num_resolutions * lidar_resolution;
				s_laser.ranges.push_back(range);
			}
			pub_laser.publish(s_laser);
			laser_count = 0;
		}
		else
		{
			++laser_count;
		}
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
