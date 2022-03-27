/// \file
/// \brief detect landmarks and publish pos
///
/// PARAMETERS:
/// 	min_cluster_num (int): minimum number of elements in a cluster
/// 	min_angle (double): minimum angle mean for circle (degrees)
///		max_angle (double): maximum angle mean for circle (degrees)
///		angle_std (double): maximum angle standard deviation for circle (radians)
///		cluster_thresh (double): measurement distance threshold for clustering (m)
///		min_circle_rad (double): minimum radius for circle (m)
///		max_circle_rad (double): maximum radius for circle (m)
/// PUBLISHES:
///     detected_circles (visualization_msgs/MarkerArray): circles detected from laser data
/// SUBSCRIBES:
///     scan (sensor_msgs/LaserScan): lidar

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "nuslam/learning.hpp"
#include "turtlelib/rigid2d.hpp"
#include <vector>

static bool is_received = false;
static sensor_msgs::LaserScan scan;

/// \brief Receives lidar data
/// \param msg - a pointer to the sensor_msgs/LaserScan message containing lidar data
void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
	scan = *msg;
	is_received = true;
}

int main(int argc, char** argv)
{
	using namespace nuslam;
	using namespace turtlelib;
	
	ros::init(argc, argv, "landmarks");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber sub = n.subscribe("scan", 10, laserCallback);
	ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("detected_circles", 10);
	
	double cluster_thresh = 0.3;  
	double angle_std = 0.15;
	int min_cluster_num = 3;  
	double min_angle_deg = 90;
	double max_angle_deg = 135;
	double min_circle_rad = 0.03;
	double max_circle_rad = 0.14;
	
	n.getParam("angle_std", angle_std);
	n.getParam("cluster_thresh", cluster_thresh);
	n.getParam("min_cluster_num", min_cluster_num);
	n.getParam("min_angle", min_angle_deg);
	n.getParam("max_angle", max_angle_deg);
	n.getParam("min_circle_rad", min_circle_rad);
	n.getParam("max_circle_rad", max_circle_rad);
	
	double min_angle = deg2rad(min_angle_deg);
	double max_angle = deg2rad(max_angle_deg);
	
	
	std::vector<std::vector<int>> clusters;
	auto current_time = ros::Time::now();
	
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		std::vector<Circle> circles;
		visualization_msgs::MarkerArray circle_markers;
		if (is_received)
		{
			clusters = cluster_range(scan.ranges, cluster_thresh, min_cluster_num);
			for (std::size_t i=0; i<clusters.size(); ++i)
			{
				if (is_circle(scan.ranges, clusters[i], scan.angle_increment, min_angle, max_angle, angle_std))
				{
					std::vector<Vector2D> pts = range2xy(scan.ranges, clusters[i], scan.angle_increment);
					Circle c = fit_circle(pts);
					if ((c.r >= min_circle_rad) && (c.r <= max_circle_rad))
					{
						circles.push_back(c);
					}
				}
			}
			for (std::size_t i=0; i<circles.size(); ++i)
			{
				visualization_msgs::Marker m;
				m.header.stamp = current_time;
				m.header.frame_id = "odo";  //from nusim
				m.ns = "circles";
				m.id = i+1; 
				m.type = visualization_msgs::Marker::CYLINDER;
				m.action = visualization_msgs::Marker::ADD;
				
				m.pose.position.x = circles[i].x; 
				m.pose.position.y = circles[i].y;
				m.pose.position.z = 0;
				m.pose.orientation.x = 0;
				m.pose.orientation.y = 0;
				m.pose.orientation.z = 0;
				m.pose.orientation.w = 1;
				
				m.scale.x = circles[i].r;
				m.scale.y = circles[i].r;
				m.scale.z = 0.2;
				// yellow 
				m.color.r = 1.0;
				m.color.b = 0.0;
				m.color.g = 1.0;
				m.color.a = 1.0;
				circle_markers.markers.push_back(m); 
			}
			pub.publish(circle_markers);
		}
		r.sleep();
	}
	
	return 0;
}
