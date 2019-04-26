#include <ros/ros.h> 
#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h>
#include "advanced_navigation_driver/gpsdata.h"

//double x = 0.0; 
//double y = 0.0;
/*
void GpsCallback(const advanced_navigation_driver::gpsdata& gps)
{
	char zone;
	LLtoUTM(gps->latitude, gps->longitude, y, x, &zone);	
}
*/

int main(int argc, char **argv)
{
	ros::init (argc, argv, "showpath"); 
	ros::NodeHandle ph; 
	
	ros::Publisher path_puba = ph.advertise<nav_msgs::Path>("trajectory_a",1, true);
	ros::Publisher path_pubb = ph.advertise<nav_msgs::Path>("trajectory_b",1, true);
	
	ros::Time current_time;
	ros::Time last_time; 
	current_time = ros::Time::now(); 
	last_time = ros::Time::now(); 
	
	nav_msgs::Path path; 
	path.header.stamp = current_time; 
	path.header.frame_id = "odom"; 
	
	double x_a = 0.0; 
	double y_a = 0.0; 
	double th_a = 0.0; 
	double vx_a = 0.1; 
	double vy_a = -0.1; 
	double vth_a = 0.1;
	
	double x_b = 0.0; 
	double y_b = 0.0; 
	double th_b = 0.0; 
	double vx_b = 0.1; 
	double vy_b = -0.1; 
	double vth_b = 0.1;
	
	ros::Rate loop_rate(1);
	while (ros::ok()) 
	{ 
		current_time = ros::Time::now(); 
		//compute odometry in a typical way given the velocities of the robot
		
		double dt = (current_time - last_time).toSec(); 
		double delta_xa = (vx_a * cos(th_a) - vy_a * sin(th_a)) * dt; 
		double delta_ya = (vx_a * sin(th_a) + vy_a * cos(th_a)) * dt; 
		double delta_tha = vth_a * dt; 
		x_a += delta_xa; 
		y_a += delta_ya; 
		th_a += delta_tha; 
		
		//将x,y坐标传入this_pose_stamped数据结构
		geometry_msgs::PoseStamped this_pose_stamped; 
		this_pose_stamped.pose.position.x = x_a; 
		this_pose_stamped.pose.position.y = y_a;
		
		//根据航向计算四元数
		geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th_a);
		this_pose_stamped.pose.orientation.x = goal_quat.x; 
		this_pose_stamped.pose.orientation.y = goal_quat.y; 
		this_pose_stamped.pose.orientation.z = goal_quat.z; 
		this_pose_stamped.pose.orientation.w = goal_quat.w;
		
		this_pose_stamped.header.stamp = current_time; 
		this_pose_stamped.header.frame_id = "odom"; 
		path.poses.push_back(this_pose_stamped); 
		path_puba.publish(path);
		
		double delta_xb = (vx_b * cos(th_b) - vy_b * sin(th_b)) * dt; 
		double delta_yb = (vx_b * sin(th_b) + vy_b * cos(th_b)) * dt; 
		double delta_thb = vth_b * dt; 
		x_b += delta_xb; 
		y_b += delta_yb; 
		th_b += delta_thb; 
		
		this_pose_stamped.pose.position.x = x_b; 
		this_pose_stamped.pose.position.y = y_b;
		
		goal_quat = tf::createQuaternionMsgFromYaw(th_b);
		this_pose_stamped.pose.orientation.x = goal_quat.x; 
		this_pose_stamped.pose.orientation.y = goal_quat.y; 
		this_pose_stamped.pose.orientation.z = goal_quat.z; 
		this_pose_stamped.pose.orientation.w = goal_quat.w;
		
		path.poses.push_back(this_pose_stamped); 
		path_pubb.publish(path);
		
		ros::spinOnce();               // check for incoming messages
		last_time = current_time; 
		loop_rate.sleep(); 
	} 
	return 0;
}

