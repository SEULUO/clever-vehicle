#include <ros/ros.h> 
#include "smart_car/stmrecv.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define DEG2RAD(x) ((x)*M_PI/180)
#define L 0.6

double vx,vy,vth,th=0.0;

void stateCallback(const smart_car::stmrecv::ConstPtr& msg)
{
	//小车运动微分方程
	vx = msg->speed_mea*cos(th);
	vy = msg->speed_mea*sin(th);
	vth = msg->speed_mea*tan(DEG2RAD(msg->steer_mea))/L;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"odometry_node");  //启动该节点并设置其名称	
	ros::NodeHandle n;

	//订阅主题，并配置回调函数
 	ros::Subscriber odom_sub = n.subscribe("kinestate", 10, stateCallback);
 	
	//将节点设置成发布者，并将所发布主题和类型的名称告知节点管理器
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	
	// initial position
	double x = 0.0; 
	double y = 0.0;
	double dt,delta_x,delta_y,delta_th;
	
	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(10); //发送数据的频率为10Hz
	
	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	
	while(ros::ok())    //当收到停止消息或者ROS停止当前节点运行时,ros::ok()行会执行停止节点运行的命令
	{
		current_time = ros::Time::now(); 
		dt = (current_time - last_time).toSec();
		delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		last_time = current_time;
		
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
