#include <ros/ros.h> 
#include <serial/serial.h>   
#include <std_msgs/String.h>
#include <cstring>
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define maxsize 60
#define imunum 7 
serial::Serial ser; //声明串口对象

//返回值为拆分之后数组的长度
//databuff——字符串数据
//def——拆分字符串，本例中为“, ”
//data——拆分后的数据数组
int split(char *databuff, char *def, double data[])
{
	int i = 0;
	char *temp = strtok(databuff,def);
	while(temp)
	{
		data[i] = atof(temp);
		temp = strtok(NULL,def);
		i++;
	}
	return i;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"razor_imu_node");  //启动该节点并设置其名称	
	ros::NodeHandle n;

	//订阅主题，并配置回调函数 
	//ros::Subscriber f_sub = n.subscribe("FGPS_data", 1000, FrontCallback);
 	//ros::Subscriber r_sub = n.subscribe("RGPS_data", 1000, RearCallback);
 	
	//将节点设置成发布者，并将所发布主题和类型的名称告知节点管理器
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	
	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0.0;

	// velocity
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	
	//accelation
	double ax = 0.0;
	double ay = 0.0;
	double dt,delta_x,delta_y,delta_th;
	//接收数据变量定义
	std_msgs::String receive; 
	char recvbuff[maxsize];
	double imudata[imunum];
	int i,length;  //length--接收字符串长度
	
	try 
	{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyACM0"); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to); 
		ser.open(); 
	 }
	catch (serial::IOException& e) 
	{ 
		ROS_ERROR_STREAM("Unable to open port "); 
		return -1; 
	} 
	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()) 
	{ 
		ROS_INFO_STREAM("Serial Port initialized"); 
	} 
	else 
	{ 
		return -1; 
	}
	
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
		if(ser.available())
		{
			//从串口中读取数据，格式为<timeMS>,<accelX>,<accelY>,<accelZ>,<gyroX>,<gyroY>,<gyroZ>
			receive.data = ser.readline();
			ROS_INFO_STREAM("Read: "<< receive.data);
			strcpy(recvbuff,receive.data.c_str());
			length = strlen(receive.data.c_str());
			recvbuff[length] = '\0';
			if(split(recvbuff,", ",imudata) != imunum)
				ROS_INFO("Data error!");
			//数据滤波
			if(fabs(imudata[1])<=0.03)
				imudata[1] = 0.0;
			if(fabs(imudata[2])<=0.01)
				imudata[2] = 0.0;
			if(fabs(imudata[6])<=0.12)
				imudata[6] = 0.0;
				/*
			else
			{
				for(i=0;i<imunum;i++)
					printf("%.2f ",imudata[i]);
				printf("\n");
			}*/
		}
		
		current_time = ros::Time::now(); 
		ax = imudata[1];
		ay = imudata[2];
		vth = imudata[6];
		dt = (current_time - last_time).toSec();
		vx += ax*dt;
		vy += ay*dt;
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
		odom.twist.twist.angular.x = imudata[4];
		odom.twist.twist.angular.y = imudata[5];
		odom.twist.twist.angular.z = imudata[6];

		last_time = current_time;
		
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
