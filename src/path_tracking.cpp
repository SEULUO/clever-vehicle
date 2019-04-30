#include <ros/ros.h> 
#include <std_msgs/String.h>
#include "sensor_msgs/LaserScan.h"
#include "smart_car/stmsend.h"
#include "advanced_navigation_driver/gpsdata.h"
#include "gps_deal.h"
#define L 0.6
#define pointNum 4
//路径经纬度
double Point_Lat[pointNum]={31.888163889,31.887802778,31.887425000,31.887411111 };
double Point_Lon[pointNum]={118.810083333,118.810102778,118.810133333,118.810344444};
double Lat = 0.0;
double Lon = 0.0;
float Yaw = 0.0;
//雷达全局变量
//bool laser_perm = false;

//前车GPS数据接收处理函数
void Gps_callback(const advanced_navigation_driver::gpsdata::ConstPtr& real_gps)
{
	//数组里按顺序分别为lat,lon,yaw
	if(real_gps->yaw!=0){
		Lat = real_gps->latitude;
		Lon = real_gps->longitude;
		Yaw = real_gps->yaw;
	}
}

//雷达数据处理函数
/*
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float x,y,deg,degree;
	int count = scan->scan_time / scan->time_increment;
	int bcount = 0;
	for(int i = 0; i < count; i++) 
	{
		deg = scan->angle_min + scan->angle_increment * i;
		degree = RAD2DEG(deg);
		x = scan->ranges[i] * sin(deg);
		y = scan->ranges[i] * cos(deg);
	 	//雷达滤波范围
		if(x>=-0.35&&x<=0.35&&y>=-1.5&&y<0)
		{
			//printf("angle-situation:[%f, %f, %f, %f]\n", x,y,degree,scan->ranges[i]);
			bcount++;	
		}
	}
	if(bcount==0)
		laser_perm = true;
	else
		laser_perm = false;	
}
*/
int main(int argc, char** argv)
{
	ros::init(argc,argv,"path_tracking");  //启动该节点并设置其名称
	ros::NodeHandle n;
	//订阅主题，并配置回调函数, "Targps_data"--消息名称，TargpsCallback--回调函数
	ros::Subscriber gps_sub = n.subscribe<advanced_navigation_driver::gpsdata>("AnppGpsdata", 10, Gps_callback);
	//ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 10, scanCallback);
	//将节点设置成发布者，并将所发布主题和类型的名称告知节点管理器
	ros::Publisher pub = n.advertise<smart_car::stmsend>("control_order", 10);
	//发送数据的频率为10Hz
	ros::Rate loop_rate(5);
	
	int i=0;
	float bear = 0;
	float yawerr = 0;
	float str = 0;
	double dis = 0;
	
	smart_car::stmsend output;
	
	while(ros::ok())
	{
		dis = distance(Lat,Lon,Point_Lat[i],Point_Lon[i]);
		if(dis<=1.0)
		{
			i++;
		}
		if(i==pointNum)
		{
			output.wheel_speed = 0;
			i=0;
		}
		else
			output.wheel_speed = 200;
		
		ROS_INFO("Target Point: %d, lat: %lf, lon: %lf\n",i,Point_Lat[i],Point_Lon[i]);
		bear = bearing(Lat,Lon,Point_Lat[i],Point_Lon[i]);
		yawerr = bear-DEG2RAD(Yaw);
		str = atan(L*sin(yawerr)/dis);
		ROS_INFO("Distance: %f Yaw: %f Bearing: %f\n",dis,Yaw,RAD2DEG(bear));
		ROS_INFO("Yawerr: %f Steer: %f\n",RAD2DEG(yawerr),RAD2DEG(str));
		
		output.ideal_steer = 10*RAD2DEG(str)/3;
		//ROS_INFO("control_order: %d, %f\n",output.wheel_speed,output.ideal_steer);
		pub.publish(output);
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
