#include <ros/ros.h> 
#include <std_msgs/String.h>
#include "sensor_msgs/LaserScan.h"
#include "smart_car/stmsend.h"
#include "gps_deal.h"
#define L 0.6

float left_dis = 0;
float right_dis = 0;

/*
//前车GPS数据接收处理函数
void FrontCallback(const smart_car::Agpsdata::ConstPtr& front_gps)
{
	//数组里按顺序分别为lat,lon,yaw,speed
	front_lat = front_gps->lat;
	front_lon = front_gps->lon;
	front_yaw = front_gps->yaw;
	front_speed = front_gps->speed;
}
//目标车GPS数据接收处理函数
void TargpsCallback(const smart_car::gpsdata::ConstPtr& tar_gps)
{
	//数组里按顺序分别为lat,lon,yaw,speed
	tar_lat = tar_gps->lat;
	tar_lon = tar_gps->lon;
	tar_yaw = tar_gps->yaw;
	tar_speed = tar_gps->speed;
}
*/

float cal_steer(float left, float right)
{
	float third,dijiao,ld,ld1,theta,theta1,r;
	third = sqrt(pow(left,2) + pow(right,2));
	ld = 0.5*third;
	dijiao = atan(right_dis/left_dis);
	theta = dijiao - 0.25*M_PI;   //负为左转，正为右转
	if(theta < 0)
		ld1 = sqrt(pow(ld,2) + pow(L,2) + 2*ld*L*cos(-theta));
	else
		ld1 = sqrt(pow(ld,2) + pow(L,2) + 2*ld*L*cos(theta));
	theta1 = asin(ld*sin(theta)/ld1);
	r = ld1/(2*sin(theta1));  //计算转弯半径
	return RAD2DEG(atan(L/r));    //结果转为角度制
}
//雷达数据处理函数
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float degree;
	int i,j;
    int count = scan->scan_time / scan->time_increment;
    //printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    //printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  	
  	j = 0;
    for(i = 85; i < 95; i++) 
    {
        degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);  
        if(scan->ranges[i]!=0)
        {
        	left_dis += scan->ranges[i];
        	j++;
		}
        //printf("angle-distance : [%f, %f]\n", degree, scan->ranges[i]); 
    }
    if(j!=0)
    	left_dis = left_dis/j;
    else
    	left_dis = 8.0;
    	
    j = 0;
    for(i = 625; i < 635; i++)
    {
    	degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    	//printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    	if(scan->ranges[i]!=0)
        {
        	right_dis += scan->ranges[i];
        	j++;
		}
        //printf("angle-distance : [%f, %f]\n", degree, scan->ranges[i]); 
	}
	if(j!=0)
		right_dis = right_dis/j;  
    else
    	right_dis = 8.0; 	
    printf("mean distance: [%.2f, %.2f]\n",left_dis,right_dis);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_control");  //启动该节点并设置其名称
	ros::NodeHandle n;
	//订阅主题，并配置回调函数, "Targps_data"--消息名称，TargpsCallback--回调函数
	//ros::Subscriber f_sub = n.subscribe("FGPS_data", 1000, FrontCallback);
 	//ros::Subscriber sub = n.subscribe("Targps_data", 1000, TargpsCallback);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
	//将节点设置成发布者，并将所发布主题和类型的名称告知节点管理器
	ros::Publisher pub = n.advertise<smart_car::stmsend>("control_order", 1000);
	//发送数据的频率为10Hz
	ros::Rate loop_rate(10);
	smart_car::stmsend output;
	
	while(ros::ok())
	{
		output.wheel_speed = 300;
		output.ideal_steer = 10*cal_steer(left_dis,right_dis)/3;
		printf("ideal_steer: %.1f\n",output.ideal_steer);
		pub.publish(output);
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
