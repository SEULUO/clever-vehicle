#include "ros/ros.h"
//#include "smart_car/gpsdata.h"
//#include "smart_car/stmsend.h"
//#include "std_msgs/String.h"
//#include <sstream>
#include "smart_car/wifidata.h"

int main(int argc,char **argv)
{
	//初始化节点 
	ros::init(argc, argv, "testdata"); 
	//声明节点句柄 
	ros::NodeHandle n; 
	//发布主题,smart_car-包名,gpsdata-数据类型
	//ros::Publisher write_pub = n.advertise<std_msgs::String>("write", 1000);
	//ros::Publisher stm_pub = n.advertise<smart_car::stmsend>("control_order", 1000);
	ros::Publisher wifi_pub = n.advertise<smart_car::wifidata>("wifisend", 1000);
	//指定循环的频率
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok())
	{
		//编制消息
		/*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "test data: " << count;
		msg.data = ss.str();
		write_pub.publish(msg);*/
		
		smart_car::wifidata msg;
		msg.front_speed = 0.5 + count;
		wifi_pub.publish(msg);
		//循环等待回调函数
		ros::spinOnce();
		//按照循环频率延时
		loop_rate.sleep();
		++count;
	}
	return 0;
}
