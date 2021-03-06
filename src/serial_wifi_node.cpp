#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <smart_car/stm_controlConfig.h>
#include <std_msgs/Int8.h> 
//#include "smart_car/gpsdata.h"  //包名，msg文件名
//#include "data_deal.h"    //数据处理头文件
serial::Serial ser; //声明串口对象

//回调函数将输出参数的新值，参数名称必须与cfg文件里相同
//包名，cfg文件名+Config
void callback(smart_car::stm_controlConfig &config, uint32_t level)
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "test data: " << config.send_number;
	msg.data = ss.str();
	ROS_INFO("%s",msg.data.c_str());  //显示数据
	ser.write(msg.data);   //发送串口数据
}
//无线串口发送回调函数
void write_callback(const std_msgs::String::ConstPtr& sendmsg)
{
	ROS_INFO_STREAM("Writing to serial port"<<sendmsg->data);
	ser.write(sendmsg->data);
}

int main (int argc, char** argv) 
{ 	
	//初始化节点,节点名-serial_wifi_node
	ros::init(argc, argv, "serial_wifi_node"); 
	
	//初始化服务器，忽略smart_carConfig配置文件
	dynamic_reconfigure::Server<smart_car::stm_controlConfig> server;
	//向服务器发送回调函数。当服务器得到重新配置请求，它会调用回调函数
	dynamic_reconfigure::Server<smart_car::stm_controlConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	//声明节点句柄 
	ros::NodeHandle n; 
	//订阅主题，并配置回调函数 
	ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback); 
	//发布主题 
	//ros::Publisher read_pub = n.advertise<serial_node::serial_receive>("receive", 1000);

	std_msgs::String receive; 
	
	try 
	{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyUSB0"); 
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
	//指定循环的频率
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		if(ser.available())
		{
			//ROS_INFO_STREAM("Reading from serial port\n");
			receive.data = ser.read(ser.available());
			ROS_INFO_STREAM("Read: "<< receive.data);
		}
		//循环等待回调函数
		ros::spinOnce();
		//按照循环频率延时
		loop_rate.sleep();
	}
	 
	return 0;
} 
