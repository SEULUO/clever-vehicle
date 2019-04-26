#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <cstring>
#include <dynamic_reconfigure/server.h>
#include <smart_car/stm_controlConfig.h>
#include "smart_car/stmsend.h"  //上位机发送控制信号
#include "smart_car/stmrecv.h"  //下位机反馈位姿数据
#include "data_deal.h"    //数据处理头文件

#define sendnum 2                //上位机发送给stm32的数据容量
#define sendlen (sendnum+1)*4
#define recvnum 2               //stm32发送给上位机的数据容量
#define recvlen (recvnum+1)*4

serial::Serial ser; //声明串口对象
//定义接收，发送数据
float senddata[sendnum];
u8 sendbuff[sendlen];
float recvdata[recvnum];
u8 recvbuff[recvlen];

void setCallback(const smart_car::stmsend::ConstPtr& output)
{
	//数组里按顺序分别为左侧设定轮速，右侧设定轮速
	senddata[0] = output->speed_cmd;
	senddata[1] = output->steer_cmd;
	sendfloat(sendbuff,senddata,sendnum);   //发送数据处理
	ser.write(sendbuff,sendlen);   //发送串口数据
}

//回调函数将输出参数的新值，参数名称必须与cfg文件里相同
//包名，cfg文件名+Config
void callback(smart_car::stm_controlConfig &config, uint32_t level)
{
	
	//ROS_INFO("Reconfigure Request: %f %f",config.left_speed,config.right_speed);
	//senddata[0] = config.left_speed;
	//senddata[1] = config.right_speed;
	ROS_INFO("Reconfigure Request: %f %f",config.wheel_speed,config.ideal_steer);
	senddata[0] = config.wheel_speed;
	senddata[1] = config.ideal_steer;
	sendfloat(sendbuff,senddata,sendnum);   //发送数据处理
	ser.write(sendbuff,sendlen);   //发送串口数据
}

int main (int argc, char** argv) 
{ 
	
	//初始化节点,节点名-serial_wifi_send
	ros::init(argc, argv, "stm_control"); 
	
	//初始化服务器，忽略smart_carConfig配置文件
	dynamic_reconfigure::Server<smart_car::stm_controlConfig> server;
	//向服务器发送回调函数。当服务器得到重新配置请求，它会调用回调函数
	dynamic_reconfigure::Server<smart_car::stm_controlConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	
	//声明节点句柄 
	ros::NodeHandle n; 
	//订阅stm32主题，并配置回调函数,该消息为上位机传给底层的控制数据 
	ros::Subscriber stm_sub = n.subscribe("control_order", 10, setCallback); 
	//发布主题 
	ros::Publisher stm_pub = n.advertise<smart_car::stmrecv>("kinestate", 10); 
	
	//数据初始化
	int i;	
	senddata[0] = 0.0;
	senddata[1] = 0.0;
	
	try 
	{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/stm32_103"); 
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
	ros::Rate loop_rate(10);
	
	while(ros::ok()) 
    { 
        if(ser.available())
        { 
        	//ROS_INFO_STREAM("Reading from serial port\n"); 
        	std_msgs::String result; 
           	result.data = ser.readline();
           	for(i=0;i<recvlen;i++)
           	{
           		recvbuff[i] = result.data[i];
           		printf("%02X ",recvbuff[i]);
			}
            recvfloat(recvbuff,recvdata,recvnum);   //接收数据处理函数 
            //数据保留一位小数
        	ROS_INFO("Vr: %.1fm/s theta: %.1fdu",recvdata[0],recvdata[1]);
        } 
        smart_car::stmrecv rdata;
        rdata.speed_mea = recvdata[0];
        rdata.steer_mea = recvdata[1];
        stm_pub.publish(rdata);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }  
	return 0;
} 
