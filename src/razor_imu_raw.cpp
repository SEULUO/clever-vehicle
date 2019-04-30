#include <ros/ros.h> 
#include <serial/serial.h>   
#include <std_msgs/String.h>
#include <cstring>
#include "stdlib.h"
#include "stdio.h"
//#include "math.h"
//#include <sensor_msgs/JointState.h>
//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define maxsize 100
#define imunum 11 
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
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("RawImu", 10);
	
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
	
	ros::Rate loop_rate(10); //发送数据的频率为10Hz
	
	while(ros::ok())    //当收到停止消息或者ROS停止当前节点运行时,ros::ok()行会执行停止节点运行的命令
	{
		if(ser.available())
		{
			//从串口中读取数据，格式为<timeMS>,<accelX>,<accelY>,<accelZ>,<gyroX>,<gyroY>,<gyroZ>,<qw>,<qx>,<qy>,<qz>
			receive.data = ser.readline();
			ROS_INFO_STREAM("Read: "<< receive.data);
			strcpy(recvbuff,receive.data.c_str());
			length = strlen(receive.data.c_str());
			recvbuff[length] = '\0';
			if(split(recvbuff,", ",imudata) != imunum)
				ROS_INFO("Data error!");
			/*
			else
			{
				for(i=0;i<imunum;i++)
					printf("%.2f ",imudata[i]);
				printf("\n");
			}*/
		}
		sensor_msgs::Imu imu_data;
		imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        //线加速度
        imu_data.linear_acceleration.x = imudata[1]; 
        imu_data.linear_acceleration.y = imudata[2];
        imu_data.linear_acceleration.z = imudata[3];
	    //角速度
        imu_data.angular_velocity.x = imudata[4]; 
        imu_data.angular_velocity.y = imudata[5]; 
        imu_data.angular_velocity.z = imudata[6];
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        imu_data.orientation.x = imudata[7];
        imu_data.orientation.y = imudata[8];
        imu_data.orientation.z = imudata[9];
       	imu_data.orientation.w = imudata[10];
        
		imu_pub.publish(imu_data);
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
