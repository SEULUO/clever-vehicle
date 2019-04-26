#include <ros/ros.h> 
//#include <serial/serial.h>  //ROS已经内置了的串口包
#include "smart_car/wifidata.h" 
#include "data_deal.h"
//#include <std_msgs/String.h> 
//#include <sstream>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

//#define SERVER_IP "223.3.99.62"
#define PORT 5678
#define maxsize 20
#define recvnum 1
 

//无线网络发送回调函数
/*
void write_callback(const std_msgs::String::ConstPtr& sendmsg)
{
	int i,len;
	ROS_INFO_STREAM("Writing to internet port" << sendmsg->data);
	len = strlen(sendmsg->data);
	for(i=0;i<len;i++)
		sendbuff[i] = send->data[i];
	write(sockfd,sendbuff,len);
}*/

int main (int argc, char** argv) 
{ 	
	//初始化节点,节点名-serial_wifi_node
	ros::init(argc, argv, "wifi_client_node"); 
	//声明节点句柄 
	ros::NodeHandle n; 
	//订阅主题，并配置回调函数 
	//ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback); 
	//发布主题 
	ros::Publisher read_pub = n.advertise<smart_car::wifidata>("frontspeed", 1000);
	int recvlen;    //接收字符数组长度
	u8 recvbuff[maxsize];   //接收字符缓存区
	float recvdata[recvnum];   //接收数组
	int sockfd;
	struct sockaddr_in server_addr; //定义服务器端套接口数据结构
	/*****************客户端开始建立socket描述符******************/
 	if((sockfd = socket(AF_INET,SOCK_STREAM,0)) < 0)
  	{
		printf("Socket created error!");
		exit(1);
  	}
	else     //创建成功
	{
     	printf("Socket created successfully!\n");
		printf("Socket id:%d\n",sockfd);
	}
	/*****************客户端填充服务器sockaddr结构***************/
	bzero(&server_addr,sizeof(struct sockaddr_in));   //清空表示地址的结构体变量
	server_addr.sin_family = AF_INET;  //设置server_addr的成员信息
	server_addr.sin_port = htons(PORT);   //端口号设置
	server_addr.sin_addr.s_addr = inet_addr(argv[1]);  //IP地址设置
	/********************客户端发起连接请求********************/
	if(connect(sockfd,(struct sockaddr*)(&server_addr),sizeof(sockaddr)) < 0)
	{
		printf("Connect error!\n");
		exit(1);
	}
	else   //连接成功
	{
		printf("Connect successfully!\n");
	} 
	smart_car::wifidata data;
	while(ros::ok())
	{
		if((recvlen = read(sockfd,recvbuff,maxsize)) < 0)
		{
			printf("Read error!\n");
			exit(1);
		}
		recvbuff[recvlen] = '\0';
		//ROS_INFO("I have received: %s\n",recvbuff);
		recvfloat(recvbuff,recvdata,recvnum);  //接收字符处理函数
		data.front_speed = recvdata[0];
		ROS_INFO("The speed of the front car is: %f",data.front_speed);
		read_pub.publish(data);
		ros::spinOnce();	
	}
	close(sockfd);
	return 0;
} 
