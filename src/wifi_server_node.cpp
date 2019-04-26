#include <ros/ros.h> 
//#include <serial/serial.h>  //ROS已经内置了的串口包 
//#include <std_msgs/String.h> 
//#include <sstream>
#include "smart_car/wifidata.h"
#include "data_deal.h"
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 5678
#define maxsize 20
#define sendnum 1
u8 sendbuff[maxsize];
float sendata[sendnum];
int sockfd,newsockfd;        //定义套接口描述符

//无线网络发送回调函数
void send_callback(const smart_car::wifidata::ConstPtr& sendmsg)
{
	//int i,len;
	//ROS_INFO_STREAM("Writing to internet port" << sendmsg->data);
	//len = strlen(sendmsg->data.c_str());
	//strcpy(sendbuff,sendmsg->data.c_str());
	ROS_INFO("Send data is: %f",sendmsg->front_speed);
	sendata[0] = sendmsg->front_speed;
	sendfloat(sendbuff,sendata,sendnum);  //发送数据处理函数
	if(write(newsockfd,sendbuff,maxsize) < 0)
		printf("Write error!\n");
}

int main (int argc, char** argv) 
{ 	
	//初始化节点,节点名-serial_wifi_node
	ros::init(argc, argv, "wifi_server_node"); 
	//声明节点句柄 
	ros::NodeHandle n; 
	//订阅主题，并配置回调函数 
	ros::Subscriber write_sub = n.subscribe("wifisend", 1000, send_callback); 
	//发布主题 
	//ros::Publisher read_pub = n.advertise<serial_node::serial_receive>("receive", 1000);
	/*****************定义所用到的数据******************/     
	struct sockaddr_in server_addr;   //定义服务器端套接口地址数据结构
	struct sockaddr_in client_addr;   //定义客户端套接口数据结构
 	socklen_t sin_size;
	/*****************服务器端开始建立socket描述符******************/
 	if((sockfd = socket(AF_INET,SOCK_STREAM,0)) < 0)
  	{
		perror("Socket created error!\n");
		exit(1);
  	}
	else     //创建成功
	{
     	printf("Socket created successfully!\n");
		printf("Socket id:%d\n",sockfd);
	}
	/*****************服务器端填充sockaddr结构***************/
	bzero(&server_addr,sizeof(struct sockaddr_in));   //清空表示地址的结构体变量
	server_addr.sin_family = AF_INET;  //设置server_addr的成员信息
	server_addr.sin_port = htons(PORT);
	//server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);  //IP地址设置
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //本机IP地址
	/******************调用bind函数绑定端口******************/
	if(bind(sockfd,(struct sockaddr*)(&server_addr),sizeof(struct sockaddr)) < 0)
	{
		printf("Bind error!\n");
		exit(1);
	}
	else    //端口绑定成功
	{
		printf("Bind port successfully!\n");
	}
	/******调用listen函数监听端口号，能同时处理的最大连接请求数为5*******/
	if(listen(sockfd,5) < 0)
	{
		perror("Listen error!\n");
		exit(1);
	}
	/*******服务器阻塞，等待接收连接请求，直到客户程序发连接请求**********/
	printf("Waiting for the client to connect......\n");
	sin_size = sizeof(struct sockaddr_in);
	if((newsockfd = accept(sockfd,(struct sockaddr*)(&client_addr),&sin_size)) < 0)
	{
		printf("Accept error!\n");
		exit(1);
	}
	printf("Server get  connection from %s\n",inet_ntoa(client_addr.sin_addr));
	printf("Connected sucessful!:\n");
	
	ros::spin();
	close(newsockfd);	
	close(sockfd);
	return 0;
} 
