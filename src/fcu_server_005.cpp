#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/InertiaStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "fcu_bridge.h"
#include "../mavlink/common/mavlink.h"
using namespace std;
//机载电脑运行此节点用于连接远程电脑，机载电脑基于网口与远程电脑双向通信
#define BUF_SIZE 32768//数据缓存区大小
static int PORT = 10005;// 定义端口号
static mavlink_channel_t mav_chan=MAVLINK_COMM_1;//MAVLINK_COMM_0虚拟串口发送，MAVLINK_COMM_1网口发送
static float pos_odom_001_x=0.0f; float pos_odom_001_y=0.0f; float pos_odom_001_z=0.0f;
static float pos_odom_001_roll=0.0f; float pos_odom_001_pitch=0.0f; float pos_odom_001_yaw=0.0f;
static int channel;
static int socket_cli;
static int get_drone=0;
static bool get_heartbeat=false;
struct sockaddr_in drone_addr;
static serial::Serial ser; //声明串口对象
static mavlink_system_t mavlink_system;
static mavlink_message_t msg_received;
static mavlink_status_t status;
static mavlink_scaled_imu_t imu;
static mavlink_global_vision_position_estimate_t pose;
static mavlink_global_position_int_t position;
static mavlink_battery_status_t batt;
static mavlink_heartbeat_t heartbeat;
static mavlink_command_long_t cmd_long;
static mavlink_set_mode_t setmode;
static mavlink_attitude_quaternion_t attitude_quaternion;
static mavlink_set_position_target_local_ned_t set_position_target;
static mavlink_set_gps_global_origin_t gps_global_origin,set_gps_global_origin;
static mavlink_message_t msg_gnss_origin;
static uint8_t buffer[BUF_SIZE];
static uint8_t TxBuffer[BUF_SIZE];
static uint8_t RxBuffer[BUF_SIZE];
static uint8_t TxBuffer_buf[BUF_SIZE];
static uint8_t TxBuffer_cmd[BUF_SIZE];
static uint8_t TxBuffer_cmd_buf[BUF_SIZE];
static double time_start=0.0f;
static double time_odom=0.0f;
static double time_heartbeat=0.0f;
static bool armed=false;
static bool get_gnss_origin=false;

ros::Publisher gnss_origin;
ros::Publisher imu_global;
ros::Publisher odom_global;
ros::Subscriber odom;
ros::Subscriber gnss_001;
ros::Subscriber cmd;
ros::Subscriber mission;
ros::Subscriber motion;
ros::Publisher path_global;
ros::Publisher goal;
ros::Publisher command;
ros::Subscriber odom001;
ros::Subscriber battery;
ros::Publisher mission_pub_001;
std_msgs::Float32MultiArray mission_001;
std_msgs::Int16 cmd_pub;
sensor_msgs::NavSatFix gnss_pub,gnss_origin_pub;
sensor_msgs::Imu imu_pub;
nav_msgs::Odometry odom_pub;
nav_msgs::Path path_pub;
geometry_msgs::PoseStamped odomPose;
geometry_msgs::PoseStamped goal_pub;
geometry_msgs::PoseStamped odom_target;
nav_msgs::Path path_target;
ros::Publisher path_target_pub;
tf::TransformBroadcaster *br = nullptr;
RingBuffer mav_buf_send, mav_buf_cmd;
RingBuffer mav_buf_receive;

void flush_data(void){
	uint16_t length=rbGetCount(&mav_buf_send);
	if(length>0){
  	  for(uint16_t i=0; i<length; i++){
  		  TxBuffer_buf[i]=rbPop(&mav_buf_send);
  	  }
      send(socket_cli, TxBuffer_buf, length, 0);
	}
}

void mav_send_buffer(mavlink_channel_t chan, char *buf, uint16_t len){
    for(uint16_t i=0; i<len; i++){
        rbPush(&mav_buf_send, buf[i]);
    }  
}

void mavlink_send_msg(mavlink_channel_t chan, mavlink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	// XXX use the right sequence here

	mav_send_buffer(chan, (char *)&msg->magic, MAVLINK_NUM_HEADER_BYTES);
	mav_send_buffer(chan, (char *)&msg->payload64, msg->len);
	mav_send_buffer(chan, (char *)ck, 2);
}

//心跳
void mav_send_heartbeat(void){
  mavlink_message_t msg_heartbeat;
  mavlink_heartbeat_t heartbeat;
  heartbeat.base_mode=0;
  if(armed){
    heartbeat.base_mode|=MAV_MODE_FLAG_SAFETY_ARMED;
  }
  mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &msg_heartbeat, &heartbeat);
  mavlink_send_msg(mav_chan, &msg_heartbeat);
}

void parse_data(void){
	int chan = 0;
    uint16_t n=rbGetCount(&mav_buf_receive);
	if(n){
		// printf("Reading from serial port:%d\n",n);
		for(int i=0; i<n; i++){
			if (mavlink_parse_char(chan,rbPop(&mav_buf_receive), &msg_received, &status)){
                //printf("Received \n");
                switch (msg_received.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        mavlink_msg_heartbeat_decode(&msg_received, &heartbeat);
                        time_heartbeat=ros::Time::now().toSec();
                        get_heartbeat=true;
                        break;
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                        mavlink_msg_set_position_target_local_ned_decode(&msg_received, &set_position_target);
                        //发布mission
                        mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
                        mission_001.layout.dim[0].label = "mission_001";
                        mission_001.layout.dim[0].size = 11;
                        mission_001.layout.dim[0].stride = 1;
                        mission_001.data.resize(11);
                        mission_001.data[0]=set_position_target.yaw;//rad
                        mission_001.data[1]=set_position_target.yaw_rate;//rad/s
                        mission_001.data[2]=set_position_target.x;//x
                        mission_001.data[3]=set_position_target.y;//y
                        mission_001.data[4]=set_position_target.z;//z
                        mission_001.data[5]=set_position_target.vx;//vx
                        mission_001.data[6]=set_position_target.vy;//vy
                        mission_001.data[7]=set_position_target.vz;//vz
                        mission_001.data[8]=set_position_target.afx;//ax
                        mission_001.data[9]=set_position_target.afy;//ay
                        mission_001.data[10]=set_position_target.afz;//az
                        mission_pub_001.publish(mission_001);
                        cmd_pub.data=102;
                        command.publish(cmd_pub);
                        break;
                    default:
                        // printf("msgid: %d\n", msg_received.msgid);
                        break;
                }
			}
		}
	}
}

void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){   
    case 2001:
        armed=false;
        break;
    case 2002:
        armed=true;
        break;
    case 2003:
        gps_global_origin.latitude=set_gps_global_origin.latitude;
        gps_global_origin.longitude=set_gps_global_origin.longitude;
        gps_global_origin.altitude=set_gps_global_origin.altitude;
        mavlink_msg_set_gps_global_origin_encode(mavlink_system.sysid, mavlink_system.compid, &msg_gnss_origin, &gps_global_origin);
        mavlink_send_msg(mav_chan, &msg_gnss_origin);
        break;
    default:
        break;
  }
}

void gnssHandler(const sensor_msgs::NavSatFix::ConstPtr& gnss){
    if(get_heartbeat){
        mavlink_global_position_int_t global_position_int;
        mavlink_message_t msg_global_position_int;
        global_position_int.lat=gnss->latitude*1e7;
        global_position_int.lon=gnss->longitude*1e7;
        global_position_int.alt=gnss->altitude*1e3;//m->mm
        global_position_int.hdg=gnss->status.service;
        mavlink_msg_global_position_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_position_int, &global_position_int);
        mavlink_send_msg(mav_chan, &msg_global_position_int);
    }
}

void battHandler(const sensor_msgs::BatteryState::ConstPtr& batt){
    if(get_heartbeat){
        mavlink_battery_status_t battery_status;
        mavlink_message_t msg_battery_status;
        //电量
        battery_status.voltages[1]=(uint16_t)(batt->voltage*1000);
        battery_status.current_battery=(int16_t)(batt->current*100);
        mavlink_msg_battery_status_encode(mavlink_system.sysid, mavlink_system.compid, &msg_battery_status, &battery_status);
        mavlink_send_msg(mav_chan, &msg_battery_status);
    }
}

void odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
    pos_odom_001_x=(float)odom->pose.pose.position.x*100;//位置点改为FRU坐标
    pos_odom_001_y=-(float)odom->pose.pose.position.y*100;
    pos_odom_001_z=(float)odom->pose.pose.position.z*100;
    float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                                (float)odom->pose.pose.orientation.x,
                                (float)odom->pose.pose.orientation.y,
                                (float)odom->pose.pose.orientation.z};
    mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_001_roll, &pos_odom_001_pitch, &pos_odom_001_yaw);
    pos_odom_001_pitch=-pos_odom_001_pitch;//姿态改为FRD坐标
    pos_odom_001_yaw=-pos_odom_001_yaw;
    if(get_heartbeat){
        mavlink_global_vision_position_estimate_t global_attitude_position;
        mavlink_message_t msg_global_attitude_position;
        //姿态+位置
        global_attitude_position.pitch=pos_odom_001_pitch;
        global_attitude_position.roll=pos_odom_001_roll;
        global_attitude_position.yaw=pos_odom_001_yaw;
        global_attitude_position.x=pos_odom_001_x;
        global_attitude_position.y=pos_odom_001_y;
        global_attitude_position.z=pos_odom_001_z;
        mavlink_msg_global_vision_position_estimate_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_attitude_position, &global_attitude_position);
        mavlink_send_msg(mav_chan, &msg_global_attitude_position);
    }
}

void execute_mission_1hz(const ros::TimerEvent &event){
    if(get_heartbeat&&(ros::Time::now().toSec()-time_heartbeat>5.0f)){
        get_heartbeat=false;
        close(socket_cli);
        get_drone=0;
    }
    mav_send_heartbeat();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "fcu_server_001");
    ros::NodeHandle nh("~");
    nh.param("DRONE_PORT", PORT, 10005);
    mavlink_system.sysid=254;//强制飞控进入自主模式
    mavlink_system.compid=MAV_COMP_ID_MISSIONPLANNER;

    ros::Rate loop_rate(200);
    battery = nh.subscribe<sensor_msgs::BatteryState>("/fcu_bridge_001/battery_state", 100, battHandler, ros::TransportHints().tcpNoDelay());
    odom001=nh.subscribe<nav_msgs::Odometry>("odom_global_001", 100, odom_global001_handler);
    gnss_001=nh.subscribe<sensor_msgs::NavSatFix>("/fcu_bridge_001/gnss_global_001", 100, gnssHandler, ros::TransportHints().tcpNoDelay());
    cmd=nh.subscribe<std_msgs::Int16>("command", 100, cmdHandler, ros::TransportHints().tcpNoDelay());
    mission_pub_001 = nh.advertise<std_msgs::Float32MultiArray>("mission_001",100);
    command = nh.advertise<std_msgs::Int16>("command",100);
    gnss_origin=nh.advertise<sensor_msgs::NavSatFix>("/fcu_bridge_001/gnss_origin",100);
    ros::Timer timer_mission_1hz = nh.createTimer(ros::Duration(1.0),execute_mission_1hz,false);
    ros::Duration(1.0).sleep();

    rbInit(&mav_buf_send, TxBuffer, BUF_SIZE);
    rbInit(&mav_buf_receive, RxBuffer, BUF_SIZE);
    br = new tf::TransformBroadcaster;

    // 1. 创建套接字
    // AF_INET: IPv4, SOCK_STREAM: TCP
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "Socket creation failed" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Socket created successfully." << std::endl;

    // 2. 设置套接字选项 (允许端口复用，防止重启服务时报错 "Address already in use")
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) == -1) {
        std::cerr << "Setsockopt failed" << std::endl;
        close(server_fd);
        return EXIT_FAILURE;
    }

    // 3. 配置服务器地址结构
    struct sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡接口
    address.sin_port = htons(PORT);       // 转换端口号为网络字节序

    // 4. 绑定套接字到地址和端口
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == -1) {
        std::cerr << "Bind failed" << std::endl;
        close(server_fd);
        return EXIT_FAILURE;
    }
    std::cout << "Bound to port " << PORT << std::endl;

    // 5. 开始监听
    // 参数 3 表示等待连接队列的最大长度
    if (listen(server_fd, 3) == -1) {
        std::cerr << "Listen failed" << std::endl;
        close(server_fd);
        return EXIT_FAILURE;
    }
    std::cout << "Listening for connections..." << std::endl;

	time_start=ros::Time::now().toSec();
    struct sockaddr_in client_address;
    socklen_t addrlen = sizeof(client_address);
    int n=0;
    while (ros::ok()) {
        ros::spinOnce();
        if(!get_drone){
            // 阻塞直到有客户端连接
            socket_cli = accept(server_fd, (struct sockaddr*)&client_address, &addrlen);
            if (socket_cli == -1) {
                std::cerr << "Accept failed" << std::endl;
                continue;
            }else{
                get_drone=1;
            }
            int flag = fcntl(socket_cli,F_GETFL,0);//获取socket_cli当前的状态
            if(flag<0){
                printf("fcntl F_GETFL fail");
                close(socket_cli);
                return -1;
            }
            fcntl(socket_cli, F_SETFL, flag | O_NONBLOCK);//设置为非阻塞态
            // 获取客户端 IP 地址用于日志
            char ip_buffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_address.sin_addr), ip_buffer, INET_ADDRSTRLEN);
            std::cout << "New connection from: " << ip_buffer << ":" << ntohs(client_address.sin_port) << std::endl;
        }
        n=read(socket_cli, buffer, BUF_SIZE - 1);

        if(n>0){
            for(uint16_t i=0; i<n; i++){
                rbPush(&mav_buf_receive, buffer[i]);
            }
        }
        parse_data();
        flush_data();
        loop_rate.sleep();
    }
    // 8. 关闭当前客户端连接
    close(socket_cli);
    close(server_fd);
    return 0;
}
