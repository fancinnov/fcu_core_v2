#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <std_msgs/Int16.h>

static char buf[16] = {0};
static std_msgs::Int16 cmd;

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_command");
  ros::NodeHandle nh("~");
  ros::Publisher command = nh.advertise<std_msgs::Int16>("command",100);

  while (ros::ok()) {
    // 获取从键盘输入的数据
    printf("请输入指令：\n");
    ssize_t size = read(STDIN_FILENO, buf, sizeof(buf));
    if(size>0){
      if(size!=2){
        printf("指令错误！\n");
        continue;
      }
    }else{
      printf("禁用指令\n");
      ros::shutdown();
      return 0;
    }
    switch(buf[0]){
      case 'p':
        cmd.data=0;
        command.publish(cmd);
        printf("执行路径\n");
        break;
      case 'a':
        cmd.data=1;
        command.publish(cmd);
        printf("解锁\n");
        break;
      case 'd':
        cmd.data=2;
        command.publish(cmd);
        printf("锁定\n");
        break;
      case 't':
        cmd.data=3;
        command.publish(cmd);
        printf("起飞\n");
        break;
      case 'l':
        cmd.data=4;
        command.publish(cmd);
        printf("降落\n");
        break;
      case 'r':
        cmd.data=5;
        command.publish(cmd);
        printf("运行\n");
        break;
      case 's':
        cmd.data=6;
        command.publish(cmd);
        printf("停止\n");
        break;
      case '1':
        cmd.data=7;
        command.publish(cmd);
        printf("位置点1\n");
        break;
      case '2':
        cmd.data=8;
        command.publish(cmd);
        printf("位置点2\n");
        break;
      case '3':
        cmd.data=9;
        command.publish(cmd);
        printf("位置点3\n");
        break;
      case '4':
        cmd.data=10;
        command.publish(cmd);
        printf("位置点4\n");
        break;
      case 'q':
        cmd.data=1011;
        command.publish(cmd);
        printf("前向追踪\n");
        break;
      case 'w':
        cmd.data=1012;
        command.publish(cmd);
        printf("下视追踪\n");
        break;
      case 'e':
        cmd.data=1013;
        command.publish(cmd);
        printf("停止追踪\n");
        break;
      case 'z':
        cmd.data=1015;
        command.publish(cmd);
        printf("自由追踪\n");
        break;
      default:
        printf("非法指令！\n");
        break;
    }
  }

  return 0;
}
