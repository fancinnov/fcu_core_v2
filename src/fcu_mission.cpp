#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <geometry_msgs/InertiaStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "../mavlink/common/mavlink.h"
/**
 * 注意：工程中mission_xxx话题为发给飞控的目标值，目标位移应为FRU坐标系，目标姿态应为FRD坐标系
 */
static bool enable_path=false;
static bool enable_track=false;
static bool get_pos_cmd=false;
static bool follow_forward=false;
static bool follow_down=false;
static bool set_goal=false;
static bool use_goal_001=false;
static bool use_goal_002=false;
static bool use_goal_003=false;
static bool use_goal_004=false;
static bool use_goal_005=false;
static bool use_goal_006=false;
static uint8_t enable_pos=0;
float pos_odom_001_x=0.0f; float pos_odom_001_y=0.0f; float pos_odom_001_z=0.0f;
float pos_odom_002_x=0.0f; float pos_odom_002_y=0.0f; float pos_odom_002_z=0.0f;
float pos_odom_003_x=0.0f; float pos_odom_003_y=0.0f; float pos_odom_003_z=0.0f;
float pos_odom_004_x=0.0f; float pos_odom_004_y=0.0f; float pos_odom_004_z=0.0f;
float pos_odom_005_x=0.0f; float pos_odom_005_y=0.0f; float pos_odom_005_z=0.0f;
float pos_odom_006_x=0.0f; float pos_odom_006_y=0.0f; float pos_odom_006_z=0.0f;
float pos_odom_001_roll=0.0f; float pos_odom_001_pitch=0.0f; float pos_odom_001_yaw=0.0f;
float pos_odom_002_roll=0.0f; float pos_odom_002_pitch=0.0f; float pos_odom_002_yaw=0.0f;
float pos_odom_003_roll=0.0f; float pos_odom_003_pitch=0.0f; float pos_odom_003_yaw=0.0f;
float pos_odom_004_roll=0.0f; float pos_odom_004_pitch=0.0f; float pos_odom_004_yaw=0.0f;
float pos_odom_005_roll=0.0f; float pos_odom_005_pitch=0.0f; float pos_odom_005_yaw=0.0f;
float pos_odom_006_roll=0.0f; float pos_odom_006_pitch=0.0f; float pos_odom_006_yaw=0.0f;

float pos_takeoff_001_x=0.0f; float pos_takeoff_001_y=0.0f; float pos_takeoff_001_z=0.0f; float pos_takeoff_001_yaw=0.0f;
float pos_takeoff_002_x=0.0f; float pos_takeoff_002_y=0.0f; float pos_takeoff_002_z=0.0f; float pos_takeoff_002_yaw=0.0f;
float pos_takeoff_003_x=0.0f; float pos_takeoff_003_y=0.0f; float pos_takeoff_003_z=0.0f; float pos_takeoff_003_yaw=0.0f;
float pos_takeoff_004_x=0.0f; float pos_takeoff_004_y=0.0f; float pos_takeoff_004_z=0.0f; float pos_takeoff_004_yaw=0.0f;
float pos_takeoff_005_x=0.0f; float pos_takeoff_005_y=0.0f; float pos_takeoff_005_z=0.0f; float pos_takeoff_005_yaw=0.0f;
float pos_takeoff_006_x=0.0f; float pos_takeoff_006_y=0.0f; float pos_takeoff_006_z=0.0f; float pos_takeoff_006_yaw=0.0f;

typedef enum {
  ReadyToGoal,
  ExecutingGoal
} path_track_flag;
path_track_flag path_track_status = ReadyToGoal;

static std_msgs::Float32MultiArray mission_001;
static ros::Publisher mission_pub_001;
void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){
    case 0:
        enable_path=true;
        break;
    case 3://起飞时刻记录坐标,原地悬停
        enable_pos=0;
        pos_takeoff_001_x=pos_odom_001_x; pos_takeoff_001_y=pos_odom_001_y; pos_takeoff_001_z=pos_odom_001_z; pos_takeoff_001_yaw=pos_odom_001_yaw;
        pos_takeoff_002_x=pos_odom_002_x; pos_takeoff_002_y=pos_odom_002_y; pos_takeoff_002_z=pos_odom_002_z; pos_takeoff_002_yaw=pos_odom_002_yaw;
        pos_takeoff_003_x=pos_odom_003_x; pos_takeoff_003_y=pos_odom_003_y; pos_takeoff_003_z=pos_odom_003_z; pos_takeoff_003_yaw=pos_odom_003_yaw;
        pos_takeoff_004_x=pos_odom_004_x; pos_takeoff_004_y=pos_odom_004_y; pos_takeoff_004_z=pos_odom_004_z; pos_takeoff_004_yaw=pos_odom_004_yaw;
        pos_takeoff_005_x=pos_odom_005_x; pos_takeoff_005_y=pos_odom_005_y; pos_takeoff_005_z=pos_odom_005_z; pos_takeoff_005_yaw=pos_odom_005_yaw;
        pos_takeoff_006_x=pos_odom_006_x; pos_takeoff_006_y=pos_odom_006_y; pos_takeoff_006_z=pos_odom_006_z; pos_takeoff_006_yaw=pos_odom_006_yaw;
        if(set_goal){
          //发布mission
          mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
          mission_001.layout.dim[0].label = "mission_001";
          mission_001.layout.dim[0].size = 11;
          mission_001.layout.dim[0].stride = 1;
          mission_001.data.resize(11);
          mission_001.data[0] = 0.0f;//rad
          mission_001.data[1] = 0.0f;//rad/s
          mission_001.data[2] = pos_takeoff_001_x;//x
          mission_001.data[3] = pos_takeoff_001_y;//y
          mission_001.data[4] = 1.0f;//take off alt
          mission_001.data[5]=0;//vx
          mission_001.data[6]=0;//vy
          mission_001.data[7]=0;//vz
          mission_001.data[8]=0;//ax
          mission_001.data[9]=0;//ay
          mission_001.data[10]=0;//az
          mission_pub_001.publish(mission_001);
        }
        break;
    case 5:
        enable_track=true;
        break;
    case 6:
        enable_track=false;
        enable_path=false;
        enable_pos=255;
        break;
    case 7:
        enable_pos=1;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 8:
        enable_pos=2;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 9:
        enable_pos=3;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 10:
        enable_pos=4;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 101:
        set_goal=true;
        break;
    case 102:
        get_pos_cmd=true;
        break;
    case 1001:
        follow_forward=true;
        break;
    case 1002:
        follow_down=true;
        break;
    case 1003:
        follow_forward=false;
        follow_down=false;
        break;
    case 1011:
        get_pos_cmd=true;
        break;
    case 1012:
        get_pos_cmd=true;
        break;
    case 1013:
        get_pos_cmd=false;
        break;
    case 1101:
        pos_takeoff_001_x=pos_odom_001_x; pos_takeoff_001_y=pos_odom_001_y; pos_takeoff_001_z=pos_odom_001_z; pos_takeoff_001_yaw=pos_odom_001_yaw;
        break;
    case 1102:
        pos_takeoff_002_x=pos_odom_002_x; pos_takeoff_002_y=pos_odom_002_y; pos_takeoff_002_z=pos_odom_002_z; pos_takeoff_002_yaw=pos_odom_002_yaw;
        break;
    case 1103:
        pos_takeoff_003_x=pos_odom_003_x; pos_takeoff_003_y=pos_odom_003_y; pos_takeoff_003_z=pos_odom_003_z; pos_takeoff_003_yaw=pos_odom_003_yaw;
        break;
    case 1104:
        pos_takeoff_004_x=pos_odom_004_x; pos_takeoff_004_y=pos_odom_004_y; pos_takeoff_004_z=pos_odom_004_z; pos_takeoff_004_yaw=pos_odom_004_yaw;
        break;
    case 1105:
        pos_takeoff_005_x=pos_odom_005_x; pos_takeoff_005_y=pos_odom_005_y; pos_takeoff_005_z=pos_odom_005_z; pos_takeoff_005_yaw=pos_odom_005_yaw;
        break;
    case 1106:
        pos_takeoff_006_x=pos_odom_006_x; pos_takeoff_006_y=pos_odom_006_y; pos_takeoff_006_z=pos_odom_006_z; pos_takeoff_006_yaw=pos_odom_006_yaw;
        break;
    default:
        break;
  }
}

static float yaw=0.0f, yaw_rate=0.0f;
static float px=0.0f, py=0.0f, pz=0.0f;
static float vx=0.0f, vy=0.0f, vz=0.0f;
static float ax=0.0f, ay=0.0f, az=0.0f;
static float theta=0.0f;

static float  px1=0.0f, py1=0.0f, pz1=0.0f, yaw1=0.0f,
              px2=0.0f, py2=0.0f, pz2=0.0f, yaw2=0.0f,
              px3=0.0f, py3=0.0f, pz3=0.0f, yaw3=0.0f,
              px4=0.0f, py4=0.0f, pz4=0.0f, yaw4=0.0f,
              px5=0.0f, py5=0.0f, pz5=0.0f, yaw5=0.0f,
              px6=0.0f, py6=0.0f, pz6=0.0f, yaw6=0.0f;

static int goal_point = 0;
void SetGoal(int id, float target_x,float target_y,float target_z, float target_yaw)
{
  switch (id)
  {
  case 1://1号机
    px1=target_x;
    py1=target_y;
    pz1=target_z;
    yaw1=target_yaw;
    break;
  case 2://2号机
    px2=target_x;
    py2=target_y;
    pz2=target_z;
    yaw2=target_yaw;
    break;
  case 3://3号机
    px3=target_x;
    py3=target_y;
    pz3=target_z;
    yaw3=target_yaw;
    break;
  case 4://4号机
    px4=target_x;
    py4=target_y;
    pz4=target_z;
    yaw4=target_yaw;
    break;
  case 5://5号机
    px5=target_x;
    py5=target_y;
    pz5=target_z;
    yaw5=target_yaw;
    break;
  case 6://6号机
    px6=target_x;
    py6=target_y;
    pz6=target_z;
    yaw6=target_yaw;
    break;
  default:
    break;
  }
}

bool IsReachGoal(int id, float dis)
{
  switch (id)
  {
  case 1:
    if(abs(px1-pos_odom_001_x) < dis && abs(py1-pos_odom_001_y) < dis){
      return true;
    }
    break;
  case 2:
    if(abs(px2-pos_odom_002_x) < dis && abs(py2-pos_odom_002_y) < dis){
      return true;
    }
    break;
  case 3:
    if(abs(px3-pos_odom_003_x) < dis && abs(py3-pos_odom_003_y) < dis){
      return true;
    }
    break;
  case 4:
    if(abs(px4-pos_odom_004_x) < dis && abs(py4-pos_odom_004_y) < dis){
      return true;
    }
    break;
  case 5:
    if(abs(px5-pos_odom_005_x) < dis && abs(py5-pos_odom_005_y) < dis){
      return true;
    }
    break;
  case 6:
    if(abs(px6-pos_odom_006_x) < dis && abs(py6-pos_odom_006_y) < dis){
      return true;
    }
    break;
  default:
    break;
  }
  return false;
}

void execute_mission_001(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_001){
    return;
  }
  //发布mission
  mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_001.layout.dim[0].label = "mission_001";
  mission_001.layout.dim[0].size = 11;
  mission_001.layout.dim[0].stride = 1;
  mission_001.data.resize(11);
  mission_001.data[0]=yaw1;//rad
  mission_001.data[1]=0.0;//rad/s
  mission_001.data[2]=px1;//x
  mission_001.data[3]=py1;//y
  mission_001.data[4]=pz1;//z
  mission_001.data[5]=vx;//vx
  mission_001.data[6]=vy;//vy
  mission_001.data[7]=vz;//vz
  mission_001.data[8]=ax;//ax
  mission_001.data[9]=ay;//ay
  mission_001.data[10]=az;//az
  mission_pub_001.publish(mission_001);
  use_goal_001=false;     
}

static std_msgs::Float32MultiArray mission_002;
static ros::Publisher mission_pub_002;
void execute_mission_002(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_002){
    return;
  }
  //发布mission_002
  mission_002.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_002.layout.dim[0].label = "mission_002";
  mission_002.layout.dim[0].size = 11;
  mission_002.layout.dim[0].stride = 1;
  mission_002.data.resize(11);
  mission_002.data[0]=yaw2;//rad
  mission_002.data[1]=0.0f;//rad/s
  mission_002.data[2]=px2;//x
  mission_002.data[3]=py2;//y
  mission_002.data[4]=pz2;//z
  mission_002.data[5]=0.0f;//vx
  mission_002.data[6]=0.0f;//vy
  mission_002.data[7]=0.0f;//vz
  mission_002.data[8]=0.0f;//ax
  mission_002.data[9]=0.0f;//ay
  mission_002.data[10]=0.0f;//az
  mission_pub_002.publish(mission_002);
  use_goal_002=false;
}

static std_msgs::Float32MultiArray mission_003;
static ros::Publisher mission_pub_003;
void execute_mission_003(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_003){
    return;
  }
  //发布mission_003
  mission_003.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_003.layout.dim[0].label = "mission_003";
  mission_003.layout.dim[0].size = 11;
  mission_003.layout.dim[0].stride = 1;
  mission_003.data.resize(11);
  mission_003.data[0]=yaw3;//rad
  mission_003.data[1]=0.0f;//rad/s
  mission_003.data[2]=px3;//x
  mission_003.data[3]=py3;//y
  mission_003.data[4]=pz3;//z
  mission_003.data[5]=0.0f;//vx
  mission_003.data[6]=0.0f;//vy
  mission_003.data[7]=0.0f;//vz
  mission_003.data[8]=0.0f;//ax
  mission_003.data[9]=0.0f;//ay
  mission_003.data[10]=0.0f;//az
  mission_pub_003.publish(mission_003);
  use_goal_003=false;
}

static std_msgs::Float32MultiArray mission_004;
static ros::Publisher mission_pub_004;
void execute_mission_004(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_004){
    return;
  }
  //发布mission_004
  mission_004.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_004.layout.dim[0].label = "mission_004";
  mission_004.layout.dim[0].size = 11;
  mission_004.layout.dim[0].stride = 1;
  mission_004.data.resize(11);
  mission_004.data[0]=yaw4;//rad
  mission_004.data[1]=0.0f;//rad/s
  mission_004.data[2]=px4;//x
  mission_004.data[3]=py4;//y
  mission_004.data[4]=pz4;//z
  mission_004.data[5]=0.0f;//vx
  mission_004.data[6]=0.0f;//vy
  mission_004.data[7]=0.0f;//vz
  mission_004.data[8]=0.0f;//ax
  mission_004.data[9]=0.0f;//ay
  mission_004.data[10]=0.0f;//az
  mission_pub_004.publish(mission_004);
  use_goal_004=false;
}

static std_msgs::Float32MultiArray mission_005;
static ros::Publisher mission_pub_005;
void execute_mission_005(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_005){
    return;
  }
  //发布mission_005
  mission_005.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_005.layout.dim[0].label = "mission_005";
  mission_005.layout.dim[0].size = 11;
  mission_005.layout.dim[0].stride = 1;
  mission_005.data.resize(11);
  mission_005.data[0]=yaw5;//rad
  mission_005.data[1]=0.0f;//rad/s
  mission_005.data[2]=px5;//x
  mission_005.data[3]=py5;//y
  mission_005.data[4]=pz5;//z
  mission_005.data[5]=0.0f;//vx
  mission_005.data[6]=0.0f;//vy
  mission_005.data[7]=0.0f;//vz
  mission_005.data[8]=0.0f;//ax
  mission_005.data[9]=0.0f;//ay
  mission_005.data[10]=0.0f;//az
  mission_pub_005.publish(mission_005);
  use_goal_005=false;
}

static std_msgs::Float32MultiArray mission_006;
static ros::Publisher mission_pub_006;
void execute_mission_006(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_006){
    return;
  }
  //发布mission_006
  mission_006.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_006.layout.dim[0].label = "mission_006";
  mission_006.layout.dim[0].size = 11;
  mission_006.layout.dim[0].stride = 1;
  mission_006.data.resize(11);
  mission_006.data[0]=yaw6;//rad
  mission_006.data[1]=0.0f;//rad/s
  mission_006.data[2]=px6;//x
  mission_006.data[3]=py6;//y
  mission_006.data[4]=pz6;//z
  mission_006.data[5]=0.0f;//vx
  mission_006.data[6]=0.0f;//vy
  mission_006.data[7]=0.0f;//vz
  mission_006.data[8]=0.0f;//ax
  mission_006.data[9]=0.0f;//ay
  mission_006.data[10]=0.0f;//az
  mission_pub_006.publish(mission_006);
  use_goal_006=false;
}

void odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_001_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_001_y=-(float)odom->pose.pose.position.y;
  pos_odom_001_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_001_roll, &pos_odom_001_pitch, &pos_odom_001_yaw);
  pos_odom_001_pitch=-pos_odom_001_pitch;//姿态改为FRD坐标
  pos_odom_001_yaw=-pos_odom_001_yaw;
}

void odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_002_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_002_y=-(float)odom->pose.pose.position.y;
  pos_odom_002_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_002_roll, &pos_odom_002_pitch, &pos_odom_002_yaw);
  pos_odom_002_pitch=-pos_odom_002_pitch;//姿态改为FRD坐标
  pos_odom_002_yaw=-pos_odom_002_yaw;
}

void odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_003_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_003_y=-(float)odom->pose.pose.position.y;
  pos_odom_003_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_003_roll, &pos_odom_003_pitch, &pos_odom_003_yaw);
  pos_odom_003_pitch=-pos_odom_003_pitch;//姿态改为FRD坐标
  pos_odom_003_yaw=-pos_odom_003_yaw;
}

void odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_004_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_004_y=-(float)odom->pose.pose.position.y;
  pos_odom_004_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_004_roll, &pos_odom_004_pitch, &pos_odom_004_yaw);
  pos_odom_004_pitch=-pos_odom_004_pitch;//姿态改为FRD坐标
  pos_odom_004_yaw=-pos_odom_004_yaw;
}

void odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_005_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_005_y=-(float)odom->pose.pose.position.y;
  pos_odom_005_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_005_roll, &pos_odom_005_pitch, &pos_odom_005_yaw);
  pos_odom_005_pitch=-pos_odom_005_pitch;//姿态改为FRD坐标
  pos_odom_005_yaw=-pos_odom_005_yaw;
}

void odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_006_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_006_y=-(float)odom->pose.pose.position.y;
  pos_odom_006_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_006_roll, &pos_odom_006_pitch, &pos_odom_006_yaw);
  pos_odom_006_pitch=-pos_odom_006_pitch;//姿态改为FRD坐标
  pos_odom_006_yaw=-pos_odom_006_yaw;
}

void pos_cmd_handler(const quadrotor_msgs::PositionCommand::ConstPtr& pose_plan)//仅机载电脑运行此函数
{
  if(follow_forward||follow_down){
    return;
  }
  //发布mission
  /**
 * 注意：工程中/fcu_bridge/mission_xxx话题为发给飞控的目标值，目标位移应为FRU坐标系，目标姿态应为FRD坐标系
 */
  get_pos_cmd=true;
  mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_001.layout.dim[0].label = "mission_001";
  mission_001.layout.dim[0].size = 11;
  mission_001.layout.dim[0].stride = 1;
  mission_001.data.resize(11);
  mission_001.data[0]=-pose_plan->yaw;//rad
  mission_001.data[1]=-pose_plan->yaw_dot;//rad/s
  mission_001.data[2]=pose_plan->position.x;//x
  mission_001.data[3]=-pose_plan->position.y;//y
  mission_001.data[4]=pose_plan->position.z;//z
  mission_001.data[5]=pose_plan->velocity.x;//vx
  mission_001.data[6]=-pose_plan->velocity.y;//vy
  mission_001.data[7]=pose_plan->velocity.z;//vz
  mission_001.data[8]=pose_plan->acceleration.x;//ax
  mission_001.data[9]=-pose_plan->acceleration.y;//ay
  mission_001.data[10]=pose_plan->acceleration.z;//az
  mission_pub_001.publish(mission_001);
}

void follow_handler(const std_msgs::Float32MultiArray::ConstPtr& follow){
  if(follow_forward){
    get_pos_cmd=true;
    //发布mission
    if(follow->data[2]==0.0f&&follow->data[3]==0.0f){
      printf("No tracking!\n");
      return;
    }
    float global_dx = follow->data[2] * cosf(pos_odom_001_yaw) - follow->data[3] * sinf(pos_odom_001_yaw);
    float global_dy = follow->data[2] * sinf(pos_odom_001_yaw) + follow->data[3] * cosf(pos_odom_001_yaw);
    mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mission_001.layout.dim[0].label = "mission_001";
    mission_001.layout.dim[0].size = 11;
    mission_001.layout.dim[0].stride = 1;
    mission_001.data.resize(11);
    mission_001.data[0]=pos_odom_001_yaw+follow->data[0];//rad
    mission_001.data[1]=0.0f;//rad/s
    mission_001.data[2]=pos_odom_001_x+global_dx;//x
    mission_001.data[3]=pos_odom_001_y+global_dy;//y
    mission_001.data[4]=0.0f;//z
    mission_001.data[5]=0.0f;//vx
    mission_001.data[6]=0.0f;//vy
    mission_001.data[7]=0.0f;//vz
    mission_001.data[8]=0.0f;//ax
    mission_001.data[9]=0.0f;//ay
    mission_001.data[10]=0.0f;//az
    mission_pub_001.publish(mission_001); 
  }else if(follow_down){
    get_pos_cmd=true;
    //发布mission
    if(follow->data[2]==0.0f&&follow->data[3]==0.0f){
      printf("No tracking!\n");
      return;
    }
    float global_dx = follow->data[2] * cosf(pos_odom_001_yaw) - follow->data[3] * sinf(pos_odom_001_yaw);
    float global_dy = follow->data[2] * sinf(pos_odom_001_yaw) + follow->data[3] * cosf(pos_odom_001_yaw);
    mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mission_001.layout.dim[0].label = "mission_001";
    mission_001.layout.dim[0].size = 11;
    mission_001.layout.dim[0].stride = 1;
    mission_001.data.resize(11);
    mission_001.data[0]=0.0f;//rad
    mission_001.data[1]=0.0f;//rad/s
    mission_001.data[2]=pos_odom_001_x+global_dx;//x
    mission_001.data[3]=pos_odom_001_y+global_dy;//y
    mission_001.data[4]=0.0f;//z
    mission_001.data[5]=0.0f;//vx
    mission_001.data[6]=0.0f;//vy
    mission_001.data[7]=0.0f;//vz
    mission_001.data[8]=0.0f;//ax
    mission_001.data[9]=0.0f;//ay
    mission_001.data[10]=0.0f;//az
    mission_pub_001.publish(mission_001);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_mission");
  ros::NodeHandle nh("~");
  ros::Subscriber comm=nh.subscribe<std_msgs::Int16>("/fcu_command/command", 100, cmdHandler);
  ros::Subscriber odom001=nh.subscribe<nav_msgs::Odometry>("odom_global_001", 100, odom_global001_handler);
  ros::Subscriber odom002=nh.subscribe<nav_msgs::Odometry>("odom_global_002", 100, odom_global002_handler);
  ros::Subscriber odom003=nh.subscribe<nav_msgs::Odometry>("odom_global_003", 100, odom_global003_handler);
  ros::Subscriber odom004=nh.subscribe<nav_msgs::Odometry>("odom_global_004", 100, odom_global004_handler);
  ros::Subscriber odom005=nh.subscribe<nav_msgs::Odometry>("odom_global_005", 100, odom_global005_handler);
  ros::Subscriber odom006=nh.subscribe<nav_msgs::Odometry>("odom_global_006", 100, odom_global006_handler);
  ros::Subscriber pos_cmd=nh.subscribe<quadrotor_msgs::PositionCommand>("pos_cmd", 100, pos_cmd_handler);
  ros::Subscriber mission_follow=nh.subscribe<std_msgs::Float32MultiArray>("mission_follow", 100, follow_handler);

  mission_pub_001 = nh.advertise<std_msgs::Float32MultiArray>("mission_001",100);
  mission_pub_002 = nh.advertise<std_msgs::Float32MultiArray>("mission_002",100);
  mission_pub_003 = nh.advertise<std_msgs::Float32MultiArray>("mission_003",100);
  mission_pub_004 = nh.advertise<std_msgs::Float32MultiArray>("mission_004",100);
  mission_pub_005 = nh.advertise<std_msgs::Float32MultiArray>("mission_005",100);
  mission_pub_006 = nh.advertise<std_msgs::Float32MultiArray>("mission_006",100);
  ros::Timer timer_mission_001 = nh.createTimer(ros::Duration(0.1),execute_mission_001,false);
  ros::Timer timer_mission_002 = nh.createTimer(ros::Duration(0.1),execute_mission_002,false);
  ros::Timer timer_mission_003 = nh.createTimer(ros::Duration(0.1),execute_mission_003,false);
  ros::Timer timer_mission_004 = nh.createTimer(ros::Duration(0.1),execute_mission_004,false);
  ros::Timer timer_mission_005 = nh.createTimer(ros::Duration(0.1),execute_mission_005,false);
  ros::Timer timer_mission_006 = nh.createTimer(ros::Duration(0.1),execute_mission_006,false);

  ros::Rate loop_rate(200);
  while (ros::ok()) {
    ros::spinOnce();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0, 0,0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "uwb"));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "world"));
    if(enable_track){
      theta+=M_PI/20/200;

      yaw1=0.0;
      px1=1.0*cosf(theta)+2;
      py1=1.0*sinf(theta)+2;
      pz1=0.6;

      yaw2=0.0;
      px2=1.0*cosf(theta+M_PI*2/6)+2;
      py2=1.0*sinf(theta+M_PI*2/6)+2;
      pz2=0.6;

      yaw3=0.0;
      px3=1.0*cosf(theta+M_PI*4/6)+2;
      py3=1.0*sinf(theta+M_PI*4/6)+2;
      pz3=0.6;

      yaw4=0.0;
      px4=1.0*cosf(theta+M_PI*6/6)+2;
      py4=1.0*sinf(theta+M_PI*6/6)+2;
      pz4=0.6;

      yaw5=0.0;
      px5=1.0*cosf(theta+M_PI*8/6)+2;
      py5=1.0*sinf(theta+M_PI*8/6)+2;
      pz5=0.6;

      yaw6=0.0;
      px6=1.0*cosf(theta+M_PI*10/6)+2;
      py6=1.0*sinf(theta+M_PI*10/6)+2;
      pz6=0.6;
    }else if(enable_path){
      switch (path_track_status)
      {
        case ReadyToGoal:
          switch(goal_point)
          {
            case 0:
            SetGoal(1,0.5,0.5,0,0);
            break;
            case 1:
            SetGoal(1,1.0,1.0,0,0);
            break;
            case 2:
            SetGoal(1,1.5,1.0,0,0);
            break;
            case 3:
            SetGoal(1,2.0,1.0,0,0);
            break;
            case 4:
            SetGoal(1,2.5,1.0,0,0);
            break;
            case 5:
            SetGoal(1,2.5,2.0,0,0);
            break;
            case 6:
            SetGoal(1,2.0,2.0,0,0);
            break;
            case 7:
            SetGoal(1,1.5,2.0,0,0);
            break;
            case 8:
            SetGoal(1,1.0,2.0,0,0);
            break;
            case 9:
            SetGoal(1,0.5,2.0,0,0);
            break;
          }
          path_track_status = ExecutingGoal;
          break;
        case ExecutingGoal:
          if(IsReachGoal(1,0.1))
          {
            path_track_status = ReadyToGoal;
            goal_point++;
            break;
          }
          // std::cout << "Not Finish yet..." << std::endl;
          break;
        default:
          break;
      }
    }else{
        switch(enable_pos){
          case 0:
              px1=pos_takeoff_001_x; py1=pos_takeoff_001_y; pz1=0.0f; yaw1=pos_takeoff_001_yaw;
              px2=pos_takeoff_002_x; py2=pos_takeoff_002_y; pz2=0.0f; yaw2=pos_takeoff_002_yaw;
              px3=pos_takeoff_003_x; py3=pos_takeoff_003_y; pz3=0.0f; yaw3=pos_takeoff_003_yaw;
              px4=pos_takeoff_004_x; py4=pos_takeoff_004_y; pz4=0.0f; yaw4=pos_takeoff_004_yaw;
              px5=pos_takeoff_005_x; py5=pos_takeoff_005_y; pz5=0.0f; yaw5=pos_takeoff_005_yaw;
              px6=pos_takeoff_006_x; py6=pos_takeoff_006_y; pz6=0.0f; yaw6=pos_takeoff_006_yaw;
              break;
          case 1:
              px=1.0;
              py=1.0;
              pz=1.0;
              yaw=0.0;
              px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz; yaw1=yaw;
              px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz; yaw2=yaw;
              px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz; yaw3=yaw;
              px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz; yaw4=yaw;
              px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz; yaw5=yaw;
              px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz; yaw6=yaw;

              break;
          case 2:
              px=1.0;
              py=-1.0;
              pz=1.0;
              yaw=0.0;
              px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz; yaw1=yaw;
              px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz; yaw2=yaw;
              px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz; yaw3=yaw;
              px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz; yaw4=yaw;
              px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz; yaw5=yaw;
              px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz; yaw6=yaw;

              break;
          case 3:
              px=-1.0;
              py=-1.0;
              pz=1.0;
              yaw=0.0;
              px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz; yaw1=yaw;
              px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz; yaw2=yaw;
              px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz; yaw3=yaw;
              px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz; yaw4=yaw;
              px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz; yaw5=yaw;
              px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz; yaw6=yaw;

              break;
          case 4:
              px=0.0;
              py=0.0;
              pz=1.0;
              yaw=0.0;
              px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz; yaw1=yaw;
              px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz; yaw2=yaw;
              px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz; yaw3=yaw;
              px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz; yaw4=yaw;
              px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz; yaw5=yaw;
              px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz; yaw6=yaw;

              break;
          default:
              break;
        }
    }
    loop_rate.sleep();
  }
  return 0;
}