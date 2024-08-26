#include <ros.h>
#include <std_msgs/String.h>
#include <cstring>  // 需要包含此头文件

uint16_t serverPort = 11411;
ros::NodeHandle nh;
std_msgs::String str_msg;
// ros主机端的ip地址
IPAddress server(192, 168, 85, 105);

// 注册消息发布者
ros::Publisher* chatter;
char* hand_name = (char*)malloc(32 * sizeof(char));
char* ros_topic_name;

void init_ros()
{
  // ros初始化
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  ros_topic_name = (char*)malloc(strlen("esp32_json_string_") + strlen(hand_name) + 1);

  // 将 "esp32_json_string" 复制到 ros_topic_name
  strcpy(ros_topic_name, "esp32_json_string_");
  // 拼接 hand_name 到 ros_topic_name
  strcat(ros_topic_name, hand_name);

  chatter = new ros::Publisher(ros_topic_name, &str_msg);
  nh.advertise(*chatter);
}