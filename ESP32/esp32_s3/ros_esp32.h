#include <ros.h>
#include <std_msgs/String.h>
#include <cstring>  // 需要包含此头文件
#include <lwip/inet.h>

//ros端口，句柄，消息
uint16_t serverPort = 11411;
ros::NodeHandle nh;
std_msgs::String str_msg;

// ros主机端的ip地址
char* ros_master_ip = (char*)malloc(32 * sizeof(char));
IPAddress server;

// 注册消息发布者和接收者
ros::Publisher* chatter;
ros::Subscriber<std_msgs::String>* sub;
ros::Subscriber<std_msgs::String>* sub_motor;
// 话题名字
char* ros_topic_pub_name;
char* ros_topic_sub_name;
char* ros_topic_motor;

//手柄名字，一般为了区分左右手，分为 left 和 right 两个名字
char* hand_name = (char*)malloc(32 * sizeof(char));

// ros通信是否成功标志位
bool if_ros_connected;
int watchdog_ros;

// 解析IP地址字符串的函数
IPAddress parseIPAddress(const char* ipStr) {
  uint8_t parts[4];  // 用于存储IP地址的四个部分
  sscanf(ipStr, "%hhu.%hhu.%hhu.%hhu", &parts[0], &parts[1], &parts[2], &parts[3]);
  return IPAddress(parts[0], parts[1], parts[2], parts[3]);
}

void feed_dog()
{
  watchdog_ros = 30;
}

// 接收者回调函数
void messageCb(const std_msgs::String& msg) {
  // 喂狗
  feed_dog();
}
void motorCb(const std_msgs::String& msg) {
  // 这里可以根据传入的msg来确认具体采用哪一种震动方式
  // 目前只采用最简单的
  // if(msg.data == "short")
  //   motor_start_short();
  // if(msg.data == "long")
    motor_start_long();
}

void init_ros()
{
  // ros初始化
  server = parseIPAddress(ros_master_ip);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  ros_topic_pub_name = (char*)malloc(strlen("esp32_json_string_") + strlen(hand_name) + 1);
  ros_topic_sub_name = (char*)malloc(strlen("esp32_master_fb_") + strlen(hand_name) + 1);
  // ros_topic_motor = (char*)malloc(strlen("esp32_motor_") + strlen(hand_name) + 1);
  ros_topic_motor = (char*)malloc(strlen("esp32_motor") + 1);

  // 将 "esp32_json_string" 复制到 ros_topic_pub_name,拼接 hand_name 到 ros_topic_pub_name
  strcpy(ros_topic_pub_name, "esp32_json_string_");
  strcat(ros_topic_pub_name, hand_name);
  strcpy(ros_topic_sub_name, "esp32_master_fb_");
  strcat(ros_topic_sub_name, hand_name);
  strcpy(ros_topic_motor, "esp32_motor");

  //此发布者用来向ros master端发送话题
  chatter = new ros::Publisher(ros_topic_pub_name, &str_msg);
  //此接收者用来接受主机端的话题，用来确认ros通信已建立成功
  sub = new ros::Subscriber<std_msgs::String>(ros_topic_sub_name, &messageCb);
  //此接收者用来接收主机端的马达震动命令，用来反馈主机端的功能
  sub_motor = new ros::Subscriber<std_msgs::String>(ros_topic_motor, &motorCb);

  //注册ros发布者接收者
  nh.advertise(*chatter);
  nh.subscribe(*sub);
  nh.subscribe(*sub_motor);
  if_ros_connected = false;
}


//看门狗循环监测，用于监测和rosmaster是否通信正常
void loop_watchdog()
{
  const long period = 100;  // 看门狗消耗的频率1hz

  static unsigned long timebefore = 0;
  unsigned long timenow = millis();
  if (timenow - timebefore >= period) {
    if(watchdog_ros > 0)
    {
      watchdog_ros --;
      if_ros_connected = true;
    }
    if(watchdog_ros == 0)
    {
      if_ros_connected = false;
    }
    timebefore = timenow;
  }
  else
  {
    return;
  }
}

