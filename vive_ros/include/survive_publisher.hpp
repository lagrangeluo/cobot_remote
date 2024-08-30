#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "arm_control/PosCmd.h"
#include "survive_publisher/joystick.h"

class survive_ros_node
{
    public:
        void init();
        void joystick_callback(const survive_publisher::joystick::ConstPtr msg);
        void update_hand_frame();

        // 用于判断左手和右手是否存在
        bool if_left_exist();
        bool if_right_exist();
    
    private:
        // ros
        ros::Subscriber joy_sub;
        ros::Subscriber joystick_sub;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        geometry_msgs::TransformStamped tracker_static_left;
        geometry_msgs::TransformStamped tracker_static_right;
        ros::Publisher esp32_motor_pub;

        // tracker and base station name
        std::string world_name;
        std::string tracker_left;
        std::string tracker_right;
        std::string control_joystick;
        std::string teleop_base_left;
        std::string teleop_base_right;
        std::string left_hand;
        std::string right_hand;
        std::string base_station_1;
        std::string base_station_2;

        // 初始位置位置补偿
        double base_x_l, base_y_l, base_z_l, base_roll_l, base_pitch_l, base_yaw_l;
        double base_x_r, base_y_r, base_z_r, base_roll_r, base_pitch_r, base_yaw_r;
        // 追踪器到手位置补偿
        double hand_x,hand_y,hand_z,hand_roll,hand_pitch,hand_yaw;
        
        // start flag from joy button
        bool start_teleop_state;
        bool start_teleop;

        // for arx arm
        arm_control::PosCmd cmd;
};