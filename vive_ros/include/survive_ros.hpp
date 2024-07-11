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
        void joy_topic_callback(const sensor_msgs::Joy::ConstPtr msg);
        void joystick_callback(const survive_publisher::joystick::ConstPtr msg);
        void check_start_teleop();
    
    private:
        // ros
        ros::Subscriber joy_sub;
        ros::Subscriber joystick_sub;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        geometry_msgs::TransformStamped tracker_static;
        ros::Publisher joint_pub;

        // tracker and base station name
        std::string world_name;
        std::string tracker_left;
        std::string tracker_right;
        std::string teleop_base;
        std::string base_station_1;
        std::string base_station_2;

        // start flag from joy button
        bool start_teleop;

        // for arx arm
        arm_control::PosCmd cmd;
};