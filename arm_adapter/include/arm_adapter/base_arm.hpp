/*
 * base_arm.hpp
 *
 * Created on: July 8, 2024
 * Description:
 *
 * Copyright (c) 2024 AgileX Robotics (lagrangeluo)
 */
#ifndef AGILEX_BASE_HPP
#define AGILEX_BASE_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include <survive_publisher/joystick.h>


//坐标系名字
struct param_t
{
    //坐标系名字
    std::string base_link;
    std::string left_arm_link;
    std::string right_arm_link;
    //摇杆话题名字
    std::string joystick_topic;
    //机械臂控制话题名字
    std::string cmd_topic_name;
    //初始化关节角度
    std::vector<double> home_joint_angle;
    //也可以使用初始化tcp坐标
    geometry_msgs::Pose home_tcp;
    //返回初始采样次数和播放插值轨迹频率
    uint16_t sample_time;
    uint16_t step_freq;
};

//存储tf变换的位姿结构体
struct transform_t
{
    geometry_msgs::TransformStamped tf_trans;
    double roll, pitch, yaw;
};

template<typename arm_cmd_type>
class ArmCommonInterface{
    public:
        // 可复用的实例化函数
        ArmCommonInterface(){}
        ArmCommonInterface(ros::NodeHandle node,param_t& param_list)
        : nh(node), param_list(param_list){}

        // 虚函数：
        // 发布坐标点到机械臂
        virtual void publish_pose() = 0;

        // 摇杆回调函数
        virtual void joystick_callback(const survive_publisher::joystick::ConstPtr msg) = 0;

        // 发送关节控制
        virtual void move_joint() = 0;

        // 转换关节类型
        virtual arm_cmd_type change_joint_type(geometry_msgs::Pose pose) = 0;

        // 发送控制指令
        void publish_cmd(arm_cmd_type& cmd);

        // 一次循环
        void loop_once();

        // 获取私有变量
        transform_t get_transform(){return transform;}
        bool get_tf_start_flag(){return tf2_start_flag;}
        bool get_init_flag(){return init_flag;}
        void set_init_flag(bool value){init_flag = value;}
        param_t& get_param(){return param_list;}

        // 初始化ros接口
        void init_interface();

        // 更新坐标变换
        void update_arm();

        // 机械臂移动到初始状态
        void move_to_homepose();

    private:
        ros::NodeHandle nh;
        //tf相关
        tf2_ros::Buffer tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        bool tf2_start_flag = false;
        bool init_flag = false;

        // 存储tf变换和位姿
        transform_t transform;
        // 存储参数
        param_t param_list;

        ros::Publisher cmd_pub;
        ros::ServiceClient cmd_client;
        ros::Subscriber joystick_sub;

};

// 显式实例化模板类
#include <arm_control/PosCmd.h>
#include <dobot_bringup/MovJ.h>

template class ArmCommonInterface<arm_control::PosCmd>;
//template class ArmCommonInterface<dobot_bringup::MovJ>;

#endif /* ARM_BASE_HPP */