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
        // 虚函数：
        // 发布坐标点到机械臂
        virtual void publish_pose() = 0;

        // 摇杆回调函数
        virtual void joystick_callback(const survive_publisher::joystick::ConstPtr msg) = 0;

        // 发送控制指令
        void publish_cmd(arm_cmd_type& cmd)
        {
            cmd_pub.publish(cmd);
        }

        // 一次循环
        void loop_once()
        {
            update_arm();
            publish_pose();
        }

        //可复用的实函数
        ArmCommonInterface(){}
        ArmCommonInterface(ros::NodeHandle node,param_t& param_list)
        : nh(node), param_list(param_list){}

        void init_interface()
        {
            // 初始化tf2
            tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

            //初始化摇杆话题接收者
            joystick_sub = nh.subscribe<survive_publisher::joystick>
                (param_list.joystick_topic,5,std::bind(&ArmCommonInterface::joystick_callback, this, std::placeholders::_1));

            cmd_pub = nh.advertise<arm_cmd_type>(param_list.cmd_topic_name,5);
        }

        transform_t get_transform(){return transform;}
        bool get_tf_start_flag(){return tf2_start_flag;}

        //更新坐标变换
        void update_arm()
        {
            ros::Time now = ros::Time::now();
            ros::Duration timeout(0.1);
            try
            {
                transform.tf_trans = tfBuffer.lookupTransform(param_list.base_link, param_list.left_arm_link, now - timeout,timeout);
                double x = transform.tf_trans.transform.translation.x;
                double y = transform.tf_trans.transform.translation.y;
                double z = transform.tf_trans.transform.translation.z;

                tf2::Quaternion quat(
                    transform.tf_trans.transform.rotation.x,
                    transform.tf_trans.transform.rotation.y,
                    transform.tf_trans.transform.rotation.z,
                    transform.tf_trans.transform.rotation.w);

                tf2::Matrix3x3(quat).getRPY(transform.roll, transform.pitch, transform.yaw);

                //标志位
                tf2_start_flag = true;

                // ROS_INFO("Translation: x=%f, y=%f, z=%f", x, y, z);
                // ROS_INFO("Rotation: roll=%f, pitch=%f, yaw=%f", transform.roll, transform.pitch, transform.yaw);
            }
            catch (tf2::TransformException &ex)
            {
                //ROS_WARN("Transform not found: %s", ex.what());
                tf2_start_flag = false;
            }
        }


    private:
        ros::NodeHandle nh;
        //tf相关
        tf2_ros::Buffer tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        bool tf2_start_flag = false;

        // 存储tf变换和位姿
        transform_t transform;
        // 存储参数
        param_t param_list;

        ros::Publisher cmd_pub;
        ros::Subscriber joystick_sub;

};

#endif /* ARM_BASE_HPP */