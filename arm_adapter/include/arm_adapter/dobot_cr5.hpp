/*
 * base_arm.hpp
 *
 * Created on: July 11, 2024
 * Description: For arx_lite arm from ARX方舟无限
 *
 * Copyright (c) 2024 AgileX Robotics (lagrangeluo)
 */
#ifndef AGILEX_ARX_LITE_HPP
#define AGILEX_ARX_LITE_HPP

#define USE_ROS_SERVICE

#include <arm_adapter/base_arm.hpp>
#include <dobot_bringup/EnableRobot.h>
#include <dobot_bringup/MovJ.h>
#include <dobot_bringup/ResetRobot.h>


class dobot_cr5 : public ArmCommonInterface<dobot_bringup::MovJ>
{
    public:
        dobot_cr5(){}
        dobot_cr5(ros::NodeHandle node,param_t& param_list);

        // 虚函数复写实现
        void publish_pose() override;
        void move_joint() override;
        void joystick_callback(const survive_publisher::joystick::ConstPtr msg) override;
        dobot_bringup::MovJ change_joint_type(geometry_msgs::Pose pose) override;

    private:
        // for arx arm
        dobot_bringup::MovJ cmd;
        ros::ServiceClient reset_client;

};

#endif /* ARM_BASE_HPP */