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

#include <arm_adapter/base_arm.hpp>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>

class arx_lite : public ArmCommonInterface<arm_control::PosCmd>
{
    public:
        arx_lite(){}
        arx_lite(ros::NodeHandle node,param_t& param_list);

        // 虚函数复写实现
        void publish_pose() override;
        void move_joint() override;
        void joystick_callback(const survive_publisher::joystick::ConstPtr msg) override;
        arm_control::PosCmd change_joint_type(geometry_msgs::Pose pose) override;

    private:
        // for arx arm
        arm_control::PosCmd cmd;
        arm_control::JointControl j_cmd;

};

#endif /* ARM_BASE_HPP */