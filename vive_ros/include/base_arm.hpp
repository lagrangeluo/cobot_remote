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

#include <geometry_msgs/Pose.h>

class ArmCommonInterface{
    public:
        //虚函数：
        //发布坐标点到机械臂
        virtual void publish_pose() = 0;
        void update_arm()
        {
            
        }

        //可复用的实函数
        geometry_msgs::Pose get_transform(){return tranform;}
    private:
        geometry_msgs::Pose tranform;


};

#endif /* ARM_BASE_HPP */