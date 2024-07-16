#include <arm_adapter/arx_lite.hpp>
#include "arm_adapter/arx_lite.hpp"



arx_lite::arx_lite(ros::NodeHandle node,param_t& param_list):ArmCommonInterface(node,param_list)
{
    // 基类ros接收者发布者初始化
    init_interface();
}

void arx_lite::publish_pose()
{
    if(get_tf_start_flag())
    {
        cmd.x = get_transform().tf_trans.transform.translation.x;
        cmd.y = get_transform().tf_trans.transform.translation.y;
        cmd.z = get_transform().tf_trans.transform.translation.z;
        cmd.roll = get_transform().roll;
        cmd.pitch = get_transform().pitch;
        cmd.yaw = get_transform().yaw;

        // 发布指令
        publish_cmd(cmd);
    }
}

void arx_lite::joystick_callback(const survive_publisher::joystick::ConstPtr msg)
{
    if(msg->press_up == true && msg->press_down == false && cmd.gripper < 5)
    {
        cmd.gripper += 0.2;
    }
    else if(msg->press_down == true && msg->press_up == false && cmd.gripper > 0)
    {
        cmd.gripper -= 0.2;
    }
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"agx_arm");
    ros::NodeHandle nh;
    ROS_INFO("arx_lite arm");
    
    param_t param_lists;

    nh.getParam("/vive/teleop_base",param_lists.base_link);
    nh.getParam("/vive/tracker_left",param_lists.left_arm_link);
    nh.getParam("/vive/tracker_right", param_lists.right_arm_link);
    nh.getParam("/vive/joystick_topic", param_lists.joystick_topic);
    nh.getParam("/vive/cmd_topic_name", param_lists.cmd_topic_name);

    std::shared_ptr<arx_lite> node_ptr = std::make_shared<arx_lite>(nh,param_lists); 
    
    ros::Rate rate(25);
    while(nh.ok())
    {
        node_ptr->loop_once();
        ros::spinOnce();
        rate.sleep();
    }
}