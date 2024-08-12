#include "arm_adapter/agilex.hpp"


arx_lite::arx_lite(ros::NodeHandle node,param_t& param_list):ArmCommonInterface(node,param_list)
{
    // 基类ros接收者发布者初始化
    init_interface();
}

void arx_lite::publish_pose()
{
    if(get_tf_start_flag())
    {
        if(if_left_exist())
        {
            cmd.x = 1000*get_left_transform().tf_trans.transform.translation.x;
            cmd.y = 1000*get_left_transform().tf_trans.transform.translation.y;
            cmd.z = 1000*get_left_transform().tf_trans.transform.translation.z;
            cmd.roll = (180*get_left_transform().roll)/3.1415;
            cmd.pitch = (180*get_left_transform().pitch)/3.1415 + 90;
            cmd.yaw = (180*get_left_transform().yaw)/3.1415;
            // cmd.roll = 0;
            // cmd.pitch = 90;
            // cmd.yaw = 0;
            // 发布指令
            publish_cmd("left",cmd);
        }
        if(if_right_exist())
        {
            cmd.x = 1000*get_right_transform().tf_trans.transform.translation.x;
            cmd.y = 1000*get_right_transform().tf_trans.transform.translation.y;
            cmd.z = 1000*get_right_transform().tf_trans.transform.translation.z;
            // cmd.roll = get_right_transform().roll;
            // cmd.pitch = get_right_transform().pitch;
            // cmd.yaw = get_right_transform().yaw;
            cmd.roll = 0;
            cmd.pitch = 90;
            cmd.yaw = 0;
            // 发布指令
            publish_cmd("right",cmd);
        }
    }
}

arm_control::PosCmd arx_lite::change_joint_type(geometry_msgs::Pose pose)
{
    //
    arm_control::PosCmd cmd;
    cmd.x = pose.position.x;
    cmd.y = pose.position.y;
    cmd.z = pose.position.z;
    cmd.roll = pose.orientation.x;
    cmd.pitch = pose.orientation.y;
    cmd.yaw = pose.orientation.z;

    return cmd;
}

void arx_lite::move_joint()
{
    // for(auto iter = get_param().home_joint_angle.begin(); iter!=get_param().home_joint_angle.end();++iter)
    // {
    //     j_cmd.joint_pos.push_back(*iter);
    //     j_cmd.joint_vel.push_back();  
    // }
}

void arx_lite::joystick_callback(const survive_publisher::joystick::ConstPtr msg)
{
    if(get_tf_start_flag())
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
    else
    {
        if(get_init_flag() == false && msg->press_js == true)
        {
            move_to_homepose();
            set_init_flag(true);
        }
        if(msg->press_js == false)
        {
            set_init_flag(false);
        }
    }
    
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"agx_arm");
    ros::NodeHandle nh;
    ROS_INFO("arx_lite arm");
    
    param_t param_lists;

    nh.getParam("/vive/teleop_base",param_lists.base_link);
    nh.getParam("/vive/left_hand",param_lists.left_arm_link);
    nh.getParam("/vive/right_hand", param_lists.right_arm_link);
    nh.getParam("/vive/joystick_topic", param_lists.joystick_topic);
    nh.getParam("/vive/cmd_left_name", param_lists.cmd_left_name);
    nh.getParam("/vive/cmd_right_name", param_lists.cmd_right_name);
    //nh.getParam("/vive/joint_num", param_lists.joint_num);
    nh.getParam("/vive/init_angle", param_lists.home_joint_angle);
    nh.getParam("/vive/base_x", param_lists.home_tcp.position.x);
    nh.getParam("/vive/base_y", param_lists.home_tcp.position.y);
    nh.getParam("/vive/base_z", param_lists.home_tcp.position.z);
    nh.getParam("/vive/base_roll", param_lists.home_tcp.orientation.x);
    nh.getParam("/vive/base_pitch", param_lists.home_tcp.orientation.y);
    nh.getParam("/vive/base_yaw", param_lists.home_tcp.orientation.z);

    std::shared_ptr<arx_lite> node_ptr = std::make_shared<arx_lite>(nh,param_lists); 
    
    ros::Rate rate(25);
    while(nh.ok())
    {
        node_ptr->loop_once();
        ros::spinOnce();
        rate.sleep();
    }
}