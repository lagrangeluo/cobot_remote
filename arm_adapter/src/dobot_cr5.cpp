#include <arm_adapter/dobot_cr5.hpp>



dobot_cr5::dobot_cr5(ros::NodeHandle node,param_t& param_list):ArmCommonInterface(node,param_list)
{
    // 基类ros接收者发布者初始化
    init_interface();
    reset_client = node.serviceClient<dobot_bringup::ResetRobot>("/dobot_bringup/srv/ResetRobot");
}

void dobot_cr5::publish_pose()
{
    if(get_tf_start_flag())
    {
        cmd.request.x = 1000*get_transform().tf_trans.transform.translation.x;
        cmd.request.y = 1000*get_transform().tf_trans.transform.translation.y;
        cmd.request.z = 1000*get_transform().tf_trans.transform.translation.z;
        // cmd.request.a = (180/3.1415)*get_transform().roll;
        // cmd.request.b = (180/3.1415)*get_transform().pitch;
        // cmd.request.c = (180/3.1415)*get_transform().yaw;
        // cmd.request.x = 500;
        // cmd.request.y = 0;
        // cmd.request.z = 500;
        cmd.request.a = -180;
        cmd.request.b = 0;
        cmd.request.c = -90;

        // 发布指令
        dobot_bringup::ResetRobot reset;
        //reset_client.call(reset);
        publish_cmd(cmd);
    }
}

dobot_bringup::MovJ dobot_cr5::change_joint_type(geometry_msgs::Pose pose)
{
    //
    dobot_bringup::MovJ cmd;
    cmd.request.x = 1000*pose.position.x;
    cmd.request.y = 1000*pose.position.y;
    cmd.request.z = 1000*pose.position.z;
    cmd.request.a = (180/3.1415)*pose.orientation.x;
    cmd.request.b = (180/3.1415)*pose.orientation.y;
    cmd.request.c = (180/3.1415)*pose.orientation.z;

    return cmd;
}

void dobot_cr5::move_joint()
{
    // for(auto iter = get_param().home_joint_angle.begin(); iter!=get_param().home_joint_angle.end();++iter)
    // {
    //     j_cmd.joint_pos.push_back(*iter);
    //     j_cmd.joint_vel.push_back();  
    // }
}

void dobot_cr5::joystick_callback(const survive_publisher::joystick::ConstPtr msg)
{
    if(get_tf_start_flag())
    {
        // if(msg->press_up == true && msg->press_down == false && cmd.gripper < 5)
        // {
        //     cmd.gripper += 0.2;
        // }
        // else if(msg->press_down == true && msg->press_up == false && cmd.gripper > 0)
        // {
        //     cmd.gripper -= 0.2;
        // }
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
    nh.getParam("/vive/cmd_topic_name", param_lists.cmd_topic_name);
    //nh.getParam("/vive/joint_num", param_lists.joint_num);
    nh.getParam("/vive/init_angle", param_lists.home_joint_angle);
    nh.getParam("/vive/base_x", param_lists.home_tcp.position.x);
    nh.getParam("/vive/base_y", param_lists.home_tcp.position.y);
    nh.getParam("/vive/base_z", param_lists.home_tcp.position.z);
    nh.getParam("/vive/base_roll", param_lists.home_tcp.orientation.x);
    nh.getParam("/vive/base_pitch", param_lists.home_tcp.orientation.y);
    nh.getParam("/vive/base_yaw", param_lists.home_tcp.orientation.z);

    std::shared_ptr<dobot_cr5> node_ptr = std::make_shared<dobot_cr5>(nh,param_lists); 
    
    ros::Rate rate(10);
    while(nh.ok())
    {
        node_ptr->loop_once();
        ros::spinOnce();
        rate.sleep();
    }
}