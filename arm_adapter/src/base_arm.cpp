#include <arm_adapter/base_arm.hpp>


// 发送控制指令
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::publish_cmd(arm_cmd_type& cmd)
{
    #ifndef USE_ROS_SERVICE
        cmd_pub.publish(cmd);
    #endif
    #ifdef USE_ROS_SERVICE
        cmd_client.call(cmd);
    #endif
}


// 一次循环
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::loop_once()
{
    update_arm();
    publish_pose();
}

// 初始化ros接口
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::init_interface()
{
    // 初始化tf2
    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

    // 初始化摇杆话题接收者
    joystick_sub = nh.subscribe<survive_publisher::joystick>
        (param_list.joystick_topic,5,std::bind(&ArmCommonInterface::joystick_callback, this, std::placeholders::_1));

    #ifndef USE_ROS_SERVICE
        cmd_pub = nh.advertise<arm_cmd_type>(param_list.cmd_topic_name,5);
    #endif
    #ifdef USE_ROS_SERVICE
        cmd_client = nh.serviceClient<arm_cmd_type>(param_list.cmd_topic_name);
    #endif
}

// 更新坐标变换
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::update_arm()
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


// 机械臂移动到初始状态
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::move_to_homepose()
{
    double x,y,z,roll,pitch,yaw;
    uint8_t sample_time = 50;
    ros::Rate rate(50);

    x = transform.tf_trans.transform.translation.x;
    y = transform.tf_trans.transform.translation.y;
    z = transform.tf_trans.transform.translation.z;
    roll = transform.roll;
    pitch = transform.pitch;
    yaw = transform.yaw;

    double x_unit = (x - param_list.home_tcp.position.x)/sample_time;
    double y_unit = (y - param_list.home_tcp.position.y)/sample_time;
    double z_unit = (z - param_list.home_tcp.position.z)/sample_time;

    // 将参数里的位姿转换成参数里的位姿
    for(int i=sample_time;i >= 0;i--)
    {
        geometry_msgs::Pose pose;
        pose.position.x = param_list.home_tcp.position.x + i * x_unit;
        pose.position.y = param_list.home_tcp.position.y + i * y_unit;
        pose.position.z = param_list.home_tcp.position.z + i * z_unit;
        pose.orientation.x = param_list.home_tcp.orientation.x;
        pose.orientation.y = param_list.home_tcp.orientation.y;
        pose.orientation.z = param_list.home_tcp.orientation.z;

        arm_cmd_type cmd = change_joint_type(pose);
        publish_cmd(cmd);
        rate.sleep();
    }
}