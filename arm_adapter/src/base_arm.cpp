#include <arm_adapter/base_arm.hpp>


// 发送控制指令
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::publish_cmd(std::string arm_type,arm_cmd_type& cmd)
{
    #ifndef USE_ROS_SERVICE
        if(arm_type == "left")
            cmd_left_pub.publish(cmd);
        if(arm_type == "right")
            cmd_right_pub.publish(cmd);
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

template<typename arm_cmd_type>
bool ArmCommonInterface<arm_cmd_type>::if_left_exist()
{
    if(param_list.cmd_left_name == "" || param_list.left_arm_link == "")
        return false;
    else
        return true;
}

template<typename arm_cmd_type>
bool ArmCommonInterface<arm_cmd_type>::if_right_exist()
{
    if(param_list.cmd_right_name == "" || param_list.right_arm_link == "")
        return false;
    else
        return true;
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
        if(if_left_exist())
            cmd_left_pub = nh.advertise<arm_cmd_type>(param_list.cmd_left_name,5);
        if(if_right_exist())
            cmd_right_pub = nh.advertise<arm_cmd_type>(param_list.cmd_right_name,5);
    #endif
    #ifdef USE_ROS_SERVICE
        cmd_client = nh.serviceClient<arm_cmd_type>(param_list.cmd_left_name);
    #endif
}

// tf变换等比例放大或缩小
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::scale_transform()
{
    //left
    transform_left.tf_trans.transform.translation.x *= param_list.scale_x;
    transform_left.tf_trans.transform.translation.y *= param_list.scale_y;
    transform_left.tf_trans.transform.translation.z *= param_list.scale_z;

    //right
    transform_right.tf_trans.transform.translation.x *= param_list.scale_x;
    transform_right.tf_trans.transform.translation.y *= param_list.scale_y;
    transform_right.tf_trans.transform.translation.z *= param_list.scale_z;
}

// 更新坐标变换
template<typename arm_cmd_type>
void ArmCommonInterface<arm_cmd_type>::update_arm()
{
    ros::Duration timeout(0.1);
    try
    {
        if(if_left_exist())
        {
            ros::Time now = ros::Time::now();
            transform_left.tf_trans = tfBuffer.lookupTransform(param_list.base_link, param_list.left_arm_link, now - timeout,timeout);

            tf2::Quaternion quat_left(
                transform_left.tf_trans.transform.rotation.x,
                transform_left.tf_trans.transform.rotation.y,
                transform_left.tf_trans.transform.rotation.z,
                transform_left.tf_trans.transform.rotation.w);

            tf2::Matrix3x3(quat_left).getRPY(transform_left.roll, transform_left.pitch, transform_left.yaw);
        }

        if(if_right_exist())
        {
            ros::Time now = ros::Time::now();
            transform_right.tf_trans = tfBuffer.lookupTransform(param_list.base_link, param_list.right_arm_link, now - timeout,timeout);

            tf2::Quaternion quat_right(
                transform_right.tf_trans.transform.rotation.x,
                transform_right.tf_trans.transform.rotation.y,
                transform_right.tf_trans.transform.rotation.z,
                transform_right.tf_trans.transform.rotation.w);

            tf2::Matrix3x3(quat_right).getRPY(transform_right.roll, transform_right.pitch, transform_right.yaw);
        }

        //等比例进行放大缩小
        scale_transform()

        //标志位
        tf2_start_flag = true;

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
    double x_unit_left,y_unit_left,z_unit_left,x_unit_right,y_unit_right,z_unit_right;
    uint8_t sample_time = 50;
    ros::Rate rate(50);

    // 如果左手存在
    if(if_left_exist())
    {
        x = transform_left.tf_trans.transform.translation.x;
        y = transform_left.tf_trans.transform.translation.y;
        z = transform_left.tf_trans.transform.translation.z;
        roll = transform_left.roll;
        pitch = transform_left.pitch;
        yaw = transform_left.yaw;

        x_unit_left = (x - param_list.home_tcp.position.x)/sample_time;
        y_unit_left = (y - param_list.home_tcp.position.y)/sample_time;
        z_unit_left = (z - param_list.home_tcp.position.z)/sample_time;
    }
    // 如果右手存在
    if(if_right_exist())
    {
        x = transform_right.tf_trans.transform.translation.x;
        y = transform_right.tf_trans.transform.translation.y;
        z = transform_right.tf_trans.transform.translation.z;
        roll = transform_right.roll;
        pitch = transform_right.pitch;
        yaw = transform_right.yaw;

        x_unit_right = (x - param_list.home_tcp.position.x)/sample_time;
        y_unit_right = (y - param_list.home_tcp.position.y)/sample_time;
        z_unit_right = (z - param_list.home_tcp.position.z)/sample_time;
    }

    // 将参数里的位姿转换成参数里的位姿
    for(int i=sample_time;i >= 0;i--)
    {
        if(if_left_exist())
        {
            geometry_msgs::Pose pose_left;
            pose_left.position.x = param_list.home_tcp.position.x + i * x_unit_left;
            pose_left.position.y = param_list.home_tcp.position.y + i * y_unit_left;
            pose_left.position.z = param_list.home_tcp.position.z + i * z_unit_left;
            pose_left.orientation.x = param_list.home_tcp.orientation.x;
            pose_left.orientation.y = param_list.home_tcp.orientation.y;
            pose_left.orientation.z = param_list.home_tcp.orientation.z;

            arm_cmd_type cmd_left = change_joint_type(pose_left);
            publish_cmd("left",cmd_left);
        }
        if(if_right_exist())
        {
            geometry_msgs::Pose pose_right;
            pose_right.position.x = param_list.home_tcp.position.x + i * x_unit_right;
            pose_right.position.y = param_list.home_tcp.position.y + i * x_unit_right;
            pose_right.position.z = param_list.home_tcp.position.z + i * x_unit_right;
            pose_right.orientation.x = param_list.home_tcp.orientation.x;
            pose_right.orientation.y = param_list.home_tcp.orientation.y;
            pose_right.orientation.z = param_list.home_tcp.orientation.z;

            arm_cmd_type cmd_right = change_joint_type(pose_right);
            publish_cmd("right",cmd_right);
        }
        //下一个插补循环
        rate.sleep();
    }
}