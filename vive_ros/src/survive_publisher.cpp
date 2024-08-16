#include "survive_publisher.hpp"


void survive_ros_node::init()
{
    
    ros::NodeHandle nh;

    nh.getParam("/vive/world_name",world_name);
    nh.getParam("/vive/tracker_left",tracker_left);
    nh.getParam("/vive/tracker_right", tracker_right);
    nh.getParam("/vive/left_hand",left_hand);
    nh.getParam("/vive/right_hand", right_hand);
    nh.getParam("/vive/teleop_base_left", teleop_base_left);
    nh.getParam("/vive/teleop_base_right", teleop_base_right);
    nh.getParam("/vive/base_station_1", base_station_1);
    nh.getParam("/vive/base_station_2", base_station_2);

    nh.getParam("/vive/base_x", base_x_l);
    nh.getParam("/vive/base_y", base_y_l);
    nh.getParam("/vive/base_z", base_z_l);
    nh.getParam("/vive/base_roll", base_roll_l);
    nh.getParam("/vive/base_pitch", base_pitch_l);
    nh.getParam("/vive/base_yaw", base_yaw_l);

    nh.getParam("/vive/base_x", base_x_r);
    nh.getParam("/vive/base_y", base_y_r);
    nh.getParam("/vive/base_z", base_z_r);
    nh.getParam("/vive/base_roll", base_roll_r);
    nh.getParam("/vive/base_pitch", base_pitch_r);
    nh.getParam("/vive/base_yaw", base_yaw_r);

    nh.getParam("/vive/hand_x", hand_x);
    nh.getParam("/vive/hand_y", hand_y);
    nh.getParam("/vive/hand_z", hand_z);
    nh.getParam("/vive/hand_roll", hand_roll);
    nh.getParam("/vive/hand_pitch", hand_pitch);
    nh.getParam("/vive/hand_yaw", hand_yaw);


    joystick_sub = nh.subscribe("/joystick",5,&survive_ros_node::joystick_callback,this);
    
    start_teleop = false;
    start_teleop_state = false;
    // init tracker base transform
    tracker_static_left={};
    tracker_static_left.header.stamp = ros::Time::now();
    tracker_static_left.header.frame_id = world_name;
    tracker_static_left.child_frame_id = teleop_base_left;
    tracker_static_left.transform.rotation.w = 1;
    tracker_static_right={};
    tracker_static_right.header.stamp = ros::Time::now();
    tracker_static_right.header.frame_id = world_name;
    tracker_static_right.child_frame_id = teleop_base_right;
    tracker_static_right.transform.rotation.w = 1;

}


void survive_ros_node::joystick_callback(const survive_publisher::joystick::ConstPtr msg)
{
    //如果长按两个按键，更新tracker基座标
    if(msg->press_up_dowm == true)
    {
        start_teleop_state = true;

    }
    else
    {
        start_teleop_state = false;

    }

    //长按js按键，结束模式清除base xyz
    if(msg->press_js == true)
    {
        ros::NodeHandle nh("~");

        //重新获取base xyz
        nh.getParam("/vive/base_x", base_x_l);
        nh.getParam("/vive/base_y", base_y_l);
        nh.getParam("/vive/base_z", base_z_l);
        nh.getParam("/vive/base_x", base_x_r);
        nh.getParam("/vive/base_y", base_y_r);
        nh.getParam("/vive/base_z", base_z_r);

    }
}

void survive_ros_node::update_hand_frame()
{
    // pub hand to tracker static
    tf::Transform hand_to_tracker;
    tf::Quaternion quaternion;
    quaternion.setRPY(hand_roll,hand_pitch,hand_yaw);
    hand_to_tracker.setOrigin(tf::Vector3(hand_x, hand_y, hand_z));
    hand_to_tracker.setRotation(quaternion);
    broadcaster.sendTransform(tf::StampedTransform(hand_to_tracker, ros::Time::now(), tracker_left, left_hand));
    broadcaster.sendTransform(tf::StampedTransform(hand_to_tracker, ros::Time::now(), tracker_right, right_hand));

    if(start_teleop_state == true)
    {
        if(start_teleop == false)
        {
            start_teleop = true;
            // storage current tf transform
            tf::StampedTransform trans;
            if(if_left_exist())
            {
                try{
                    listener.lookupTransform(world_name,left_hand,ros::Time(0),trans);
                    tf::Quaternion q = trans.getRotation();

                    // get r p y and only use y angle
                    double roll, pitch,yaw;
                    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    roll = 0;
                    pitch = 0;

                    tf::Quaternion new_q;
                    new_q.setRPY(roll, pitch, yaw);
                    //ROS_INFO("Tracker Base RPY: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);

                    // 创建 world 到 stand 的变换
                    tf::Transform world_to_stand_trans,stand_to_base_trans;
                    world_to_stand_trans.setOrigin(trans.getOrigin());
                    world_to_stand_trans.setRotation(new_q);

                    // 创建 stand 到 base 的变换
                    stand_to_base_trans.setOrigin(tf::Vector3(- base_x_l, - base_y_l, - base_z_l)); // 使用 base 相对于 hand 的 x, y, z 位置
                    stand_to_base_trans.setRotation(tf::Quaternion(0, 0, 0, 1)); // 如果有旋转，设置相应的旋转四元数

                    // world到base
                    tf::Transform world_to_base_trans = world_to_stand_trans * stand_to_base_trans;

                    tracker_static_left.transform.translation.x = world_to_base_trans.getOrigin().x();
                    tracker_static_left.transform.translation.y = world_to_base_trans.getOrigin().y();
                    tracker_static_left.transform.translation.z = world_to_base_trans.getOrigin().z();
                    tracker_static_left.transform.rotation.x = world_to_base_trans.getRotation().x();
                    tracker_static_left.transform.rotation.y = world_to_base_trans.getRotation().y();
                    tracker_static_left.transform.rotation.z = world_to_base_trans.getRotation().z();
                    tracker_static_left.transform.rotation.w = world_to_base_trans.getRotation().w();
                }
                catch(tf::TransformException &ex){
                    ROS_ERROR("%s", ex.what());
                    start_teleop=false;
                    return;
                }
            }

            if(if_right_exist())
            {
                try{
                    listener.lookupTransform(world_name,right_hand,ros::Time(0),trans);
                    tf::Quaternion q = trans.getRotation();

                    // get r p y and only use y angle
                    double roll, pitch,yaw;
                    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    roll = 0;
                    pitch = 0;

                    tf::Quaternion new_q;
                    new_q.setRPY(roll, pitch, yaw);
                    //ROS_INFO("Tracker Base RPY: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);

                    // 创建 world 到 stand 的变换
                    tf::Transform world_to_stand_trans,stand_to_base_trans;
                    world_to_stand_trans.setOrigin(trans.getOrigin());
                    world_to_stand_trans.setRotation(new_q);

                    // 创建 stand 到 base 的变换
                    stand_to_base_trans.setOrigin(tf::Vector3(- base_x_r, - base_y_r, - base_z_r)); // 使用 base 相对于 hand 的 x, y, z 位置
                    stand_to_base_trans.setRotation(tf::Quaternion(0, 0, 0, 1)); // 如果有旋转，设置相应的旋转四元数

                    // world到base
                    tf::Transform world_to_base_trans = world_to_stand_trans * stand_to_base_trans;

                    tracker_static_right.transform.translation.x = world_to_base_trans.getOrigin().x();
                    tracker_static_right.transform.translation.y = world_to_base_trans.getOrigin().y();
                    tracker_static_right.transform.translation.z = world_to_base_trans.getOrigin().z();
                    tracker_static_right.transform.rotation.x = world_to_base_trans.getRotation().x();
                    tracker_static_right.transform.rotation.y = world_to_base_trans.getRotation().y();
                    tracker_static_right.transform.rotation.z = world_to_base_trans.getRotation().z();
                    tracker_static_right.transform.rotation.w = world_to_base_trans.getRotation().w();
                }
                catch(tf::TransformException &ex){
                    ROS_ERROR("%s", ex.what());
                    start_teleop=false;
                    return;
                }
            }

        }
        // pub tracker static tf
        tracker_static_left.header.stamp = ros::Time::now();
        broadcaster.sendTransform(tracker_static_left);
        tracker_static_right.header.stamp = ros::Time::now();
        broadcaster.sendTransform(tracker_static_right);

    }
    else
    {
        if(start_teleop == true)
        {
            start_teleop = false;
            //
            tf::StampedTransform trans_left;
            try{
                if(if_left_exist())
                {
                    listener.lookupTransform(teleop_base_left,tracker_left,ros::Time(0),trans_left);
                    // 暂停后从上次停止的地方恢复
                    base_x_l = trans_left.getOrigin().x();
                    base_y_l = trans_left.getOrigin().y();
                    base_z_l = trans_left.getOrigin().z();
                    ROS_INFO("base left x: %f , y: %f , z: %f",base_x_l,base_y_l,base_z_l);
                }
                if(if_right_exist())
                {
                    listener.lookupTransform(teleop_base_right,tracker_right,ros::Time(0),trans_left);
                    // 暂停后从上次停止的地方恢复
                    base_x_r = trans_left.getOrigin().x();
                    base_y_r = trans_left.getOrigin().y();
                    base_z_r = trans_left.getOrigin().z();
                    ROS_INFO("base left x: %f , y: %f , z: %f",base_x_r,base_y_r,base_z_r);
                }

            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
                start_teleop=true;
                return;
            }
        }
    }
}

bool survive_ros_node::if_left_exist()
{
    if(tracker_left=="")
        return false;
    else
        return true;
}

bool survive_ros_node::if_right_exist()
{
    //
    if(tracker_left=="")
        return false;
    else
        return true;
}


int main(int argc, char **argv) {

    ros::init(argc,argv,"survive_teleop");
    ros::NodeHandle nh;

    std::shared_ptr<survive_ros_node> node_ptr = std::make_shared<survive_ros_node>(); 
    node_ptr->init();
    ros::Rate rate(50);

    while(nh.ok())
    {
        node_ptr->update_hand_frame();
        ros::spinOnce();
        rate.sleep();
    }
}