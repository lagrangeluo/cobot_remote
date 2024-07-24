#include "survive_publisher.hpp"


void survive_ros_node::init()
{
    
    ros::NodeHandle nh;

    nh.getParam("/vive/world_name",world_name);
    nh.getParam("/vive/tracker_left",tracker_left);
    nh.getParam("/vive/tracker_right", tracker_right);
    nh.getParam("/vive/teleop_base", teleop_base);
    nh.getParam("/vive/base_station_1", base_station_1);
    nh.getParam("/vive/base_station_2", base_station_2);
    nh.getParam("/vive/base_x", base_x);
    nh.getParam("/vive/base_y", base_y);
    nh.getParam("/vive/base_z", base_z);
    nh.getParam("/vive/base_roll", base_roll);
    nh.getParam("/vive/base_pitch", base_pitch);
    nh.getParam("/vive/base_yaw", base_yaw);


    joystick_sub = nh.subscribe("/joystick",5,&survive_ros_node::joystick_callback,this);
    
    start_teleop = false;
    // init tracker base transform
    tracker_static={};
    tracker_static.header.stamp = ros::Time::now();
    tracker_static.header.frame_id = world_name;
    tracker_static.child_frame_id = teleop_base;
    tracker_static.transform.rotation.w = 1;
    // broadcaster.sendTransform(tracker_static);

}


void survive_ros_node::joystick_callback(const survive_publisher::joystick::ConstPtr msg)
{
    //如果长按两个按键，更新tracker基座标
    if(msg->press_up_dowm == true)
    {
        if(start_teleop == false)
        {
            start_teleop = true;
            // storage current tf transform
            tf::StampedTransform trans;
            try{
                listener.lookupTransform(world_name,tracker_left,ros::Time(0),trans);
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
                stand_to_base_trans.setOrigin(tf::Vector3(- base_x, - base_y, - base_z)); // 使用 base 相对于 hand 的 x, y, z 位置
                stand_to_base_trans.setRotation(tf::Quaternion(0, 0, 0, 1)); // 如果有旋转，设置相应的旋转四元数

                // world到base
                tf::Transform world_to_base_trans = world_to_stand_trans * stand_to_base_trans;

                tracker_static.header.stamp = ros::Time::now();
                tracker_static.transform.translation.x = world_to_base_trans.getOrigin().x();
                tracker_static.transform.translation.y = world_to_base_trans.getOrigin().y();
                tracker_static.transform.translation.z = world_to_base_trans.getOrigin().z();
                tracker_static.transform.rotation.x = world_to_base_trans.getRotation().x();
                tracker_static.transform.rotation.y = world_to_base_trans.getRotation().y();
                tracker_static.transform.rotation.z = world_to_base_trans.getRotation().z();
                tracker_static.transform.rotation.w = world_to_base_trans.getRotation().w();
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
                start_teleop=false;
                return;
            }
        }
        // pub tracker static tf
        tracker_static.header.stamp = ros::Time::now();
        broadcaster.sendTransform(tracker_static);
    }
    else
    {
        if(start_teleop == true)
        {
            start_teleop = false;
            //
            tf::StampedTransform trans_left;
            try{
                listener.lookupTransform(teleop_base,tracker_left,ros::Time(0),trans_left);
                // 暂停后从上次停止的地方恢复
                base_x = trans_left.getOrigin().x();
                base_y = trans_left.getOrigin().y();
                base_z = trans_left.getOrigin().z();
                ROS_INFO("base x: %f , y: %f , z: %f",base_x,base_y,base_z);
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
                start_teleop=true;
                return;
            }
        }
    }

    //长按js按键，结束模式清除base xyz
    if(msg->press_js == true)
    {
        ros::NodeHandle nh("~");

        //重新获取base xyz
        nh.getParam("/vive/base_x", base_x);
        nh.getParam("/vive/base_y", base_y);
        nh.getParam("/vive/base_z", base_z);
    }
}


int main(int argc, char **argv) {

    ros::init(argc,argv,"survive_teleop");
    ros::NodeHandle nh;

    std::shared_ptr<survive_ros_node> node_ptr = std::make_shared<survive_ros_node>(); 
    node_ptr->init();
    ros::Rate rate(50);

    while(nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}