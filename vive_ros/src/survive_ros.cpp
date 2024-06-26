#include "survive_ros.hpp"


void survive_ros_node::init()
{
    
    ros::NodeHandle nh("~");

    nh.getParam("world_name",world_name);
    nh.getParam("tracker_left",tracker_left);
    nh.getParam("tracker_right", tracker_right);
    nh.getParam("teleop_base", teleop_base);
    nh.getParam("base_station_1", base_station_1);
    nh.getParam("base_station_2", base_station_2);

    joy_sub = nh.subscribe("/libsurvive/LHR_F30CC195/joy",5,&survive_ros_node::joy_topic_callback,this);
    
    // init start flag
    start_teleop = false;
    
    // init tracker base transform
    tracker_static={};
    tracker_static.header.stamp = ros::Time::now();
    tracker_static.header.frame_id = world_name;
    tracker_static.child_frame_id = teleop_base;
    tracker_static.transform.rotation.w = 1;

}
void survive_ros_node::joy_topic_callback(const sensor_msgs::Joy::ConstPtr msg)
{
    if(msg->buttons[3] == 0)
    {
        if(start_teleop == false)
        {
            // switch flag
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
                ROS_INFO("Tracker Base RPY: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
                
                tracker_static.header.stamp = ros::Time::now();
                tracker_static.transform.translation.x = trans.getOrigin().x();
                tracker_static.transform.translation.y = trans.getOrigin().y();
                tracker_static.transform.translation.z = trans.getOrigin().z();
                tracker_static.transform.rotation.x = new_q.x();
                tracker_static.transform.rotation.y = new_q.y();
                tracker_static.transform.rotation.z = new_q.z();
                tracker_static.transform.rotation.w = new_q.w();
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
            }
        }
        else if(start_teleop == true)
            start_teleop = false;
        ROS_INFO("change start flag to %d",start_teleop);
    }
}

void survive_ros_node::check_start_teleop()
{
    if(start_teleop == true)
    {
        // pub tracker static tf
        tracker_static.header.stamp = ros::Time::now();
        broadcaster.sendTransform(tracker_static);
        
        tf::StampedTransform trans;
        try{
            listener.lookupTransform(teleop_base,tracker_left,ros::Time(0),trans);
            // ROS_INFO("Translation: x=%f, y=%f, z=%f", trans.getOrigin().x(), 
            //                 trans.getOrigin().y(), trans.getOrigin().z());
                        
            // get r p y and only use y angle
            double roll, pitch,yaw;
            tf::Quaternion q = trans.getRotation();
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            // ROS_INFO("RPY: roll=%f, pitch=%f, yaw=%f",roll,pitch,yaw);

        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
        }
    }
    else
    {

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
        node_ptr->check_start_teleop();
        ros::spinOnce();
        rate.sleep();
    }
}