#include <arm_adapter/base_arm.hpp>


int main(int argc, char **argv) {

    ros::init(argc,argv,"base_arm");
    ros::NodeHandle nh;

    // std::shared_ptr<ArmCommonInterface> node_ptr = std::make_shared<ArmCommonInterface>(); 
    
    ros::Rate rate(50);
    while(nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}