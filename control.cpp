#include <ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <std_srvs/Trigger.h>


class UR5Move {
public:
    UR5Move(ros::NodeHandle& nh);
    void control();

private:
    bool openGripperCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool closeGripperCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void updateMotion(const ros::TimerEvent&);
    
    void openGripper();
    void closeGripper();


    ros::NodeHandle nodeHandle_;

    ros::ServiceClient dynamixel_client_;
    ros::ServiceClient mission_done_client_;

    ros::ServiceServer openGripperSrv_;
    ros::ServiceServer closeGripperSrv_;
    ros::Time last_update_time_;
    ros::Timer motion_timer_;

    dynamixel_workbench_msgs::DynamixelCommand dynamixel_srv;

    int open_position_;
    int close_position_;

};

UR5Move::UR5Move(ros::NodeHandle& nh)
    : nodeHandle_(nh)
{
   
    dynamixel_client_ = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    mission_done_client_ = nh.serviceClient<std_srvs::Trigger>("/mission_done");

   
    openGripperSrv_ = nodeHandle_.advertiseService("/open_gripper", &UR5Move::openGripperCallback, this);
    closeGripperSrv_ = nodeHandle_.advertiseService("/close_gripper", &UR5Move::closeGripperCallback, this);

    dynamixel_srv.request.command = "";
    dynamixel_srv.request.id = 1;
    dynamixel_srv.request.addr_name="Profile_Velocity";
    dynamixel_srv.request.value = 20;
    dynamixel_client_.call(dynamixel_srv);

    dynamixel_srv.request.command = "";
    dynamixel_srv.request.id = 1;
    dynamixel_srv.request.addr_name = "Goal_Position";

    open_position_ = 900;
    close_position_ = 1450;


}


bool UR5Move::openGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Received request to open gripper.");
    //open_ = true;

    //openGripper();
    dynamixel_srv.request.value = open_position_;
    if (dynamixel_client_.call(dynamixel_srv)) 
    {
        ROS_INFO("Gripper opened successfully.");
    } else {
        ROS_ERROR("Failed to open gripper.");
    }
    
    res.success = true;
    return true;
}

bool UR5Move::closeGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Received request to close gripper.");
    //close_ = true;

    //closeGripper(); 
    dynamixel_srv.request.value = close_position_;
    if (dynamixel_client_.call(dynamixel_srv)) 
    {
        ROS_INFO("Gripper opened successfully.");
    } else {
        ROS_ERROR("Failed to open gripper.");
    }

    res.success = true;
    return true;
}

void UR5Move::control() {
    ros::Rate rate(10); 
    ros::waitForShutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    UR5Move ur5(nh);

    ros::AsyncSpinner spinner(0);  
    spinner.start();  
    
    ur5.control();

    return 0;
}
