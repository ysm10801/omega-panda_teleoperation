#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/GripperCommand.h"
#include "sensor_msgs/Image.h"

class RecorderSubNode{
public:
    RecorderSubNode();
    void EEposeCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void GripperCallback(const control_msgs::GripperCommandConstPtr &msg);
    void FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void HandCameraCallback(const sensor_msgs::ImageConstPtr &msg);
    void FullCameraCallback(const sensor_msgs::ImageConstPtr &msg);

    void PublishSyncData();

private:
    ros::NodeHandle nh;

    ros::Subscriber sync_EEpose_sub;
    ros::Subscriber sync_gripper_sub;
    ros::Subscriber sync_FT_sub;
    ros::Subscriber sync_hand_cam_sub;
    ros::Subscriber sync_full_cam_sub;

    sensor_msgs::Image hand_cam_img_msg;
    sensor_msgs::Image full_cam_img_msg;
    std_msgs::Float64MultiArray FT_msg;
    geometry_msgs::PoseStamped EE_pose_msg;
    control_msgs::GripperCommand width_msg;
};

RecorderSubNode::RecorderSubNode()
{
    sync_EEpose_sub = nh.subscribe("/ee_pose_sync", 4, &RecorderSubNode::EEposeCallback, this);
    sync_gripper_sub = nh.subscribe("/gripper_sync", 4, &RecorderSubNode::GripperCallback, this);
    sync_FT_sub = nh.subscribe("/FT_sync", 4, &RecorderSubNode::FT_Callback, this);
    sync_hand_cam_sub = nh.subscribe("/camera_hand_sync", 1, &RecorderSubNode::HandCameraCallback, this);
    sync_full_cam_sub = nh.subscribe("/camera_full_sync", 1, &RecorderSubNode::FullCameraCallback, this);

    ROS_INFO("Sync_data_node");
}

void RecorderSubNode::EEposeCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    EE_pose_msg.pose = msg->pose;
    ROS_INFO("ee_subed");
}

void RecorderSubNode::GripperCallback(const control_msgs::GripperCommandConstPtr &msg){
    width_msg.position = msg->position;
    ROS_INFO("width_subed");
}

void RecorderSubNode::FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    FT_msg.data = msg->data;
    ROS_INFO("ft_subed");
}

void RecorderSubNode::HandCameraCallback(const sensor_msgs::ImageConstPtr &msg){
    hand_cam_img_msg.data = msg->data;
    ROS_INFO("hcam_subed");
}

void RecorderSubNode::FullCameraCallback(const sensor_msgs::ImageConstPtr &msg){
    full_cam_img_msg.data = msg->data;
    ROS_INFO("fcam_subed");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_sub_node");

    RecorderSubNode r;
    ros::Rate loop_rate(30);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}