#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/GripperCommand.h"
#include "sensor_msgs/Image.h"

class RecorderNode{
public:
    RecorderNode();
    void EEposeCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void GripperCallback(const control_msgs::GripperCommandConstPtr &msg);
    void FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void FT_MedianFiltering();
    void HandCameraCallback(const sensor_msgs::ImageConstPtr &msg);
    void FullCameraCallback(const sensor_msgs::ImageConstPtr &msg);

    void PublishSyncData();

private:
    ros::NodeHandle nh;

    ros::Publisher sync_EEpose_pub;
    ros::Publisher sync_gripper_pub;
    ros::Publisher sync_FT_pub;
    ros::Publisher sync_hand_cam_pub;
    ros::Publisher sync_full_cam_pub;

    ros::Subscriber raw_EEpose_sub;
    ros::Subscriber raw_gripper_sub;
    ros::Subscriber raw_FT_sub;
    ros::Subscriber raw_hand_cam_sub;
    ros::Subscriber raw_full_cam_sub;

    sensor_msgs::Image hand_cam_img_msg;
    sensor_msgs::Image full_cam_img_msg;
    std_msgs::Float64MultiArray FT_msg;
    geometry_msgs::PoseStamped EE_pose_msg;
    control_msgs::GripperCommand width_msg;
};

RecorderNode::RecorderNode()
{
    raw_EEpose_sub = nh.subscribe("/ee_pose_d", 4, &RecorderNode::EEposeCallback, this);
    raw_gripper_sub = nh.subscribe("/topic_gripper_width", 4, &RecorderNode::GripperCallback, this);
    raw_FT_sub = nh.subscribe("/ft_sensor_value", 4, &RecorderNode::FT_Callback, this);
    raw_hand_cam_sub = nh.subscribe("/camera_hand/color/image_raw", 1, &RecorderNode::HandCameraCallback, this);
    raw_full_cam_sub = nh.subscribe("/camera_full/color/image_raw", 1, &RecorderNode::FullCameraCallback, this);

    sync_EEpose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_sync", 4);
    sync_gripper_pub = nh.advertise<control_msgs::GripperCommand>("/gripper_sync", 4);
    sync_FT_pub = nh.advertise<std_msgs::Float64MultiArray>("/FT_sync", 4);
    sync_hand_cam_pub = nh.advertise<sensor_msgs::Image>("/camera_hand_sync", 4);
    sync_full_cam_pub = nh.advertise<sensor_msgs::Image>("/camera_full_sync", 4);
    ROS_INFO("Sync_data_node");
}

void RecorderNode::PublishSyncData(){
    sync_EEpose_pub.publish(EE_pose_msg);
    sync_FT_pub.publish(FT_msg);
    sync_gripper_pub.publish(width_msg);
    sync_hand_cam_pub.publish(hand_cam_img_msg);
    sync_full_cam_pub.publish(full_cam_img_msg);
    ROS_INFO("Data Published");
}

void RecorderNode::EEposeCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    EE_pose_msg.pose = msg->pose;
    // ROS_INFO("ee_subed");
}

void RecorderNode::GripperCallback(const control_msgs::GripperCommandConstPtr &msg){
    width_msg.position = msg->position;
    // ROS_INFO("width_subed");
}

void RecorderNode::FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    FT_msg.data = msg->data;
    // ROS_INFO("ft_subed");
}

void RecorderNode::HandCameraCallback(const sensor_msgs::ImageConstPtr &msg){
    hand_cam_img_msg.data = msg->data;
    // ROS_INFO("hcam_subed");
}

void RecorderNode::FullCameraCallback(const sensor_msgs::ImageConstPtr &msg){
    full_cam_img_msg.data = msg->data;
    // ROS_INFO("fcam_subed");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_node");

    RecorderNode r;
    ros::Rate loop_rate(30);

    while(ros::ok()){
        r.PublishSyncData();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}