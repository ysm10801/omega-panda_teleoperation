#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/GripperCommand.h"
#include "sensor_msgs/Image.h"

class RecorderNode{
public:
    RecorderNode();
    void EEposeCurrentCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void EEposeDesiredCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void GripperCurrentCallback(const control_msgs::GripperCommandConstPtr &msg);
    void GripperDesiredCallback(const control_msgs::GripperCommandConstPtr &msg);
    void FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void FT_Computed_Callback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void HandCameraCallback(const sensor_msgs::ImageConstPtr &msg);
    void FullCameraCallback(const sensor_msgs::ImageConstPtr &msg);

    void PublishSyncData();

private:
    ros::NodeHandle nh;

    ros::Publisher sync_EEpose_obs_pub;
    ros::Publisher sync_EEpose_action_pub;
    ros::Publisher sync_gripper_obs_pub;
    ros::Publisher sync_gripper_action_pub;
    ros::Publisher sync_FT_pub;
    ros::Publisher sync_FT_computed_pub;
    ros::Publisher sync_hand_cam_pub;
    ros::Publisher sync_full_cam_pub;

    ros::Subscriber raw_EEpose_obs_sub;
    ros::Subscriber raw_gripper_obs_sub;
    ros::Subscriber raw_EEpose_action_sub;
    ros::Subscriber raw_gripper_action_sub;
    ros::Subscriber raw_FT_sub;
    ros::Subscriber raw_FT_computed_sub;
    ros::Subscriber raw_hand_cam_sub;
    ros::Subscriber raw_full_cam_sub;

    sensor_msgs::Image hand_cam_img_msg;
    sensor_msgs::Image full_cam_img_msg;
    std_msgs::Float64MultiArray FT_msg;
    std_msgs::Float64MultiArray FT_computed_msg;
    geometry_msgs::PoseStamped EE_pose_msg;
    geometry_msgs::PoseStamped EE_pose_d_msg;
    control_msgs::GripperCommand width_msg;
    control_msgs::GripperCommand width_d_msg;
};

RecorderNode::RecorderNode()
{
    raw_EEpose_obs_sub = nh.subscribe("/ee_pose", 4, &RecorderNode::EEposeCurrentCallback, this);
    raw_EEpose_action_sub = nh.subscribe("/ee_pose_d", 4, &RecorderNode::EEposeDesiredCallback, this);
    raw_gripper_obs_sub = nh.subscribe("/gripper_width_current", 4, &RecorderNode::GripperCurrentCallback, this);
    raw_gripper_action_sub = nh.subscribe("/gripper_width_desired", 4, &RecorderNode::GripperDesiredCallback, this);
    raw_FT_sub = nh.subscribe("/ft_sensor_filtered_value", 4, &RecorderNode::FT_Callback, this);
    raw_FT_computed_sub = nh.subscribe("/tau_A", 4, &RecorderNode::FT_Computed_Callback, this);
    raw_hand_cam_sub = nh.subscribe("/camera_hand/color/image_raw", 1, &RecorderNode::HandCameraCallback, this);
    raw_full_cam_sub = nh.subscribe("/camera_full/color/image_raw", 1, &RecorderNode::FullCameraCallback, this);

    sync_EEpose_obs_pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_sync", 4);
    sync_EEpose_action_pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose_d_sync", 4);
    sync_gripper_obs_pub = nh.advertise<control_msgs::GripperCommand>("/gripper_sync", 4);
    sync_gripper_action_pub = nh.advertise<control_msgs::GripperCommand>("/gripper_d_sync", 4);
    sync_FT_pub = nh.advertise<std_msgs::Float64MultiArray>("/FT_sync", 4);
    sync_FT_computed_pub = nh.advertise<std_msgs::Float64MultiArray>("/FT_computed_sync", 4);
    sync_hand_cam_pub = nh.advertise<sensor_msgs::Image>("/camera_hand_sync", 4);
    sync_full_cam_pub = nh.advertise<sensor_msgs::Image>("/camera_full_sync", 4);
    ROS_INFO("Sync_data_node");
}

void RecorderNode::PublishSyncData(){
    sync_EEpose_obs_pub.publish(EE_pose_msg);
    sync_EEpose_action_pub.publish(EE_pose_d_msg);
    sync_FT_pub.publish(FT_msg);
    sync_FT_computed_pub.publish(FT_computed_msg);
    sync_gripper_obs_pub.publish(width_msg);
    sync_gripper_action_pub.publish(width_d_msg);
    sync_hand_cam_pub.publish(hand_cam_img_msg);
    sync_full_cam_pub.publish(full_cam_img_msg);
    ROS_INFO("Data Published");
}

void RecorderNode::EEposeCurrentCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    EE_pose_msg.pose = msg->pose;
    // ROS_INFO("ee_subed");
}

void RecorderNode::EEposeDesiredCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    EE_pose_d_msg.pose = msg->pose;
    // ROS_INFO("ee_subed");
}

void RecorderNode::GripperCurrentCallback(const control_msgs::GripperCommandConstPtr &msg){
    width_msg.position = msg->position;
    // ROS_INFO("width_subed");
}

void RecorderNode::GripperDesiredCallback(const control_msgs::GripperCommandConstPtr &msg){
    width_d_msg.position = msg->position;
    // ROS_INFO("width_subed");
}

void RecorderNode::FT_Callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    FT_msg.data = msg->data;
    // ROS_INFO("ft_subed");
}

void RecorderNode::FT_Computed_Callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    FT_computed_msg.data = msg->data;
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