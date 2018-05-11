/*
 * Created By: Raad Khan
 * Created On: May 11, 2018
 * Description: A testing node which subscribes to EncoderOdometryNode
 *              and republishes its Odometry message without covariance
 *              Originally designed for use with the Phidget 1047 encoder
 */

#include "EncoderOdometryNode.h"
#include <sb_utils.h>

void encoderOdometryCallback(const std_msgs::String::ConstPtr&);
void republishMsg(std::string);

ros::Publisher encoder_odometry_no_covariance_publisher;

int main (int argc, char **argv) {

    // Setup NodeHandles
    ros::init(argc, argv, "encoder_to_odometry_no_covar");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber
    ros::Subscriber encoder_odometry_subscriber =
            nh.subscribe("/encoders/odom",
                         10,
                         encoderOdometryCallBack);

    // Setup Publisher
    encoder_odometry_no_covariance_publisher =
            private_nh.advertise<nav_msgs::Odometry>("/encoders/odom_no_covariance",
                                                     10);

    ros::spin();

    return 0;
}

void encoderOdometryCallBack(const nav_msgs::Odometry::ConstPtr& subscribed_odom) {

    ROS_INFO("Received Odometry message");

    nav_msgs::Odometry odom_no_covariance;
    odom_no_covariance.header = subscribed_odom->header;
    odom_no_covariance.child_frame_id = subscribed_odom->child_frame_id;
    odom_no_covariance.pose.pose = subscribed_odom->pose.pose;
    odom_no_covariance.twist.twist = subscribed_odom->twist.twist;


    encoder_odometry_no_covariance_publisher.publish(odom_no_covariance);

    ROS_INFO("Published Odometry message with no covariance");
}

