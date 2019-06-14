/*
 * Created By: Raad Khan
 * Created On: May 25, 2019
 * Description: An RF communication node that receives messages
 * from the client node and returns statuses in response.
 */

#ifndef RF_COMM_SERVER_NODE_H
#define RF_COMM_SERVER_NODE_H

// STD Includes
#include <iostream>

// ROS Includes
#include <std_msgs/String.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class RfCommServer {
public:
    RfCommServer(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void ClientCommandCallBack(const std_msgs::String::ConstPtr& command);
    /**
     * Publishes a given string
     *
     * @param command the string to publish
     */
    void ServerResponseCallBack(std::string command);

    ros::Subscriber client_subscriber;
    ros::Publisher server_publisher;
};
#endif // RF_COMM_SERVER_NODE_H
