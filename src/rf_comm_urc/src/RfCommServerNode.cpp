#include <utility>

/*
 * Created By: Raad Khan
 * Created On: May 25, 2019
 * Description: An RF communication node that receives messages
 * from the client node and returns statuses in response.
 */

#include <RfCommServerNode.h>

RfCommServer::RfCommServer(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "client";
    int queue_size                    = 10;
    client_subscriber                     = nh.subscribe(
            topic_to_subscribe_to, queue_size, &RfCommServer::ClientCommandCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("server");
    queue_size        = 1;
    server_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void RfCommServer::ClientCommandCallBack(const std_msgs::String::ConstPtr& command) {
    ROS_INFO("Received message");
    ServerResponseCallBack(command->data);
}

void RfCommServer::ServerResponseCallBack(std::string command) {
    std_msgs::String response;
    response.data = std::move(command);
    server_publisher.publish(response);
    ROS_INFO("Published message");
}
