/*
 * Created By: Raad Khan
 * Created On: May 25, 2019
 * Description: An RF communication node that receives messages
 * from the client node and returns statuses in response.
 */

#include <RfCommServerNode.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "rf_comm_server_node";

    // Create an instance of your class
    RfCommServer rf_comm_server(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}