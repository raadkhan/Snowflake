/*
 * Created By: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#ifndef LANE_FOLLOW_LANEFOLLOW_H
#define LANE_FOLLOW_LANEFOLLOW_H

// ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Snowbots
#include "LineDetect.h"
#include "IPM.h"
#include "sb_utils.h"

using namespace ros;
using namespace cv;

class LaneFollow {
  public:
    // Constructor
    LaneFollow(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback for the filtered image
     *
     * @param filtered image
     */
    void laneFollowCallback(const sensor_msgs::Image::ConstPtr &filteredImage);

    /**
     * Callback for the green light detection
     *
     * @param filtered image
     */
    void greenLightCallBack(const std_msgs::Bool& green_light_detected);

    // Initializes the corners of the IPM filter
    void IPMFilter(float ipm_base_width,
                   float ipm_top_width,
                   float ipm_base_displacement,
                   float ipm_top_displacement,
                   float image_height,
                   float image_width);

    /**
     * Converts Image to Mat
     *
     * @param Image
     *
     * @return Mat
     */
    cv::Mat rosImageToMat(const sensor_msgs::Image::ConstPtr &image);

    /**
     * Draws windows made by LineDetect
     *
     * @param filtered image
     * @param lane points
     * @param window width
     * @param vertical slices
     *
     * @return vertical slices
     */
    void drawWindows(cv::Mat &filtered_image,
                     std::vector<std::vector<cv::Point2d>> lane_points,
                     int window_width,
                     int vertical_slices);

    /**
     * Converts filtered lane points to perspective lane points
     *
     * @param left and right filtered lane points
     *
     * @return left and right perspective lane points
     */
    std::vector<std::vector<Point2d>>
    getPerspectiveLanePoints(std::vector<std::vector<cv::Point2d>> filtered_lane_points);

    /**
     * Gets the angle of the lane intersect point from the origin point
     *
     * @param lane intersect point
     *
     * @return lane intersect angle
     */
    double getAngleFromOriginToIntersectPoint(cv::Point2d lane_intersect_point);

    // Instantiate LineDetect to generate the lane lines
    LineDetect ld;

    // Traffic light detection subscriber
    ros::Subscriber traffic_light_sub;

    // Filtered image subscriber
    image_transport::Subscriber filtered_image_sub;

    // Image processing pipeline Mat
    cv::Mat filtered_image;

    // Steering driver publisher
    ros::Publisher stay_in_lane_pub;

    // Recommended steer to stay in lane
    geometry_msgs::Twist stay_in_lane;

    // Whether or not we received the first image
    bool receivedFirstImage;

    // Minimum histogram peak values to be considered as lanes
    int min_left_peak;
    int min_right_peak;

    // Origin of the robot
    int origin_point;

    // Angle of destination point to robot
    double lane_intersect_angle;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Speed scalars
    double angular_vel_multiplier;
    double linear_vel_multiplier;

    // IPM filter variables
    float ipm_base_width,
          ipm_top_width,
          ipm_base_displacement,
          ipm_top_displacement;

    // Corners of the portion of the image to be filtered
    int x1, y1;
    int x2, y2;
    int x3, y3;
    int x4, y4;

    // Filters and their variables
    IPM ipm;
    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;

    // Traffic light detection
    int minimum_green_recognised_count;
    int green_count_recognised;
};

#endif