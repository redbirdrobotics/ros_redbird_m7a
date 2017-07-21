/**
 * File: test_receiver.cpp
 *
 * Demonstrates grabbing images from a ROS topic created
 * by the camera_node.
 *
 * Author: Alex Bennett
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("Image From Topic", cv_bridge::toCvShare(msg, "bgr8")->image);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS components
    ros::init(argc, argv, "test_receiver");
    ros::NodeHandle nh;

    // Create window
    cv::namedWindow("Image From Topic");
    cv::startWindowThread();

    // Create subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/redbird/localization/camera/image", 1, callback);

    // Spin until killed
    ros::spin();

    // Destroy window
    cv::destroyWindow("Image From Topic");
}
