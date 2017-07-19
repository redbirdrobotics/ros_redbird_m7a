/**
 * File: camera_node.cpp
 *
 * Handles grabbing images from the camera and publishing
 * to the /redbird/localization/camera/image topic.
 *
 * Author: Alex Bennett
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char** argv)
{
    // Initialize ROS components
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // Create image transporter
    image_transport::ImageTransport it(nh);

    // Create publisher
    image_transport::Publisher pub = it.advertise("/redbird/localization/camera/image", 10000);

    // Create capture pipeline
    const char* capture_pipe = "v4l2src device=/dev/video0 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)I420 ! videoconvert ! appsink";
    cv::VideoCapture cap(capture_pipe);

    // Create rate
    ros::Rate rate(60); // Should match expected FPS

    // Check that capture is opened
    if(!cap.isOpened())
    {
        ROS_ERROR("Could not open camera stream");
        return -1;
    }

    // Enter main loop
    while(nh.ok())
    {
        // Create frame matrix
        cv::Mat frame;

        // Read
        bool success = cap.read(frame);

        // If there was an error, log it
        if(!success) ROS_WARN("Frame skipped");

        // Publish
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

        // Keep desired rate
        rate.sleep();
    }
}
