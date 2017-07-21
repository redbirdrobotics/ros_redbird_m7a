/**
 * File: test_receiver.cpp
 *
 * Demonstrates grabbing images from a ROS topic created
 * by the camera_node.
 *
 * Author: Alexander Rickert
 */

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "RedRobot.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Initialize ROS components
    ros::init(argc, argv, "redrobot_node");
    ros::NodeHandle nh;

	RedRobot RRa(0,2);
	RedRobot RRb(1,2);
	RedRobot RRc(2,2);
	RedRobot RRd(3,2);
	RedRobot RRe(4,2);

	RedRobot* RRa_p = &RRa;
	RedRobot* RRb_p = &RRb;
	RedRobot* RRc_p = &RRc;
	RedRobot* RRd_p = &RRd;
	RedRobot* RRe_p = &RRe;

	RedRobot* RRlist[5] = {RRa_p, RRb_p, RRc_p, RRd_p, RRe_p};
	int RRlistSize = RRlist + sizeof(RRlist)/sizeof(RedRobot)

	std::vector<RedRobot*> unfoundList;
	std::vector<RedRobot*> foundList;
	std::vector<vector<int>> data;
	std::vector<KeyPoint> keypoints;

	//Blob Detector
	SimpleBlobDetector detector;
	SimpleBlobdetector* det_p = &detector

	SimpleBlobDetector::Params params;

	params.minThreshold = 0;
	params.maxThreshold = 256;

	params.filterByColor = True;
	params.blobColor = 255;

	params.filterByArea = True;
	params.minArea = 50;
	params.maxArea = 6000;

	params.filterByCircularity = False;
	params.filterByConvexity = False;
	params.filterByInertia = False;

	Ptr<SimpleBlobDetector> detector = SimpleBlobdetector::create(params);
	
}