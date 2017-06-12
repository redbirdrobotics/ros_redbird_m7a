# Redbird IARC Mission 7a ROS Packages
This repository contains ROS packages for the Redbird Robotics IARC Mission 7a competition entry.

## redbird_m7a_msgs
This package is dedicated to custom message types that are shared among multiple ROS packages.

## redbird_m7a_vehicle
The vehicle node runs onboard the aerial vehicle. It contains flight logic, localization, and simulation nodes.

## redbird_m7a_controlpanel
The control panel node runs on a dedicated ground station and communicates with the aerial vehicle in realtime via ROS topics.
