//=================================================================================================
// Copyright (c) 2014, Vibekananda Dutta, WUT
// All rights reserved.

//=================================================================================================

#ifndef _HECTOR_MOTION_DETECTION_H_
#define _HECTOR_MOTION_DETECTION_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <time.h>
#include <geometry_msgs/Point.h>
//#include "duration.h"
#include <dirent.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <tinyxml.h>
#include <fstream>  
#include <string.h>  
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <hector_motion_detection/MotionDetectionConfig.h>
#include <std_msgs/Int32.h>
#include<iostream>

#define dZ0 450
#define alfa 40
#define h 310
#define d 50
#define PI 3.14159265

using hector_motion_detection::MotionDetectionConfig;

class MotionDetection{
public:
    MotionDetection();
    ~MotionDetection();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& img); //, const sensor_msgs::CameraInfoConstPtr& info);
    //void mappingCallback(const thermaleye_msgs::Mapping& mapping);
    void directoryExistsOrCreate(const char* pzPath);
    bool saveImg(cv::Mat image, const std::string DIRECTORY, const std::string EXTENSION, const char * DIR_FORMAT, const char * FILE_FORMAT);
    
    void parseRegionXML(std::string file_region, std::vector<cv::Point2f> &region);
    void dynRecParamCallback(MotionDetectionConfig &config, uint32_t level);

    ros::Publisher image_percept_pub_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::CameraPublisher image_motion_pub_;
    image_transport::CameraPublisher image_detected_pub_;

    image_transport::Subscriber image_sub_;

    dynamic_reconfigure::Server<MotionDetectionConfig> dyn_rec_server_;

    cv_bridge::CvImageConstPtr img_prev_ptr_;
    cv_bridge::CvImageConstPtr img_current_ptr_;
    cv_bridge::CvImageConstPtr img_current_col_ptr_;

    int motion_detect_threshold_;
    double min_percept_size, max_percept_size;
    double min_density;
    std::string percept_class_id_;

};

#endif
