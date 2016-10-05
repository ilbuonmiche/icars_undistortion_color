#ifndef UTILS_H
#define UTILs_H

#include <cmath>
#include <cstdio>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//Eigen
#if ROS_INDIGO
  #include <eigen3/Eigen/Eigen> 
#else
  #include <Eigen/Eigen>
#endif

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

bool loadExtrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1_cv, cv::Mat& R2_cv);

bool readFromYamlCameraType(const std::string& filename, std::string& cameraType);

void fromQuaternionToRotationMatrix(double qx,double qy,double qz, double qw, Eigen::Matrix3d &rot);

cv::Mat makeQMatrix(cv::Point2d image_center,double focal_length, double baseline);

#endif
