/**
 *\file    icars_undistortion_mono_node.cpp
 *\brief   icars_undistortion_mono_node.cpp: class that contains functions to rectify images
 *
 *\author  Michele Olivieri(IRCCyN ECN Nantes)
 *\date    20/04/2016
 */

//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

//ROS topics synchronization
#include <message_filters/subscriber.h>

#include <image_geometry/pinhole_camera_model.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Eigen
#if ROS_INDIGO
  #include <eigen3/Eigen/Dense>
#else
  #include <Eigen/Dense>
#endif

#include "utils.h"
#include "utils_pinhole.h"
#include "utils_kannala.h"
#include "utils_mei.h"

using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;

static string enc = sensor_msgs::image_encodings::BGR8;

//Global variables
sensor_msgs::CameraInfo info_cam;//Information of the camera (image size, intrinsic parameters)
string fileNameCalibration;//The name of the calibration file (especified as an input parameter)
bool publishRectified=true;//Whether to publish the rectified image or not
bool sizeInit=true;//Whether the size must be initialized

string camFrame;//Frame names

//Publishers
ros::Publisher pub_img_rectified;;
ros::Publisher pub_info;

//for undistort images
Mat mapRect1,mapRect2;

//Publish the rectified image in grey
void publishRectifiedImage(const Mat &img,ros::Time tstamp, string frame_id);

//Image processing callback
/** \brief callback to catch the input images in ROS and process them
  *
  */
//void dataCallback( const sensor_msgs::ImageConstPtr& image_msg, const icarsPathFollower::StatusConstPtr& pf_stat_msg)
void dataCallback( const sensor_msgs::ImageConstPtr& image_msg)
{
  ros::Time tstamp;
  cv::Mat imageRectified;

  //Get timestamp
  tstamp = image_msg->header.stamp;

  //get distorted img in OpenCV format
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(image_msg, enc);

  // undistortion
  remap(cv_ptr->image, imageRectified, mapRect1, mapRect2, INTER_LINEAR);

  if (publishRectified)//Publish the composed rectified image
  {
      publishRectifiedImage(imageRectified, image_msg->header.stamp, image_msg->header.frame_id);
      // DEBUG
      // publishRectifiedImage(cv_ptr->image, image_msg->header.stamp, image_msg->header.frame_id);
  }

  //Publish the camera info
  info_cam.header.stamp = image_msg->header.stamp;
  pub_info.publish(info_cam);
}

void publishRectifiedImage(const Mat &img, ros::Time tstamp, string frame_id)
{
    //Publish the image with features
    cv_bridge::CvImage out_msg;

    out_msg.encoding = enc;
    out_msg.image    = img;
    out_msg.header.seq = 1;
    out_msg.header.frame_id = frame_id;

    out_msg.header.stamp = tstamp;
    pub_img_rectified.publish(out_msg.toImageMsg());
}

/** \brief the main function of the program
  *
  */
int main(int argc, char **argv)
{    
  ros::init(argc, argv, "icars_undistortion_color");

  ros::NodeHandle local_nh("~");
  ros::NodeHandle global_nh("");

  //Load local parameters
  local_nh.getParam("Publishrectified", publishRectified);//Flag for debugging and/or visualization
  local_nh.getParam("PathXmlCalibration", fileNameCalibration);//The path of the calibration file

  string cameraType;

  readFromYamlCameraType(fileNameCalibration, cameraType);

   if(cameraType=="KANNALA_BRANDT")
   {
      KANNALA::parameters param;
      //Load the calibration parameters of the camera
      if (KANNALA::loadIntrinsicFromYAML(fileNameCalibration, info_cam, param)==false)
      {
         cout << "Failed to open file:" << fileNameCalibration << endl;
         exit(0);
      }

      initUndistortRectifyMap(mapRect1,mapRect2,cv::Mat::eye(3, 3, CV_32F), param);
   }
   else if(cameraType=="MEI")
   {
      MEI::parameters param;
      //Load the calibration parameters of the camera
      if (MEI::loadIntrinsicFromYAML(fileNameCalibration, info_cam, param)==false)
      {
         cout << "Failed to open file:" << fileNameCalibration << endl;
         exit(0);
      }

      initUndistortRectifyMap(mapRect1,mapRect2,cv::Mat::eye(3, 3, CV_32F), param);
   }
   else if(cameraType=="PINHOLE")
   {
      PINHOLE::parameters param;
      //Load the calibration parameters of the camera
      if (PINHOLE::loadIntrinsicFromYAML(fileNameCalibration, info_cam, param)==false)
      {
         cout << "Failed to open file:" << fileNameCalibration << endl;
         exit(0);
      }
      initUndistortRectifyMap(mapRect1,mapRect2,cv::Mat::eye(3, 3, CV_32F), param);
   }
   else 
   {
      cout << "Unknow type of camera" << endl;
      exit(0);
   }

  // Read global parameters
  string camFrameParamName;

  local_nh.getParam("CameraFrameParamName", camFrameParamName);//The name of the camera frame
  global_nh.getParam(camFrameParamName, camFrame);//The name of the camera frame
  cout << camFrameParamName << camFrame << endl;

  //Publishing
  if (publishRectified)//Maybe the rectified image is needed for debugging purposes (It is not necessary for the visual odometry anymore)
  {
      pub_img_rectified = local_nh.advertise<sensor_msgs::Image>("/imageRectified", 1);
  }
  pub_info = local_nh.advertise<sensor_msgs::CameraInfo>("/camInfo", 1);//Camera info publisher

  //Subscriptions to images
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber image_sub_;
  image_sub_ = it.subscribe("/image", 1, dataCallback);

  //Listen for topics
  ros::spin();

  return 0;
}

