#include "utils.h"

using namespace cv;

Mat makeQMatrix(Point2d image_center,double focal_length, double baseline)
{
   Mat Q=Mat::eye(4,4,CV_64F);
   Q.at<double>(0,3)=-image_center.x;
   Q.at<double>(1,3)=-image_center.y;
   Q.at<double>(2,3)=focal_length;
   Q.at<double>(3,3)=0.0;
   Q.at<double>(2,2)=0.0;
   Q.at<double>(3,2)=1.0/baseline;
 
/*
Q = [ 1 0   0      -Cx      ]
    [ 0 1   0      -Cy      ]
    [ 0 0   0       Fx      ]
    [ 0 0 -1/Tx (Cx-Cx')/Tx ]*/

   return Q;
}

void fromQuaternionToRotationMatrix(double qx,double qy,double qz, double qw, Eigen::Matrix3d &rot)
{
   rot(0,0)=pow(qw,2.0) + pow(qx,2.0) - pow(qy,2.0) - pow(qz,2.0);
   rot(0,1)=2.0*qx*qy - 2.0*qw*qz;
   rot(0,2)=2.0*qx*qz + 2.0*qw*qy;
   rot(1,0)=2.0*qx*qy + 2.0*qw*qz;
   rot(1,1)=pow(qw,2.0) - pow(qx,2.0) + pow(qy,2.0) - pow(qz,2.0);
   rot(1,2)=2.0*qy*qz - 2.0*qw*qx;
   rot(2,0)=2.0*qx*qz - 2.0*qw*qy;
   rot(2,1)=2.0*qy*qz + 2.0*qw*qx;
   rot(2,2)=pow(qw,2.0) - pow(qx,2.0) - pow(qy,2.0) + pow(qz,2.0);
}

bool readFromYamlCameraType(const std::string& filename, std::string& cameraType)
{   
    cv::FileStorage fs(filename.c_str(), CV_STORAGE_READ);
    std::cout << "Calibration file:" << filename << std::endl;
    if(!fs.isOpened())
    {
        std::cout << "Couldn't to open calibration file:" << filename << std::endl;
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        fs["model_type"] >> cameraType;
    }

    fs.release();
    return true;
}

bool loadExtrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1_cv, cv::Mat& R2_cv)
{
   Eigen::Matrix3d rot;
   Eigen::Matrix3d R1, R2;
   cv::FileStorage fs(filename, cv::FileStorage::READ);

   if (!fs.isOpened())
   {
      std::cout<<"error"<<std::endl;
      return false;
   }
   cv::FileNode n = fs["transform"];
   double qx = static_cast<double>(n["q_x"]);
   double qy = static_cast<double>(n["q_y"]);
   double qz = static_cast<double>(n["q_z"]);
   double qw = static_cast<double>(n["q_w"]);
   double tx = static_cast<double>(n["t_x"]);
   double ty = static_cast<double>(n["t_y"]);
   double tz = static_cast<double>(n["t_z"]);
   double baseline = sqrt(tx*tx+ty*ty+tz*tz);

   fromQuaternionToRotationMatrix(qx,qy,qz,qw,rot);

   R1 = rot*0.5;
   R2 = R1.inverse();

   cv::eigen2cv(R1, R1_cv);
   cv::eigen2cv(R2, R2_cv);

   //Rectification matrix
   info_caml.R[0]=R1_cv.at<double>(0,0);
   info_caml.R[1]=R1_cv.at<double>(0,1);
   info_caml.R[2]=R1_cv.at<double>(0,2);
   info_caml.R[3]=R1_cv.at<double>(1,0);
   info_caml.R[4]=R1_cv.at<double>(1,1);
   info_caml.R[5]=R1_cv.at<double>(1,2);
   info_caml.R[6]=R1_cv.at<double>(2,0);
   info_caml.R[7]=R1_cv.at<double>(2,1);
   info_caml.R[8]=R1_cv.at<double>(2,2);

   info_camr.R[0]=R2_cv.at<double>(0,0);
   info_camr.R[1]=R2_cv.at<double>(0,1);
   info_camr.R[2]=R2_cv.at<double>(0,2);
   info_camr.R[3]=R2_cv.at<double>(1,0);
   info_camr.R[4]=R2_cv.at<double>(1,1);
   info_camr.R[5]=R2_cv.at<double>(1,2);
   info_camr.R[6]=R2_cv.at<double>(2,0);
   info_camr.R[7]=R2_cv.at<double>(2,1);
   info_camr.R[8]=R2_cv.at<double>(2,2);

   //Projection camera matrix
   info_camr.P[3]=-info_camr.P[0]*baseline;   //Tx

   fs.release();
   return true;
}

