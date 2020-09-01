#ifndef _CAMERA_H
#define _CAMERA_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <common/mark_vision.h>
#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
using namespace std;
using namespace cv;

class Camera
{
public:
    Camera(ros::NodeHandle& nh, const cv::Mat& cameraMatrix_, const cv::Mat& distCoeffs_);
    ~Camera(){}
    void PicSaveNProcessLeft(const sensor_msgs::Image& msg);
    void PicSaveNProcessRight(const sensor_msgs::Image& msg);
    std::vector<Point> GetCornerFromPic(const cv::Mat& picture_);
    cv::Mat getRotation()
    {
        return R_camera;
    }
    cv::Mat getTranslation()
    {
        return T_camera;
    }
    cv::Mat getCameraSE3()
    {
        return Camera_SE3;
    }
private:
    ros::NodeHandle nh_;
    cv::Mat img;
    cv::Mat img2;
    cv::Mat picture1,picture2;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat cameraMatrixleft, distCoeffsleft;
    cv::Mat cameraMatrixright, distCoeffsright;
    cv::Mat Camera_SE3;
    cv::Mat R_camera;
    cv::Mat T_camera;
    vector<int> vin_size;
    ros::Publisher camera_se3_pub;

};


#endif
