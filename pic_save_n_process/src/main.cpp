#include <Camera.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
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
#define M_pi 3.141592653

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_se3_talker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    int count = 0;
    //===========================INPUT CAMERA MATRIX============================//


    ifstream ifile;
    ifile.open("/home/weili/catkin_ws/src/visual_servo/saved_image/extern_matrix.txt");
    char buffer[256];

    int num=0;
    int num_of_ros = 8;
    vector< vector<char> > data_;
    vector<char> data_iter;
    if(!ifile.is_open())
    {cout << "Error opening file"; exit (1);}
    while(num_of_ros)
    {
        data_iter.clear();
        ifile.getline(buffer,200,'\n');
        for(int i=0;i<256;i++)
        {
            if((buffer[i]=='[')||(buffer[i]==' '))
            {
                continue;
            }
            else if(buffer[i]==',')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                continue;
            }
            else if(buffer[i]==';')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                break;
            }
            else if(buffer[i]==']')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                break;
            }
            else
            {
                data_iter.push_back(buffer[i]);
            }
        }
        num_of_ros--;
    }
    double ddata[14];
    for(int i=0;i<14;i++)
    {
        string data;

        for(int j=0;j<data_[i].size();j++)
        {
            data+=data_[i][j];
        }
        ddata[i]=atof(data.c_str());
        cout<<"ddata"<<i<<":"<<ddata[i]<<endl;
    }

    cv::Mat cameraMatrix(3,3,CV_64FC1);
    cameraMatrix.at<double>(0,0) = ddata[0];
    cameraMatrix.at<double>(0,1) = ddata[1];
    cameraMatrix.at<double>(0,2) = ddata[2];
    cameraMatrix.at<double>(1,0) = ddata[3];
    cameraMatrix.at<double>(1,1) = ddata[4];
    cameraMatrix.at<double>(1,2) = ddata[5];
    cameraMatrix.at<double>(2,0) = ddata[6];
    cameraMatrix.at<double>(2,1) = ddata[7];
    cameraMatrix.at<double>(2,2) = ddata[8];

    cv::Mat distCoeffs(1,5,CV_64F);
    distCoeffs.at<double>(0,0) = ddata[9];
    distCoeffs.at<double>(0,1) = ddata[10];
    distCoeffs.at<double>(0,2) = ddata[11];
    distCoeffs.at<double>(0,3) = ddata[12];
    distCoeffs.at<double>(0,4) = ddata[13];

    cout<<"=================cameraMatrix================"<<endl;
    cout<<cameraMatrix<<endl;
    cout<<"=================distCoeffs=================="<<endl;
    cout<<distCoeffs<<endl;

    Camera camera_(nh, cameraMatrix , distCoeffs);
    return 0;
}
