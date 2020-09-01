#ifndef _COMMON_H
#define _COMMON_H
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
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
//#include <opencv2/nonfree/nonfree.hpp>

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
//joint state msg related
#include <std_msgs/Float32MultiArray.h>
//KDL related
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

//boost related
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;
using namespace cv;

void Quat2RotMat(double *q, cv::Mat* m);
vector<double> RotMat2Eular(cv::Mat *m);

#endif
