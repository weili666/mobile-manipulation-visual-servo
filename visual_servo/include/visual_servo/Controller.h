#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <common/mark_vision.h>
#include <sensor_msgs/Image.h>
#include <robotiq_85_msgs/GripperCmd.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <time.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <nav_msgs/Odometry.h>
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
#include <common/mark_vision.h>
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

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
using namespace std;
using namespace cv;

class Controller
{
public:
    Controller(const ros::NodeHandle& node_handle, boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver_, boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_, planning_scene_monitor::PlanningSceneMonitorPtr sc_,moveit::planning_interface::MoveGroup* ro_arm_);
    ~Controller(){}

    cv::Mat getVControl();
    cv::Mat getDeltaX();
    void base_Velocity_Save(const nav_msgs::Odometry& base_velocity);
    void arm_Velocity_Save(const sensor_msgs::JointState& arm_velocity);
    void camera_SE3_Save(const common::mark_vision &mark_msgs);
private:
    double alpha;
    double D;
    double K;
    double Lambda;
    double a, b, c;
    double S1, S2, S3, S4;
    double theta_x, theta_y, theta_z;
    double delta_t;
    ros::NodeHandle nh;
    cv::Mat camera_SE3;
    cv::Mat arm_end_2_camera;
    cv::Mat arm_base_2_arm_end;
    cv::Mat arm_base_2_camera;
    cv::Mat world_2_arm_base;
    cv::Mat world_2_arm_end;
    cv::Mat Jacobi;
    cv::Mat delta_X;
    cv::Mat V_control;
    vector<cv::Mat> V_arm;
    vector<cv::Mat> V_base;
    cv::Mat V_real;
    cv::Mat Q;
    cv::Mat R;
    cv::Mat A;
    cv::Mat B;
    cv::Mat H;
    cv::Mat Kalman;
    int stage;
    clock_t startTime,endTime;
    vector<cv::Mat> X_post_vec;
    vector<cv::Mat> P_post_vec;
    vector<double> t_vec;
    vector<double> thetas_now;
    vector<double> position_now;
    vector<double> thetas_target;
    vector<double> position_target;
    cv::Mat R_target;
    vector<clock_t> start_Time_vec;
    vector<cv::Mat> camera_SE3_vec;
    vector<cv::Mat> V_control_vec;
    vector<cv::Mat> dX_vec;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver;
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver;
    planning_scene_monitor::PlanningSceneMonitorPtr scene;
    moveit::planning_interface::MoveGroup* robot_arm;
    ros::Subscriber camera_se3_sub;
    double distance;
    double distance_thread;
    ros::Publisher speed_pub_ ;
    ros::Publisher bulldog_pub_;
    ros::Publisher gripper_pub_;
};

#endif
