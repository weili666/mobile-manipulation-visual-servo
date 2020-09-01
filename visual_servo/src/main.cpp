#include <Controller.h>
#include <Common.h>

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

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

#include <time.h>
#define M_pi 3.141592653

using namespace std;


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "visual_servo_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    planning_scene_monitor::PlanningSceneMonitorPtr scene_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();
    cout<<"scene set !"<<endl;

    moveit::planning_interface::MoveGroup *robot_arm = new moveit::planning_interface::MoveGroup("ur5_arm");

    //=========================set kdl tree and group========================//
    KDL::Tree kdl_tree;
    cout<<"hello world!!"<<endl;

    string robot_desc_string;
    nh.param("robot_description",robot_desc_string,string());
    if(!kdl_parser::treeFromString(robot_desc_string,kdl_tree))
    {
        cout<<"Failed to construct kdl tree"<<endl;
    }

    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_;

    KDL::Chain jaco_chain;
    if(!kdl_tree.getChain("ur5_arm_base_link", "arm_end_link", jaco_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(jaco_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;
    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;

    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));


    KDL::JntArray  q_;
    KDL::Frame     x_;


    vector<double> joint_now;
    joint_now = robot_arm->getCurrentJointValues();
    double a,b,c,d;
    q_.resize(6);
    q_(0)=joint_now[0];q_(1)=joint_now[1];q_(2)=joint_now[2];
    q_(3)=joint_now[3];q_(4)=joint_now[4];q_(5)=joint_now[5];
    jnt_to_pose_solver_->JntToCart(q_, x_);
    cout<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<endl;
    x_.M.GetQuaternion(a,b,c,d);
    cout<<"qx:"<<a<<" qy:"<<b<<" qz:"<<c<<" qw:"<<d<<endl;
    double qs[4];
    qs[0] = a;
    qs[1] = b;
    qs[2] = c;
    qs[3] = d;
    cv::Mat *mat = new cv::Mat(3, 3, CV_64FC1);
    Quat2RotMat(qs, mat);
    vector<double> eula = RotMat2Eular(mat);
    cout<<"Eula angular:"<<eula[0]<<", "<<eula[1]<<", "<<eula[2]<<endl;

    geometry_msgs::PoseStamped pose = robot_arm->getCurrentPose("arm_end_link");
    cout<<"current joint values are: "<<joint_now[0]<<","<<joint_now[1]<<","<<joint_now[2]<<","<<joint_now[3]<<","<<joint_now[4]<<","<<joint_now[5]<<endl;
    cout<<"pose of arm_end is:x:"<<pose.pose.position.x<<",y:"<<pose.pose.position.y<<",z:"<<pose.pose.position.z<<",qx:"<<pose.pose.orientation.x<<",qy:"<<pose.pose.orientation.y<<",qz:"<<pose.pose.orientation.z<<",qw:"<<pose.pose.orientation.w<<endl;


    //==========================================================================//

    Controller controller_(nh,jnt_to_pose_solver_,jnt_to_jac_solver_,scene_,robot_arm);

    return 0;
}
