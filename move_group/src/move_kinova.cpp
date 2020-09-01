#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <moveit_msgs/PlanningScene.h>
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
//eigen related
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
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematic_constraints/utils.h>

//tf related
#include <tf/transform_listener.h>

//finger action related
#include <actionlib/client/simple_action_client.h>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_ur5_example");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber jointStateSubscriber;



    std::cout << "Getting the urdf..." << std::endl;
    std::string robot_description;
    std::string robot_param;

    nh.searchParam("robot_description", robot_param);
    nh.param<std::string>(robot_param, robot_description, "");
    //check the param server return
    if (robot_description.empty())
    {
        std::cout << "Failed to retreive robot urdf" << std::endl;
        return 0;
    }
    //parse the KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree))
    {
        std::cout << "Failed to parse the kdl tree" << std::endl;
        return 0;
    }

    //parse the KDL chain
    KDL::Chain jaco_chain;

    if (!kdl_tree.getChain("ur5_arm_base_link", "ur5_arm_wrist_3_link", jaco_chain))  //arm_end_link
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
        return 0;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(jaco_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;

    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    KDL::JntArray  q_;
    KDL::Frame     x_;
    KDL::Jacobian  J_;
    double a,b,c,d;
    q_.resize(6);

    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));

    q_(0)=3.0356; q_(1)=-2.5767;q_(2)=-1.3060;
    q_(3)=-0.8471;q_(4)=-0.8471;q_(5)=-0.8471;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    cout<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<endl;
    x_.M.GetQuaternion(a,b,c,d);
    cout<<"qw:"<<a<<" qx:"<<b<<" qy:"<<c<<" qz:"<<d<<endl;

    moveit::planning_interface::MoveGroup group("ur5_arm");

    //============================ plan 1 ============================//

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x=0;//-0.707*0.577;
    target_pose.orientation.y=0;//-0.707*0.577;
    target_pose.orientation.z=0;//*0.577;
    target_pose.orientation.w=1;
    target_pose.position.x=0.7;
    target_pose.position.y=-0.1;
    target_pose.position.z=0.4;
    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan_r;
    //use 2 seconds to plan puting down the box
    group.setPlanningTime(10.0);
    group.setPlannerId("RRTConnectkConfigDefault");
    bool success_l=group.plan(my_plan_r);
    sleep(3.0);
    ROS_INFO("Visualizing plan (manipulator goal) %s",success_l?"":"FAILED");
    group.execute(my_plan_r);
    cout<<"the size of trajectory:"<<my_plan_r.trajectory_.joint_trajectory.points.size()<<endl;
    for(int i = 0; i < my_plan_r.trajectory_.joint_trajectory.points.size(); i++)
    {
        cout<<"the "<<i<<" th point is: "<<my_plan_r.trajectory_.joint_trajectory.points[i]<<endl;
    }
    for(int i = 0; i < my_plan_r.trajectory_.joint_trajectory.joint_names.size(); i++)
    {
        cout<<"the "<<i<<"th joint's name:"<<my_plan_r.trajectory_.joint_trajectory.joint_names[i];
    }
    cout<<endl;

    //=============================== plan 2 ===========================//

    std::map<std::string,double> joint2;
    joint2["ur5_arm_shoulder_pan_joint"]=2.8238;
    joint2["ur5_arm_shoulder_lift_joint"]=-1.7296;
    joint2["ur5_arm_elbow_joint"]=-0.9883;
    joint2["ur5_arm_wrist_1_joint"]=-1.8708;
    joint2["ur5_arm_wrist_2_joint"]=1.4825;
    joint2["ur5_arm_wrist_3_joint"]=-2.6826;
    moveit::planning_interface::MoveGroup::Plan right_arm_plan;
    group.setJointValueTarget(joint2);
    group.setPlanningTime(10.0);
    group.setPlannerId("RRTConnectkConfigDefault");
    bool success_ri=group.plan(right_arm_plan);
    sleep(3.0);
    ROS_INFO("Visualizing simultanious plan 1st stage %s",success_ri?"":"FAILED");
    group.execute(right_arm_plan);
    cout<<"the size of trajectory:"<<right_arm_plan.trajectory_.joint_trajectory.points.size()<<endl;
    for(int i = 0; i < right_arm_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        cout<<"the "<<i<<" th point is: "<<right_arm_plan.trajectory_.joint_trajectory.points[i]<<endl;
    }
    for(int i = 0; i < right_arm_plan.trajectory_.joint_trajectory.joint_names.size(); i++)
    {
        cout<<"the "<<i<<"th joint's name:"<<right_arm_plan.trajectory_.joint_trajectory.joint_names[i];
    }
    cout<<endl;

    //=============================== plan 3 ===========================//

    double t;
    double T=6.28;
    int N=21;
    int N1=10;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::RobotTrajectory trajectory_msg_;
    trajectory_msgs::JointTrajectory trajectory =  trajectory_msg_.joint_trajectory;
    trajectory.points.resize(N1);
    trajectory.joint_names.push_back("ur5_arm_shoulder_pan_joint");
    trajectory.joint_names.push_back("ur5_arm_shoulder_lift_joint");
    trajectory.joint_names.push_back("ur5_arm_elbow_joint");
    trajectory.joint_names.push_back("ur5_arm_wrist_1_joint");
    trajectory.joint_names.push_back("ur5_arm_wrist_2_joint");
    trajectory.joint_names.push_back("ur5_arm_wrist_3_joint");
    //trajectory.joint_names.push_back("arm_end_joint");

    int num_of_joints=6;
    double A=0.3;
    double B=0.3;
    double C=0.3;
    double w=1;

    for(int i=0;i<N1;i++)
    {
        t=i*T/N;
        trajectory.points[i].positions.resize(num_of_joints);
        trajectory.points[i].velocities.resize(num_of_joints);
        trajectory.points[i].accelerations.resize(num_of_joints);
        trajectory.points[i].positions[0]= 2.8238-0.1*i;
        trajectory.points[i].velocities[0]= 0;
        trajectory.points[i].accelerations[0]= 0;
        trajectory.points[i].positions[1]= 0.1*i-1.7296;
        trajectory.points[i].velocities[1]= 0;
        trajectory.points[i].accelerations[1]= 0;
        trajectory.points[i].positions[2]=  0.05*i-0.9883;
        trajectory.points[i].velocities[2]= 0;
        trajectory.points[i].accelerations[2]= 0;
        trajectory.points[i].positions[3]= -1.8708;
        trajectory.points[i].velocities[3]= 0;
        trajectory.points[i].accelerations[3]= 0;
        trajectory.points[i].positions[4]= 1.4825;
        trajectory.points[i].velocities[4]= 0;
        trajectory.points[i].accelerations[4]= 0;
        trajectory.points[i].positions[5]= -2.6826;
        trajectory.points[i].velocities[5]= 0;
        trajectory.points[i].accelerations[5]= 0;

    }
    moveit::planning_interface::MoveGroup::Plan mani_plan;
    trajectory_msg_.joint_trajectory=trajectory;
    mani_plan.trajectory_=trajectory_msg_;
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start=mani_plan.start_state_;
    display_trajectory.trajectory.push_back(mani_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    bool _return=group.execute(mani_plan);
    ROS_INFO("Visualizing simultanious plan manifold %s",_return?" ":"Failed");
    sleep(5.0);

    //=============================== plan 4 ===========================//

    std::map<std::string,double> joint3;
    joint3["ur5_arm_shoulder_pan_joint"]=3.0356;
    joint3["ur5_arm_shoulder_lift_joint"]=-2.5767;
    joint3["ur5_arm_elbow_joint"]=-1.3060;
    joint3["ur5_arm_wrist_1_joint"]=-0.8471;
    joint3["ur5_arm_wrist_2_joint"]=1.5884;
    joint3["ur5_arm_wrist_3_joint"]=-2.6826;
    moveit::planning_interface::MoveGroup::Plan arm_plan;
    group.setJointValueTarget(joint3);
    group.setPlanningTime(10.0);
    group.setPlannerId("RRTConnectkConfigDefault");
    bool success_=group.plan(arm_plan);
    sleep(3.0);
    ROS_INFO("Visualizing simultanious plan 1st stage %s",success_?"":"FAILED");
    group.execute(arm_plan);
    cout<<"the size of trajectory:"<<arm_plan.trajectory_.joint_trajectory.points.size()<<endl;
    for(int i = 0; i < arm_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        cout<<"the "<<i<<" th point is: "<<arm_plan.trajectory_.joint_trajectory.points[i]<<endl;
    }

    ros::spin();
    return 0;
}
