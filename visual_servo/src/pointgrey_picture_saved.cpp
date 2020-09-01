#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <ctime>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <math.h>
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

using namespace std;
int num=0;
class ArmJoints
{
public:
    ArmJoints()
    {
        joint1=0;joint2=M_PI;joint3=M_PI;joint4=0;joint5=0;joint6=0;
    }
    ArmJoints(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6):joint1(jo1),joint2(jo2),joint3(jo3),joint4(jo4),joint5(jo5),joint6(jo6){}
    ArmJoints(const ArmJoints& aj)
    {
        joint1=aj.joint1;joint2=aj.joint2;joint3=aj.joint3;joint4=aj.joint4;joint5=aj.joint5;joint6=aj.joint6;
    }
    void setJoint(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6)
    {
        joint1=jo1;joint2=jo2;joint3=jo3;joint4=jo4;joint5=jo5;joint6=jo6;
    }
    double joint1,joint2,joint3,joint4,joint5,joint6;

};
class ArmJointsSaved
{
public:
    ArmJointsSaved(ros::NodeHandle& nh):nh_(nh)
    {
        ros::Subscriber arm_sub = nh_.subscribe("/ur5_arm/joint_states",1,& ArmJointsSaved::ArmSavedCB,this);

        ros::Rate loop_rate(1);
        int num_loop = 2;
        while(num_loop)
        {
            ros::spinOnce();
            loop_rate.sleep();
            num_loop--;
        }
    }
    void ArmSavedCB(const sensor_msgs::JointState& msg);
private:
    ros::NodeHandle nh_;

};
void ArmJointsSaved::ArmSavedCB(const sensor_msgs::JointState& msg)
{
   double joint[6];

    double a,b,c,d;

    joint[0] = msg.position[0]; joint[1] = msg.position[1]; joint[2] = msg.position[2];
    joint[3] = msg.position[3]; joint[4] = msg.position[4]; joint[5] = msg.position[5];

    ofstream file;
    file.open("/home/weili/catkin_ws/src/visual_servo/saved_image/grasp_pa.txt",ios::app|ios::out);

    file<<"joints:"<<joint[0]<<" "<<joint[1]<<" "<<joint[2]<<" "<<joint[3]<<" "<<joint[4]<<" "<<joint[5]<<";"<<endl;

}
class PictureSaved
{
public:
    PictureSaved(ros::NodeHandle& nh):nh_(nh)
    {
        num++;
        cout<<"begin save picture: "<<num<<endl;
        ros::Subscriber pic_sub_l = nh_.subscribe("/camera/left/image_raw", 1, & PictureSaved::PicSavedCBLeft,this);
        ros::Subscriber pic_sub_r = nh_.subscribe("/camera/right/image_raw", 1, & PictureSaved::PicSavedCBRight,this);
        ros::Rate loop_rate(1);
        int num_loop = 5;
        while(num_loop)
        {
            cout<<"hello world!!"<<endl;
            ros::spinOnce();
            loop_rate.sleep();
            num_loop--;
        }
    }
    void PicSavedCBLeft(const sensor_msgs::Image& msg);
    void PicSavedCBRight(const sensor_msgs::Image& msg);
private:
    ros::NodeHandle nh_;

};
void PictureSaved::PicSavedCBLeft(const sensor_msgs::Image& msg)
{
    cout<<"begin save!"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);

    char buff[100];
    sprintf(buff, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_left_%d.jpg",num);

    cout<<"======================saved picture num:"<<num<<"======================="<<endl;
    cout<<"saved name as:"<<buff<<endl;
    cv::imwrite(buff, image_gray);

}
void PictureSaved::PicSavedCBRight(const sensor_msgs::Image& msg)
{
    cout<<"begin save right!"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);

    char buff[100];
    sprintf(buff, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_right_%d.jpg",num);

    cout<<"======================saved picture num:"<<num<<"======================="<<endl;
    cout<<"saved name as:"<<buff<<endl;
    cv::imwrite(buff, image_gray);

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointgrey_camera_picture");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    ros::Rate loop_rate(10);

    planning_scene_monitor::PlanningSceneMonitorPtr scene_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    cout<<"hello world!"<<endl;
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();
    cout<<"scene set !"<<endl;
    moveit::planning_interface::MoveGroup robot_arm("ur5_arm");

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
    boost::shared_ptr<KDL::ChainIdSolver>        pose_to_jnt_solver;

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
   // pose_to_jnt_solver.reset(new KDL::ChainIdSolver(jaco_chain));



    KDL::JntArray  q_;
    KDL::Frame     x_;

    double a,b,c,d;
    q_.resize(6);
    q_(0)=-1.4735840227881702;q_(1)=2.919063173960516;q_(2)=1.0135101613037494;
    q_(3)=-2.0836849337533323;q_(4)=1.443466866652682;q_(5)=1.3149469735032022;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    cout<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<endl;
    x_.M.GetQuaternion(a,b,c,d);
    cout<<"qx:"<<a<<" qy:"<<b<<" qz:"<<c<<" qw:"<<d<<endl;

    //==========================================================================//

    //========================collect data for hand-eye-calibration=====================//
    ArmJoints arm(-0.1607306639300745, -2.1802433172809046, 1.6912956237792969, -2.2754295508014124, -1.4598348776446741, 2.9998998641967773);

    vector<ArmJoints> arm_joints;
    ArmJoints arm_joint_1(-0.1607306639300745, -2.1802433172809046, 1.6912956237792969, -2.2754295508014124, -1.4598348776446741, 2.9998998641967773);
    arm_joints.push_back(arm_joint_1);

    ArmJoints arm_joint_2(-0.12525874773134404, -2.1207502524005335, 1.5557456016540527, -2.0407002607928675, -1.5142181555377405, 3.0021016597747803);
    arm_joints.push_back(arm_joint_2);

    ArmJoints arm_joint_3(-0.4030244986163538, -2.086653534566061, 1.6284666061401367, -2.0609362761126917, -1.0086348692523401, 2.921243667602539);
    arm_joints.push_back(arm_joint_3);

    ArmJoints arm_joint_4(-0.5768902937518519, -2.079992119465963, 1.616279125213623, -1.8559940497027796, -0.6678784529315394, 2.7017436027526855);
    arm_joints.push_back(arm_joint_4);

    ArmJoints arm_joint_5(-2.1470845381366175, -1.1764467398272913, -1.4225948492633265, -1.8929031530963343, 0.5076626539230347, -2.033034149800436);
    arm_joints.push_back(arm_joint_5);

    ArmJoints arm_joint_6(-2.001197640095846, -1.327143971120016, -1.2689531485186976, -2.336012665425436, 0.5029737949371338, -1.5925424734698694);
    arm_joints.push_back(arm_joint_6);

    ArmJoints arm_joint_7(-2.3611162344561976, -1.237537686024801, -1.1251853148089808, -1.6050623098956507, 0.9346908926963806, -2.601253096257345);
    arm_joints.push_back(arm_joint_7);

    ArmJoints arm_joint_8(-2.864800755177633, -2.22243577638735, 1.0685582160949707, -2.605492893849508, 1.6004412174224854, -3.0039570967303675);
    arm_joints.push_back(arm_joint_8);

    ArmJoints arm_joint_9(0.12471608817577362, -2.199698273335592, 2.4104790687561035, -3.123063389454977, -1.7464912573443812, -3.0321810881244105);
    arm_joints.push_back(arm_joint_9);

    ArmJoints arm_joint_10(0.7929587364196777, -1.6859071890460413, 2.1689743995666504, -3.1341550985919397, -2.6243858973132532, -2.6974273363696497);
    arm_joints.push_back(arm_joint_10);

    ArmJoints arm_joint_11(-1.2973998228656214, -1.958515469227926, 2.1733450889587402, -0.6399114767657679, -0.115852181111471, 0.8666961789131165);
    arm_joints.push_back(arm_joint_11);


    vector<ArmJoints>::iterator aj_iter;
    for(aj_iter=arm_joints.begin();aj_iter!=arm_joints.end();aj_iter++)
    {
        ArmJoints aj=*aj_iter;

        double a,b,c,d;

        std::map<std::string,double> joint;
        joint["ur5_arm_shoulder_pan_joint"]=aj.joint1;
        joint["ur5_arm_shoulder_lift_joint"]=aj.joint2;
        joint["ur5_arm_elbow_joint"]=aj.joint3;
        joint["ur5_arm_wrist_1_joint"]=aj.joint4;
        joint["ur5_arm_wrist_2_joint"]=aj.joint5;
        joint["ur5_arm_wrist_3_joint"]=aj.joint6;

        q_(0)=aj.joint1;q_(1)=aj.joint2;q_(2)=aj.joint3;
        q_(3)=aj.joint4;q_(4)=aj.joint5;q_(5)=aj.joint6;

        jnt_to_pose_solver_->JntToCart(q_, x_);
        //cout<<x_.p.x()<<" "<<x_.p.y()<<" "<<x_.p.z();

        x_.M.GetQuaternion(a,b,c,d);
        //cout<<" "<<a<<" "<<b<<" "<<c<<" "<<d<<endl;

        ofstream file;
        file.open("/home/weili/catkin_ws/src/visual_servo/saved_image/grasp_pa.txt",ios::app|ios::out);
        file<<x_.p.x()<<" "<<x_.p.y()<<" "<<x_.p.z()<<" "<<a<<" "<<b<<" "<<c<<" "<<d<<";"<<endl;

        cv::waitKey(2.0);

        moveit::planning_interface::MoveGroup::Plan my_plan;
        robot_arm.setJointValueTarget(joint);
        robot_arm.setPlanningTime(6.0);
        robot_arm.setPlannerId("RRTstarkConfigDefault");
        bool success_=robot_arm.plan(my_plan);
        sleep(3.0);
        ROS_INFO("Visualizing simultanious plan 1st stage %s",success_?"":"FAILED");
        robot_arm.execute(my_plan);
        sleep(6.0);
        PictureSaved ps(nh);
        //ArmJointsSaved as(nh);
    }

    //==================================================================================//
    ROS_INFO("Finished");
    ros::spin();
    return 0;
}
