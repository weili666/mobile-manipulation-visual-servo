#include <Controller.h>
#include <Common.h>
#include <time.h>

#define M_pi 3.141592653
using namespace std;

double Over(cv::Mat dX1, cv::Mat dX2)
{
    cv::Mat delta_X1(6, 1, CV_64FC1);
    cv::Mat delta_X2(6, 1, CV_64FC1);
    delta_X1 = dX1;
    delta_X2 = dX2;
    double mod1 = sqrt(delta_X1.at<double>(0, 0)*delta_X1.at<double>(0, 0) + delta_X1.at<double>(1, 0)*delta_X1.at<double>(1, 0) + delta_X1.at<double>(2, 0)*delta_X1.at<double>(2, 0) + delta_X1.at<double>(3, 0)*delta_X1.at<double>(3, 0) + delta_X1.at<double>(4, 0)*delta_X1.at<double>(4, 0) + delta_X1.at<double>(5, 0)*delta_X1.at<double>(5, 0));
    double mod2 = sqrt(delta_X2.at<double>(0, 0)*delta_X2.at<double>(0, 0) + delta_X2.at<double>(1, 0)*delta_X2.at<double>(1, 0) + delta_X2.at<double>(2, 0)*delta_X2.at<double>(2, 0) + delta_X2.at<double>(3, 0)*delta_X2.at<double>(3, 0) + delta_X2.at<double>(4, 0)*delta_X2.at<double>(4, 0) + delta_X2.at<double>(5, 0)*delta_X2.at<double>(5, 0));
    double d = mod1/mod2;
    return d;
}

Controller::Controller(const ros::NodeHandle& node_handle, boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver_, boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_, planning_scene_monitor::PlanningSceneMonitorPtr sc_,moveit::planning_interface::MoveGroup* ro_arm_)
{
    nh = node_handle;
    ros::Rate loop_rate(100);
    camera_SE3 = cv::Mat(4, 4, CV_64FC1);
    distance = 10;
    stage = 1;

    speed_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5_arm/ur_driver/joint_speed", 1000);
    bulldog_pub_ = nh.advertise<geometry_msgs::Twist>("/bulldog_velocity_controller/cmd_vel", 1000);
    gripper_pub_ = nh.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 1);

    robotiq_85_msgs::GripperCmd gripper_yes;
    gripper_yes.position = 0.4;
    gripper_yes.speed = 0.1;
    gripper_pub_.publish(gripper_yes);

    jnt_to_pose_solver = jnt_to_pose_solver_;
    jnt_to_jac_solver = jnt_to_jac_solver_;
    scene = sc_;
    robot_arm = ro_arm_;

    arm_end_2_camera = cv::Mat(4,4,CV_64FC1);
    arm_end_2_camera.at<double>(0, 0) = -1.0;
    arm_end_2_camera.at<double>(0, 1) = 0.0;
    arm_end_2_camera.at<double>(0, 2) = 0.0;
    arm_end_2_camera.at<double>(0, 3) = 0.0;
    arm_end_2_camera.at<double>(1, 0) = 0.0;
    arm_end_2_camera.at<double>(1, 1) = 0.0;
    arm_end_2_camera.at<double>(1, 2) = 1.0;
    arm_end_2_camera.at<double>(1, 3) = -0.128;
    arm_end_2_camera.at<double>(2, 0) = 0.0;
    arm_end_2_camera.at<double>(2, 1) = 1.0;
    arm_end_2_camera.at<double>(2, 2) = 0.0;
    arm_end_2_camera.at<double>(2, 3) = -0.06;
    arm_end_2_camera.at<double>(3, 0) = 0.0;
    arm_end_2_camera.at<double>(3, 1) = 0.0;
    arm_end_2_camera.at<double>(3, 2) = 0.0;
    arm_end_2_camera.at<double>(3, 3) = 1.0;


    cout<<"//=====================================================================//"<<endl;
    cout<<"//****************************Begin Loop!******************************//"<<endl;
    cout<<"//=====================================================================//"<<endl;

    ros::Subscriber arm_velocity_sub = nh.subscribe("/ur5_arm/joint_states", 10, &Controller::arm_Velocity_Save, this);
    ros::Subscriber base_velocity_sub = nh.subscribe("/bulldog_velocity_controller/odom", 10, &Controller::base_Velocity_Save, this);
    ros::Subscriber camera_se3_sub = nh.subscribe("camera_se3_talker_msg", 10, &Controller::camera_SE3_Save, this);

    ros::spin();
    loop_rate.sleep();


}

cv::Mat Controller::getVControl()
{
    return V_control;
}

cv::Mat Controller::getDeltaX()
{
    return delta_X;
}

void Controller::arm_Velocity_Save(const sensor_msgs::JointState &arm_velocity)
{
    cv::Mat V_arm_ = cv::Mat(6,1,CV_64FC1);
    V_arm_.at<double>(0, 0) = arm_velocity.velocity[0];
    V_arm_.at<double>(1, 0) = arm_velocity.velocity[1];
    V_arm_.at<double>(2, 0) = arm_velocity.velocity[2];
    V_arm_.at<double>(3, 0) = arm_velocity.velocity[3];
    V_arm_.at<double>(4, 0) = arm_velocity.velocity[4];
    V_arm_.at<double>(5, 0) = arm_velocity.velocity[5];
    V_arm.push_back(V_arm_);
    //cout<<"V_arm : "<<V_arm_<<endl;
}

void Controller::base_Velocity_Save(const nav_msgs::Odometry &base_velocity)
{
    cv::Mat V_base_ = cv::Mat(2, 1, CV_64FC1);
    V_base_.at<double>(0, 0) = 2*base_velocity.twist.twist.angular.z;
    V_base_.at<double>(1, 0) = 2*base_velocity.twist.twist.linear.x;
    V_base.push_back(V_base_);
    //cout<<"V_base : "<<V_base_<<endl;
}

void Controller::camera_SE3_Save(const common::mark_vision &mark_msgs)
{
    cout<<"//=====================================================================//"<<endl;
    cout<<"//*******************************At Loop!******************************//"<<endl;
    cout<<"//=====================================================================//"<<endl;

    a = 0.197;
    b = 0;
    c = 0.319;
    D = 0.57;
    K = 0.2;
    Lambda = 0.2;
    double F = (726.406721300392 + 731.1184431877515) / 2.0;
    double u0 = 640;
    double v0 = 360;

    double h = 0.71;
    double delta_d = 0.6;
    trajectory_msgs::JointTrajectoryPoint trjp;
    trajectory_msgs::JointTrajectory trj;
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trj.points.push_back(trjp);
    ros::Time be = ros::Time::now();
    geometry_msgs::Twist cmdvel;


    //***************************************************************************//
    //********************the last target we want to move to*********************//
    //***************************************************************************//
    vector<double> thetas_target_last;
    vector<double> position_target_last;
    cv::Mat R_target_last = cv::Mat::zeros(3, 3, CV_64FC1);
    position_target_last.push_back(0);
    position_target_last.push_back(-0.29);
    position_target_last.push_back(h);
    thetas_target_last.push_back(-M_pi);
    thetas_target_last.push_back(0);
    thetas_target_last.push_back(M_pi);
    R_target_last.at<double>(0, 0) = -1.0;
    R_target_last.at<double>(1, 1) = 1.0;
    R_target_last.at<double>(2, 2) = -1.0;
    double u_target[4];

    double v_target[4];

    if(mark_msgs.state == 0)
    {
        u_target[0] = (905 + 724)/2.0 - u0;
        u_target[1] = (625 + 440)/2.0 - u0;
        u_target[2] = (890 + 711)/2.0 - u0;
        u_target[3] = (615 + 433)/2.0 - u0;

        v_target[0] = (339 + 348)/2.0 - v0;
        v_target[1] = (350 + 361)/2.0 - v0;
        v_target[2] = (65 + 69)/2.0 - v0;
        v_target[3] = (65 + 81)/2.0 - v0;
    }
    else if(mark_msgs.state == 1)
    {
        u_target[0] = 905 - u0;
        u_target[1] = 625 - u0;
        u_target[2] = 890 - u0;
        u_target[3] = 615 - u0;

        v_target[0] = 339 - v0;
        v_target[1] = 350 - v0;
        v_target[2] = 65 - v0;
        v_target[3] = 65 - v0;
    }
    else if(mark_msgs.state == 2)
    {
        u_target[0] = 724 - u0;
        u_target[1] = 440 - u0;
        u_target[2] = 711 - u0;
        u_target[3] = 433 - u0;

        v_target[0] = 348 - v0;
        v_target[1] = 361 - v0;
        v_target[2] = 69- v0;
        v_target[3] = 81 - v0;
    }


    cout<<"target 1 :"<<u_target[0]<<", "<<v_target[0]<<endl;
    cout<<"target 2 :"<<u_target[1]<<", "<<v_target[1]<<endl;
    cout<<"target 3 :"<<u_target[2]<<", "<<v_target[2]<<endl;
    cout<<"target 4 :"<<u_target[3]<<", "<<v_target[3]<<endl;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

    //***************************************************************************//
    //*******************the pre - target we want to move to*********************//
    //***************************************************************************//
    double delta_l = 0.06;
    vector<double> thetas_target_pre;
    vector<double> position_target_pre;
    cv::Mat R_target_pre = cv::Mat::zeros(3, 3, CV_64FC1);
    double pre_x, pre_y, pre_z;
    pre_x = position_target_last[0] - delta_l*R_target_last.at<double>(0, 1);
    pre_y = position_target_last[1] - delta_l*R_target_last.at<double>(1, 1);
    pre_z = position_target_last[2] - delta_l*R_target_last.at<double>(2, 1);
    position_target_pre.push_back(pre_x);
    position_target_pre.push_back(pre_y);
    position_target_pre.push_back(pre_z);
    thetas_target_pre.push_back(-M_pi);
    thetas_target_pre.push_back(0);
    thetas_target_pre.push_back(M_pi);
    R_target_pre.at<double>(0, 0) = -1.0;
    R_target_pre.at<double>(1, 1) = 1.0;
    R_target_pre.at<double>(2, 2) = -1.0;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


    cout<<"########################Stage is :#########################  "<<stage<<endl;

    if(stage == 1)
    {
        distance_thread = 0.11;
        position_target = position_target_pre;
        thetas_target = thetas_target_pre;
        R_target = R_target_pre;
    }
    else if(stage == 2)
    {
        distance_thread = 0.06;
        position_target = position_target_last;
        thetas_target = thetas_target_last;
        R_target = R_target_last;
    }


    V_real = cv::Mat(8, 1, CV_64FC1);
    cv::Mat V_arm_avg(6, 1, CV_64FC1);
    cv::Mat V_arm_(6, 1, CV_64FC1);
    cv::Mat V_base_avg(2, 1, CV_64FC1);
    cv::Mat V_base_(2, 1, CV_64FC1);


    for(int i = 0; i != 6; i++)
    {
        V_arm_avg.at<double>(i, 0) = 0;
    }
    for(int i = 0; i != 2; i++)
    {
        V_base_avg.at<double>(i, 0) = 0;
    }
    for(int i = 0; i != V_arm.size(); i++)
    {
        V_arm_ = V_arm[i];
        //cout<<"V_arm_ is :"<<V_arm_<<endl;
        V_arm_avg += V_arm_;

    }
    V_arm_avg = V_arm_avg/V_arm.size();
    for(int i = 0; i != V_base.size(); i++)
    {
        V_base_ = V_base[i];
        //cout<<"V_base_ is :"<<V_base_<<endl;
        V_base_avg += V_base_;
    }
    V_base_avg = V_base_avg/V_base.size();
    cout<<"the average velocity of arm is :"<<V_arm_avg<<endl;
    cout<<"the average velocity of base is :"<<V_base_avg<<endl;
    V_arm.clear();
    V_base.clear();

    V_arm_avg.copyTo(V_real(cv::Rect(0, 0, 1, 6)));
    V_base_avg.copyTo(V_real(cv::Rect(0, 6, 1, 2)));

    cout<<"the real velocity of the robot is :"<<V_real<<endl;

    double qq[4];
    qq[0] = mark_msgs.pose.orientation.x;
    qq[1] = mark_msgs.pose.orientation.y;
    qq[2] = mark_msgs.pose.orientation.z;
    qq[3] = mark_msgs.pose.orientation.w;
    camera_SE3 = cv::Mat(4,4,CV_64FC1);
    Quat2RotMat(qq, &camera_SE3);
    camera_SE3.at<double>(0, 3) = mark_msgs.pose.position.x;
    camera_SE3.at<double>(1, 3) = mark_msgs.pose.position.y;
    camera_SE3.at<double>(2, 3) = mark_msgs.pose.position.z;
    camera_SE3.at<double>(3, 0) = 0;
    camera_SE3.at<double>(3, 1) = 0;
    camera_SE3.at<double>(3, 2) = 0;
    camera_SE3.at<double>(3, 3) = 1.0;
    cv::Mat camera_Se3(4, 4, CV_64FC1);
    Quat2RotMat(qq, &camera_Se3);
    camera_Se3.at<double>(0, 3) = mark_msgs.pose.position.x;
    camera_Se3.at<double>(1, 3) = mark_msgs.pose.position.y;
    camera_Se3.at<double>(2, 3) = mark_msgs.pose.position.z;
    camera_Se3.at<double>(3, 0) = 0;
    camera_Se3.at<double>(3, 1) = 0;
    camera_Se3.at<double>(3, 2) = 0;
    camera_Se3.at<double>(3, 3) = 1.0;
    if(camera_SE3_vec.size() > 2)
    {
        double d1_2 = (camera_SE3_vec[camera_SE3_vec.size()-1].at<double>(0, 3) - mark_msgs.pose.position.x);
        double d1 = pow(d1_2,2);
        double d2_2 = (camera_SE3_vec[camera_SE3_vec.size()-1].at<double>(1, 3) - mark_msgs.pose.position.y);
        double d2 = pow(d2_2,2);
        double d3_2 = (camera_SE3_vec[camera_SE3_vec.size()-1].at<double>(2, 3) - mark_msgs.pose.position.z);
        double d3 = pow(d3_2,2);
        double d = std::sqrt(d1 + d2 + d3);

        if((d < delta_d)&&(mark_msgs.pose.position.y < 0))
        {
            camera_SE3_vec.push_back(camera_Se3);
        }
    }
    else
    {
        camera_SE3_vec.push_back(camera_Se3);
    }
    //cout<<"camera_pose, x:"<<mark_msgs.pose.position.x<<", y:"<<mark_msgs.pose.position.y<<", z:"<<mark_msgs.pose.position.z<<", qx:"<<mark_msgs.pose.orientation.x<<", qy:"<<mark_msgs.pose.orientation.y<<", qz:"<<mark_msgs.pose.orientation.z<<", qw:"<<mark_msgs.pose.orientation.w<<endl;
    //cout<<"camera_SE3_vec[camera_SE3_vec.size()-1]:"<<camera_SE3_vec[camera_SE3_vec.size()-1]<<endl;

    vector<double> joint_now;

    KDL::JntArray  q_;
    KDL::Frame     x_;
    KDL::Jacobian  J_;
    joint_now = robot_arm->getCurrentJointValues();
    cout<<"The robot joints are :["<<joint_now[0]<<", "<<joint_now[1]<<", "<<joint_now[2]<<", "<<joint_now[3]<<", "<<joint_now[4]<<", "<<joint_now[5]<<"]"<<endl;

    double a_,b_,c_,d_;
    q_.resize(6);
    J_.resize(6);
    q_(0)=joint_now[0];q_(1)=joint_now[1];q_(2)=joint_now[2];
    q_(3)=joint_now[3];q_(4)=joint_now[4];q_(5)=joint_now[5];
    jnt_to_pose_solver->JntToCart(q_, x_);
    jnt_to_jac_solver->JntToJac(q_, J_);
    cv::Mat Jacobi_origin (6, 6, CV_64FC1);
    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
             Jacobi_origin.at<double>(i,j) = J_.data(i,j);
        }
    }
    x_.M.GetQuaternion(a_,b_,c_,d_);
    double q[4];
    q[0] = a_;
    q[1] = b_;
    q[2] = c_;
    q[3] = d_;
    cv::Mat* M_b2e = new cv::Mat(3, 3, CV_64FC1);
    Quat2RotMat(q, M_b2e);
    arm_base_2_arm_end = cv::Mat(4,4,CV_64FC1);
    M_b2e->copyTo(arm_base_2_arm_end(cv::Rect(0, 0, 3, 3)));
    arm_base_2_arm_end.at<double>(0, 3) = x_.p.x();
    arm_base_2_arm_end.at<double>(1, 3) = x_.p.y();
    arm_base_2_arm_end.at<double>(2, 3) = x_.p.z();
    arm_base_2_arm_end.at<double>(3, 0) = 0.0;
    arm_base_2_arm_end.at<double>(3, 1) = 0.0;
    arm_base_2_arm_end.at<double>(3, 2) = 0.0;
    arm_base_2_arm_end.at<double>(3, 3) = 1.0;
    //cout<<"arm_base_2_arm_end:"<<endl<<arm_base_2_arm_end<<endl;
    arm_base_2_camera = arm_base_2_arm_end*arm_end_2_camera;
    //cout<<"camera_SE3_vec[camera_SE3_vec.size()-1]:"<<endl<<camera_SE3_vec[camera_SE3_vec.size()-1]<<endl;
    world_2_arm_base = camera_SE3_vec[camera_SE3_vec.size()-1]*arm_base_2_camera.inv();
    world_2_arm_end = camera_SE3_vec[camera_SE3_vec.size()-1]*arm_end_2_camera.inv();
    cout<<"world_2_arm_base:"<<endl<<world_2_arm_base<<endl;
    cout<<"world_2_arm_end:"<<endl<<world_2_arm_end<<endl;
    //cout<<"arm_end_2_camera:"<<endl<<arm_end_2_camera<<endl;
    theta_x = atan2(world_2_arm_base.at<double>(2, 1), world_2_arm_base.at<double>(2, 2));
    theta_y = atan2(-world_2_arm_base.at<double>(2, 0),sqrt(world_2_arm_base.at<double>(2, 1)*world_2_arm_base.at<double>(2, 1) + world_2_arm_base.at<double>(2, 2)*world_2_arm_base.at<double>(2, 2)));
    theta_z = atan2(world_2_arm_base.at<double>(1, 0), world_2_arm_base.at<double>(0, 0));
    alpha = theta_z;
    S1 = -(x_.p.x()+a)*sin(alpha)-(x_.p.y()+b)*cos(alpha);
    S2 = cos(alpha);
    S3 = (x_.p.x()+a)*cos(alpha)-(x_.p.y()+b)*sin(alpha);
    S4 = sin(alpha);
    cv::Mat Jacobi_1(6, 8, CV_64FC1, Scalar(0.0));
    Jacobi_1.at<double>(0, 0) = cos(alpha);
    Jacobi_1.at<double>(0, 1) = -sin(alpha);
    Jacobi_1.at<double>(1, 0) = sin(alpha);
    Jacobi_1.at<double>(1, 1) = cos(alpha);
    Jacobi_1.at<double>(2, 2) = 1.0;
    Jacobi_1.at<double>(3, 3) = cos(alpha);
    Jacobi_1.at<double>(3, 4) = -sin(alpha);
    Jacobi_1.at<double>(4, 3) = sin(alpha);
    Jacobi_1.at<double>(4, 4) = cos(alpha);
    Jacobi_1.at<double>(5, 5) = 1.0;
    Jacobi_1.at<double>(0, 6) = S1;
    Jacobi_1.at<double>(0, 7) = S2;
    Jacobi_1.at<double>(1, 6) = S3;
    Jacobi_1.at<double>(1, 7) = S4;
    Jacobi_1.at<double>(5, 6) = 1.0;
    Jacobi_1.at<double>(5, 7) = 0.0;
    cv::Mat Jacobi_2(8, 8, CV_64FC1, Scalar(0.0));
    Jacobi_origin.copyTo(Jacobi_2(cv::Rect(0, 0, 6, 6)));
    Jacobi_2.at<double>(6, 6) = 1.0;
    Jacobi_2.at<double>(7, 7) = 1.0;
    Jacobi = Jacobi_1*Jacobi_2;
    //cout<<"Jacobi_1 :"<<Jacobi_1<<endl;
    //cout<<"Jacobi_2 :"<<Jacobi_2<<endl;
    //cout<<"Jacobi :"<<Jacobi<<endl;

    //------------------------Kalman Filter Process----------------------------//

    cv::Mat X_prior(6, 1, CV_64FC1, Scalar(0.0));
    cv::Mat X_post(6, 1, CV_64FC1, Scalar(0.0));
    cv::Mat P_prior(6, 6, CV_64FC1, Scalar(0.0));
    cv::Mat P_post(6, 6, CV_64FC1, Scalar(0.0));
    cv::Mat Z(6, 1, CV_64FC1, Scalar(0.0));
    thetas_now = RotMat2Eular(&world_2_arm_end);
    position_now.push_back(world_2_arm_end.at<double>(0, 3));
    position_now.push_back(world_2_arm_end.at<double>(1, 3));
    position_now.push_back(world_2_arm_end.at<double>(2, 3));
    A = cv::Mat::eye(6, 6, CV_64FC1);
    B = cv::Mat(6, 8, CV_64FC1);
    H = cv::Mat::eye(6, 6, CV_64FC1);
    Q = cv::Mat::eye(6, 6, CV_64FC1);
    Q = 0.1*Q;
    R = cv::Mat::eye(6, 6, CV_64FC1);
    R = 0.1*R;
    Z.at<double>(0, 0) = position_now[0]; Z.at<double>(1, 0) = position_now[1]; Z.at<double>(2, 0) = position_now[2];
    Z.at<double>(3, 0) = thetas_now[0];   Z.at<double>(4, 0) = thetas_now[1];   Z.at<double>(5, 0) = thetas_now[2];

    cout<<"origin Z is :"<<Z<<endl;
    cout<<"X_post_vec.size() :"<<X_post_vec.size()<<endl;

    if(abs(Z.at<double>(3, 0)-thetas_target[0]-2*M_pi)<0.5)
    {
        Z.at<double>(3, 0) = Z.at<double>(3, 0) - 2*M_pi;
    }
    if(abs(Z.at<double>(3, 0)-thetas_target[0]+2*M_pi)<0.5)
    {
        Z.at<double>(3, 0) = Z.at<double>(3, 0) + 2*M_pi;
    }
    if(abs(Z.at<double>(4, 0)-thetas_target[1]-2*M_pi)<0.5)
    {
        Z.at<double>(4, 0) = Z.at<double>(4, 0) - 2*M_pi;
    }
    if(abs(Z.at<double>(4, 0)-thetas_target[1]+2*M_pi)<0.5)
    {
        Z.at<double>(4, 0) = Z.at<double>(4, 0) + 2*M_pi;
    }
    if(abs(Z.at<double>(5, 0)-thetas_target[2]-2*M_pi)<0.5)
    {
        Z.at<double>(5, 0) = Z.at<double>(5, 0) - 2*M_pi;
    }
    if(abs(Z.at<double>(5, 0)-thetas_target[2]+2*M_pi)<0.5)
    {
        Z.at<double>(5, 0) = Z.at<double>(5, 0) + 2*M_pi;
    }

    cout<<"Z is :"<<Z<<endl;

    if(X_post_vec.size()>0)
    {
        endTime = clock();
        delta_t = (double)(endTime-start_Time_vec[start_Time_vec.size()-1])/CLOCKS_PER_SEC;
        cout<<"start time is :"<<start_Time_vec[start_Time_vec.size()-1]<<", end time is :"<<endTime<<endl;
        cout<<"*******delta time is :"<<delta_t<<endl;
        //delta_t = 0.5;
        B = Jacobi*delta_t;
        X_post = X_post_vec[X_post_vec.size()-1];
        P_post = P_post_vec[P_post_vec.size()-1];
        //------------------------Kalman Filter Prior Process----------------------//

        X_prior = A*X_post + B*V_real;
        P_prior = A*P_post*A + Q;

        //------------------------Kalman Filter Post Process-----------------------//

        cv::Mat HPHR = H*P_prior*H.t() + R;
        cv::Mat eyeM = cv::Mat::eye(6, 6, CV_64FC1);
        Kalman = P_prior*H.t()*HPHR.inv();
        X_post = X_prior + Kalman*(Z - H*X_prior);
        P_post = (eyeM - Kalman*H)*P_prior;
        X_post_vec.push_back(X_post);
        P_post_vec.push_back(P_post);


        cout<<"X_prior is :"<<X_prior<<endl;
        cout<<"X_post is :"<<X_post<<endl;

    }
    else
    {
        X_post = Z;
        P_post = cv::Mat::eye(6, 6, CV_64FC1);
        P_post = 0.1*P_post;
        X_post_vec.push_back(X_post);
        P_post_vec.push_back(P_post);
    }

    //-------------------------------------------------------------------------//

    //---------------------------Calculate the dX------------------------------//

    delta_X = cv::Mat(6, 1, CV_64FC1, Scalar(0.0));
    cv::Mat dX(6, 1, CV_64FC1);
    cv::Mat world_2_arm_end_rot(3, 3, CV_64FC1);
    cv::Mat delta_R(3, 3, CV_64FC1);
    cv::Mat delta_R_inv(3, 3, CV_64FC1);
    world_2_arm_end(cv::Rect(0, 0, 3, 3)).copyTo(world_2_arm_end_rot);
    delta_R = R_target*world_2_arm_end_rot.inv();
    delta_R_inv = delta_R.inv();
    vector<double> delta_thetas = RotMat2Eular(&delta_R_inv);
    //cout<<"delta_R is:"<<endl<<delta_R<<endl;

    for(int i = 0; i < 3; i ++)
    {
        dX.at<double>(i, 0) = X_post.at<double>(i, 0) - position_target[i];
        dX.at<double>(i + 3, 0) = delta_thetas[i];
        //dX.at<double>(i + 3, 0) = X_post.at<double>(i + 3, 0) - thetas_target[i];//can not use minus in calculating angular directly
    }


    dX_vec.push_back(dX);
    delta_X = dX;

    //--------------------------------------------------------------------------//

    //------------------the 1st method to calculate pseudo Jacobian------------------//
    cv::Mat Jacobi_3(6, 6, CV_64FC1, Scalar(0.0));
    cv::Mat Jacobi_pseudo_inv(8, 6, CV_64FC1, Scalar(0.0));
    Jacobi_3 = Jacobi*Jacobi.t();
    Jacobi_pseudo_inv = Jacobi.t()*Jacobi_3.inv();

    //------------------the Moore-Penrose method to calculate pseudo Jacobian------------------//
//    cv::Mat Jacobi_pseudo_inv(8, 6, CV_64FC1, Scalar(0.0));
//    cv::Mat U, W, V;
//    cv::Mat W_inv = cv::Mat::zeros(6, 6, CV_64FC1);
//    cv::SVD::compute(Jacobi, W, U, V);
//    for(int i = 0; i < W.rows; i ++)
//    {
//            if(W.at<double>(i, 0)!=0)
//            {
//                W_inv.at<double>(i, i) = 1.0/W.at<double>(i, 0);
//            }
//    }
//    Jacobi_pseudo_inv = V.t()*W_inv*U.t();

    V_control = cv::Mat(8, 1, CV_64FC1);
    cv::Mat V_control_pbvs(8, 1, CV_64FC1, Scalar(0.0));
    cv::Mat V_control_ibvs(8, 1, CV_64FC1, Scalar(0.0));

    //---------------------the control rule of PBVS----------------------//

    cv::Mat L_3d = cv::Mat::eye(6, 6, CV_64FC1);
    L_3d.at<double>(0, 1+3) = -delta_X.at<double>(2, 0);
    L_3d.at<double>(0, 2+3) = delta_X.at<double>(1, 0);
    L_3d.at<double>(1, 0+3) = delta_X.at<double>(2, 0);
    L_3d.at<double>(1, 2+3) = -delta_X.at<double>(0, 0);
    L_3d.at<double>(2, 0+3) = -delta_X.at<double>(1, 0);
    L_3d.at<double>(2, 1+3) = delta_X.at<double>(0, 0);
    V_control_pbvs = -K*Jacobi_pseudo_inv*L_3d*delta_X;
    cout<<"The velocity control rule of PBVS is :"<<V_control_pbvs<<endl;
    //---------------------the control rule of IBVS----------------------//

    cv::Mat camera_SO3(3, 3, CV_64FC1, Scalar(0.0));
    cv::Mat grasp_SO3(3, 3, CV_64FC1, Scalar(0.0));
    camera_SE3(cv::Rect(0, 0, 3, 3)).copyTo(camera_SO3);
    cv::Mat pd(3, 1, CV_64FC1);
    cv::Mat pt(3, 1, CV_64FC1);
    cv::Mat H(6, 6, CV_64FC1, Scalar(0.0));
    pd.at<double>(0, 0) = 0.0;
    pd.at<double>(1, 0) = 0.06;
    pd.at<double>(2, 0) = 0.128;
    pt = camera_SO3*pd;
    cv::Mat R_pt(3, 3, CV_64FC1, Scalar(0.0));
    R_pt.at<double>(0, 1) = -pt.at<double>(2, 0);
    R_pt.at<double>(0, 2) = pt.at<double>(1, 0);
    R_pt.at<double>(1, 0) = pt.at<double>(2, 0);
    R_pt.at<double>(1, 2) = -pt.at<double>(0, 0);
    R_pt.at<double>(2, 0) = -pt.at<double>(1, 0);
    R_pt.at<double>(2, 1) = pt.at<double>(0, 0);
    cv::Mat R_12(3, 3, CV_64FC1, Scalar(0.0));
    R_12 = -R_pt*camera_SO3;
    //grasp_SO3 = world_2_arm_end_rot.inv()*camera_SO3;
    grasp_SO3.at<double>(0, 0) = -1.0;
    grasp_SO3.at<double>(2, 1) = 1.0;
    grasp_SO3.at<double>(1, 2) = 1.0;
    camera_SO3.copyTo(H(cv::Rect(0, 0, 3, 3)));
    //grasp_SO3.copyTo(H(cv::Rect(3, 3, 3, 3)));
    camera_SO3.copyTo(H(cv::Rect(3, 3, 3, 3)));
    R_12.copyTo(H(cv::Rect(3, 0, 3, 3)));
    cout<<"Matrix H is :"<<endl<<H<<endl;
    cv::Mat L_2d(8, 6, CV_64FC1, Scalar(0.0));

    double u_[4];
    double v_[4];
    double Z_[4];

    u_[0] = mark_msgs.point1.x - u0;
    v_[0] = mark_msgs.point1.y - v0;
    Z_[0] = mark_msgs.point1.z;
    u_[1] = mark_msgs.point2.x - u0;
    v_[1] = mark_msgs.point2.y - v0;
    Z_[1] = mark_msgs.point2.z;
    u_[2] = mark_msgs.point3.x - u0;
    v_[2] = mark_msgs.point3.y - v0;
    Z_[2] = mark_msgs.point3.z;
    u_[3] = mark_msgs.point4.x - u0;
    v_[3] = mark_msgs.point4.y - v0;
    Z_[3] = mark_msgs.point4.z;
    cout<<"the mark points position :"<<"u1:"<<u_[0]<<",v1:"<<v_[0]<<",z1:"<<Z_[0]<<endl
       <<"u2:"<<u_[1]<<",v2:"<<v_[1]<<",z2:"<<Z_[1]<<endl
      <<"u3:"<<u_[2]<<",v3:"<<v_[2]<<",z3:"<<Z_[2]<<endl
     <<"u4:"<<u_[3]<<",v4:"<<v_[3]<<",z4:"<<Z_[3]<<endl;
    for(int i = 0; i < 4; i ++)
    {
        L_2d.at<double>(2*i, 0) = -F/Z_[i];
        L_2d.at<double>(2*i, 2) = u_[i]/Z_[i];
        L_2d.at<double>(2*i, 3) = u_[i]*v_[i]/F;
        L_2d.at<double>(2*i, 4) = -(F*F + u_[i]*u_[i])/F;
        L_2d.at<double>(2*i, 5) = v_[i];
        L_2d.at<double>(2*i + 1, 1) = -F/Z_[i];
        L_2d.at<double>(2*i + 1, 2) = v_[i]/Z_[i];
        L_2d.at<double>(2*i + 1, 3) = (F*F + v_[i]*v_[i])/F;
        L_2d.at<double>(2*i + 1, 4) = -u_[i]*v_[i]/F;
        L_2d.at<double>(2*i + 1, 5) = -u_[i];
    }
    //cout<<" The Matrix L_2d is : "<<endl<<L_2d<<endl;
    cv::Mat L_2d_pseudo_inv(6, 8, CV_64FC1, Scalar(0.0));
    cv::Mat L_2d_squre(6, 6, CV_64FC1, Scalar(0.0));
    L_2d_squre = L_2d.t()*L_2d;
    L_2d_pseudo_inv = L_2d_squre.inv()*L_2d.t();
    //cout<<" The Matrix L_2d_inv is : "<<endl<<L_2d_pseudo_inv<<endl;
    cv::Mat delta_u(8, 1, CV_64FC1, Scalar(0.0));
    delta_u.at<double>(0, 0) = u_[0] - u_target[0];
    delta_u.at<double>(1, 0) = v_[0] - v_target[0];
    delta_u.at<double>(2, 0) = u_[1] - u_target[1];
    delta_u.at<double>(3, 0) = v_[1] - v_target[1];
    delta_u.at<double>(4, 0) = u_[2] - u_target[2];
    delta_u.at<double>(5, 0) = v_[2] - v_target[2];
    delta_u.at<double>(6, 0) = u_[3] - u_target[3];
    delta_u.at<double>(7, 0) = v_[3] - v_target[3];
    cout<<" The delta_u is : "<<endl<<delta_u<<endl;
    cv::Mat v_camera_ibvs = -Lambda*H*L_2d_pseudo_inv*delta_u;
    V_control_ibvs = -Lambda*Jacobi_pseudo_inv*H*L_2d_pseudo_inv*delta_u;
    //cout<<"The velocity of camera control rule of IBVS is :"<<v_camera_ibvs<<endl;
    cout<<"The velocity control rule of IBVS is :"<<V_control_ibvs<<endl;
    //---------------------the control rule of HVS-----------------------//

    double d1 = 0.5;
    cout<<"Alpha:"<<alpha<<endl;
    distance = 0;
    for(int i = 0; i < 6; i ++)
    {
        distance += delta_X.at<double>(i, 0)*delta_X.at<double>(i, 0);
    }
    distance = sqrt(distance);
    double yita_ibvs = exp(-distance/d1);
    double yita_pbvs = 1 - exp(-distance/d1);
    cout<<"yita_ibvs :"<<yita_ibvs<<endl;
    cout<<"yita_pbvs :"<<yita_pbvs<<endl;

    //-------------------------------------------------------------------//

//    V_control = V_control_ibvs;
//    V_control = V_control_pbvs;
    V_control = yita_ibvs*V_control_ibvs + yita_pbvs*V_control_pbvs;
    //-------------------------------------------------------------------//

    V_control_vec.push_back(V_control);
    //cout<<"Jacobi_pseudo_inv :"<<endl<<Jacobi_pseudo_inv<<endl;
    ros::Time k = ros::Time::now();
    bool first = true;
    if(first)
    {
        first = false;
        be = k;
    }
    ros::Duration du = k - be;


    trj.header.stamp = ros::Time::now();

    position_now.clear();
    thetas_now.clear();
    startTime = clock();
    clock_t startTime_copy = clock();
    start_Time_vec.push_back(startTime_copy);

    cout<<"***the delta_X of the robot is:"<<delta_X<<endl;
    cout<<"distance is :"<<distance<<endl;
    cout<<"***the angular velocity of the base:"<<V_control.at<double>(6, 0)<<endl;
    cout<<"***the velocity of the base:"<<V_control.at<double>(7, 0)<<endl;
    cout<<"***the executing velocity of the arm is:"<<V_control.at<double>(0, 0)<<",***"<<V_control.at<double>(1, 0)<<",***"<<V_control.at<double>(2, 0)<<",***"<<V_control.at<double>(3, 0)<<",***"<<V_control.at<double>(4, 0)<<",***"<<V_control.at<double>(5, 0)<<endl;


    if(distance >= distance_thread)
    {
        trj.points[0].velocities[0] = V_control.at<double>(0, 0);
        trj.points[0].velocities[1] = V_control.at<double>(1, 0);
        trj.points[0].velocities[2] = V_control.at<double>(2, 0);
        trj.points[0].velocities[3] = V_control.at<double>(3, 0);
        trj.points[0].velocities[4] = V_control.at<double>(4, 0);
        trj.points[0].velocities[5] = V_control.at<double>(5, 0);

        cmdvel.linear.x = V_control.at<double>(7, 0);
        cmdvel.angular.z = V_control.at<double>(6, 0);

        speed_pub_.publish(trj);
        bulldog_pub_.publish(cmdvel);
    }
    else if(distance < distance_thread)
    {
        trj.points[0].velocities[0] = 0.0;
        trj.points[0].velocities[1] = 0.0;
        trj.points[0].velocities[2] = 0.0;
        trj.points[0].velocities[3] = 0.0;
        trj.points[0].velocities[4] = 0.0;
        trj.points[0].velocities[5] = 0.0;

        cmdvel.linear.x = 0.0;
        cmdvel.angular.z = 0.0;

        if(stage == 2)
        {
            robotiq_85_msgs::GripperCmd gripper_yes;
            gripper_yes.position = -0.4;
            gripper_yes.speed = -0.1;
            gripper_pub_.publish(gripper_yes);
        }

        stage = 2;
        speed_pub_.publish(trj);
        bulldog_pub_.publish(cmdvel);


    }

}
