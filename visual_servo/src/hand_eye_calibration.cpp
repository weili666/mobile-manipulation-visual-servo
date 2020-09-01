#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include <ctime>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <math.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/eigen.hpp>
#include <algorithm>


#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
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
int num=0;

Mat skew(Mat_<double> p)
{
    Mat P(3, 3, CV_64FC1);
    P.at<double>(0,0)=0;
    P.at<double>(0,1)=-p.at<double>(2,0);
    P.at<double>(0,2)=p.at<double>(1,0);
    P.at<double>(1,0)=p.at<double>(2,0);
    P.at<double>(1,1)=0;
    P.at<double>(1,2)=-p.at<double>(0,0);
    P.at<double>(2,0)=-p.at<double>(1,0);
    P.at<double>(2,1)=p.at<double>(0,0);
    P.at<double>(2,2)=0;
    return P;
}

Mat cross(const Mat& a,const Mat& b)
{
    Mat c(3, 1, CV_64FC1);
    c.at<double>(0,0) = a.at<double>(1,0)*b.at<double>(2,0)-a.at<double>(2,0)*b.at<double>(1,0);
    c.at<double>(1,0) = a.at<double>(2,0)*b.at<double>(0,0)-a.at<double>(0,0)*b.at<double>(2,0);
    c.at<double>(2,0) = a.at<double>(0,0)*b.at<double>(1,0)-a.at<double>(1,0)*b.at<double>(0,0);
    return c;
}

Mat qmult(const Eigen::Quaterniond& q1,const Eigen::Quaterniond& q2)
{
    Mat prod1(4, 1, CV_64FC1);
    prod1.at<double>(0,0) = q1.x()*q2.w(); prod1.at<double>(1,0) = -q1.x()*q2.z(); prod1.at<double>(2,0) = q1.x()*q2.y(); prod1.at<double>(3,0) = -q1.x()*q2.x();
    Mat prod2(4, 1, CV_64FC1);
    prod2.at<double>(0,0) = q1.y()*q2.z(); prod2.at<double>(1,0) = q1.y()*q2.w(); prod2.at<double>(2,0) = -q1.y()*q2.x(); prod2.at<double>(3,0) = -q1.y()*q2.y();
    Mat prod3(4, 1, CV_64FC1);
    prod3.at<double>(0,0) = -q1.z()*q2.y(); prod3.at<double>(1,0) = q1.z()*q2.x(); prod3.at<double>(2,0) = q1.z()*q2.w(); prod3.at<double>(3,0) = -q1.z()*q2.z();
    Mat prod4(4, 1, CV_64FC1);
    prod4.at<double>(0,0) = q1.w()*q2.x(); prod4.at<double>(1,0) = q1.w()*q2.y(); prod4.at<double>(2,0) = q1.w()*q2.z(); prod4.at<double>(3,0) = q1.w()*q2.w();
    Mat q_out(4, 1, CV_64FC1);
    q_out = prod1 + prod2 + prod3 + prod4;
    return q_out;
}



class HandEyeCalibration
{
public:
    HandEyeCalibration(int num_cols,int num_rows,double dist,const vector<geometry_msgs::Pose>& pose_vec):target_poses(pose_vec)
    {
        cols = num_cols;
        rows = num_rows;
        distance = dist;
        point_counts=cols*rows;
        cv::Size patternSize;
        patternSize.width = cols;
        patternSize.height = rows;
        for (int i=0;i<patternSize.height;i++)
        {
            for (int j=0;j<patternSize.width;j++)
            {
                worldPoints.push_back(cv::Point3f(j*distance,i*distance,0));
            }
        }

        cv::Mat img,img2;
        cv::Mat calpic,calpic2;
        char buff[100];
        char buff2[100];

        int num_of_pic=target_poses.size();
        cout<<"num of picture:"<<num_of_pic<<endl;
        //===========================DETECT CORNER============================//
        for(int i=1;i<=num_of_pic;i++)
        {
            sprintf(buff, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_left_%d.jpg",i);
            sprintf(buff2, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_right_%d.jpg",i);

            img = cv::imread(buff);
            img2 = cv::imread(buff2);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);
            cv::cvtColor(img2, calpic2, CV_BGR2GRAY);

            bool find=cv::findChessboardCorners(img,patternSize,corners);
            bool find2=cv::findChessboardCorners(img2,patternSize,corners2);
//            cv::cornerSubPix(calpic, corners, cv::Size(11, 11), cv::Size(-1, -1),
//                        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            cornersVect.push_back(corners);
            cornersVect2.push_back(corners2);
            worldPointsVect.push_back(worldPoints);

            cv::drawChessboardCorners(img,patternSize,corners,1);
            char buff3[100];
            sprintf(buff3, "/home/weili/catkin_ws/src/visual_servo/saved_image/draw_chessboard_coners_left_%d.jpg",i);
            cv::imwrite(buff3, img);
            cv::drawChessboardCorners(img2,patternSize,corners2,1);
            char buff4[100];
            sprintf(buff4, "/home/weili/catkin_ws/src/visual_servo/saved_image/draw_chessboard_coners_right_%d.jpg",i);
            cv::imwrite(buff4, img2);
        }
        //===================================================================//

        cv::calibrateCamera(worldPointsVect,cornersVect,calpic.size(),cameraMatrix,distCoeffs,rvecs,tvecs);
        cv::calibrateCamera(worldPointsVect,cornersVect2,calpic2.size(),cameraMatrix2,distCoeffs,rvecs2,tvecs2);//calculate the chessboard coodinate in camera coordinate.

        //============================RE-PROJECT=============================//
        for(int i=0;i<cornersVect.size();i++)
        {
            sprintf(buff, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_left_%d.jpg",i+1);
            img = cv::imread(buff);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(worldPointsVect[i],rvecs[i],tvecs[i],cameraMatrix,distCoeffs,projectedPoints);
            cornersVectProj.push_back(projectedPoints);

            cv::drawChessboardCorners(calpic,patternSize,projectedPoints,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/catkin_ws/src/visual_servo/saved_image/draw_chessboard_project_coners_left_%d.jpg",i+1);
            cv::imwrite(buff2, calpic);
            cv::undistort(img,calpic,cameraMatrix,distCoeffs);
            char buff3[100];
            sprintf(buff3, "/home/weili/catkin_ws/src/visual_servo/saved_image/undistort_left_%d.jpg",i+1);
            cv::imwrite(buff3, calpic);

        }

        for(int i=0;i<cornersVect.size();i++)
        {
            sprintf(buff, "/home/weili/catkin_ws/src/visual_servo/saved_image/saved_right_%d.jpg",i+1);
            img2 = cv::imread(buff);
            cv::cvtColor(img2, calpic2, CV_BGR2GRAY);
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(worldPointsVect[i],rvecs2[i],tvecs2[i],cameraMatrix2,distCoeffs2,projectedPoints);
            cornersVectProj2.push_back(projectedPoints);

            cv::drawChessboardCorners(calpic2,patternSize,projectedPoints,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/catkin_ws/src/visual_servo/saved_image/draw_chessboard_project_coners_right_%d.jpg",i+1);
            cv::imwrite(buff2, calpic2);
            cv::undistort(img2,calpic2,cameraMatrix2,distCoeffs2);
            char buff3[100];
            sprintf(buff3, "/home/weili/catkin_ws/src/visual_servo/saved_image/undistort_right_%d.jpg",i+1);
            cv::imwrite(buff3, calpic2);

        }
        //===================================================================//

        //===================================================================//

        cout<<"<<<<<<<<<<<<<---------calpic_size-------->>>>>>>>>>>>>>>"<<endl;
        cout<<calpic.size()<<endl;
        cout<<"size of connersVect:"<<cornersVect.size()<<endl;
        cout<<"<<<<<<<<<<<<<---------cameraMatrix-------->>>>>>>>>>>>>>>"<<endl;
        cout<<cameraMatrix<<endl;
        cout<<"<<<<<<<<<<<<<-----------distCoeffs-------->>>>>>>>>>>>>>>"<<endl;
        cout<<distCoeffs<<endl;
        cout<<"<<<<<<<<<<<<<---------cameraMatrix-------->>>>>>>>>>>>>>>"<<endl;
        cout<<cameraMatrix2<<endl;
        cout<<"<<<<<<<<<<<<<-----------distCoeffs-------->>>>>>>>>>>>>>>"<<endl;
        cout<<distCoeffs2<<endl;



        for(int k=1;k<=num_of_pic;k++)
        {
            cv::Mat rotMatrixi;
            cv::Mat Grasp_matrix_rt(4, 4, CV_64FC1);
            cv::Mat Camera_matrix_rt(4, 4, CV_64FC1);
            cv::Mat rotMatrixi2;
            cv::Mat Camera_matrix_rt2(4, 4, CV_64FC1);
            cv::Rodrigues(rvecs[k-1],rotMatrixi,JacoMatrix);
            cv::Rodrigues(rvecs2[k-1],rotMatrixi2,JacoMatrix2);

            rotMatrix = rotMatrixi.inv();
            rotMatrix2 = rotMatrixi2.inv();

            cv::Mat tveci = tvecs[k-1];
            cv::Mat tveci2 = tvecs2[k-1];

            tvecMatrix = rotMatrix*(-tveci);
            tvecMatrix2 = rotMatrix2*(-tveci2);

            cv::Mat rotMat(3, 3, CV_64FC1);
            cv::Mat tranMat(3, 1, CV_64FC1);
            cv::Mat rotMat2(3, 3, CV_64FC1);
            cv::Mat tranMat2(3, 1, CV_64FC1);
            //==========================Camera==========================//
            for (int i=0;i<rotMatrix.rows;i++)
            {
                for (int j=0;j<rotMatrix.cols;j++)
                {
                    rotMat.at<double>(i,j)=rotMatrix.at<double>(i,j);
                    rotMat2.at<double>(i,j)=rotMatrix2.at<double>(i,j);
                }
            }
            for (int i=0;i<3;i++)
            {
                tranMat.at<double>(i,0)= tvecMatrix.at<double>(i,0)/1000.0;
                tranMat2.at<double>(i,0)= tvecMatrix2.at<double>(i,0)/1000.0;
            }

            Camera_Matrix_r.push_back(rotMat);
            Camera_Matrix_t.push_back(tranMat);
            Camera_Matrix_r2.push_back(rotMat2);
            Camera_Matrix_t2.push_back(tranMat2);
            rotMat.copyTo(Camera_matrix_rt(Rect(0, 0, 3, 3)));
            tranMat.copyTo(Camera_matrix_rt(Rect(3, 0, 1, 3)));
            rotMat2.copyTo(Camera_matrix_rt2(Rect(0, 0, 3, 3)));
            tranMat2.copyTo(Camera_matrix_rt2(Rect(3, 0, 1, 3)));

            Camera_matrix_rt.at<double>(3, 0) = 0.0;
            Camera_matrix_rt.at<double>(3, 1) = 0.0;
            Camera_matrix_rt.at<double>(3, 2) = 0.0;
            Camera_matrix_rt.at<double>(3, 3) = 1.0;
            Camera_Matrix_rt.push_back(Camera_matrix_rt);

            Camera_matrix_rt2.at<double>(3, 0) = 0.0;
            Camera_matrix_rt2.at<double>(3, 1) = 0.0;
            Camera_matrix_rt2.at<double>(3, 2) = 0.0;
            Camera_matrix_rt2.at<double>(3, 3) = 1.0;
            Camera_Matrix_rt2.push_back(Camera_matrix_rt2);
            //==========================================================//

            //==========================Grasp==========================//
            geometry_msgs::Pose pose=target_poses[k-1];
            Eigen::Quaterniond q (pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
            Eigen::Matrix3d G_Mat(q);
            cv::Mat grasp_mat(3, 3, CV_64FC1);
            for(int i=0;i<grasp_mat.rows;i++)
            {
                for(int j=0;j<grasp_mat.cols;j++)
                {
                    grasp_mat.at<double>(i,j)=G_Mat(i,j);
                }
            }
            cv::Mat grasp_mat_t(3, 1, CV_64FC1);
            grasp_mat_t.at<double>(0,0)=pose.position.x;
            grasp_mat_t.at<double>(1,0)=pose.position.y;
            grasp_mat_t.at<double>(2,0)=pose.position.z;
            Grasp_Matrix_r.push_back(grasp_mat);
            Grasp_Matrix_t.push_back(grasp_mat_t);

            grasp_mat.copyTo(Grasp_matrix_rt(Rect(0, 0, 3, 3)));
            grasp_mat_t.copyTo(Grasp_matrix_rt(Rect(3, 0, 1, 3)));
            Grasp_matrix_rt.at<double>(3, 0) = 0.0;
            Grasp_matrix_rt.at<double>(3, 1) = 0.0;
            Grasp_matrix_rt.at<double>(3, 2) = 0.0;
            Grasp_matrix_rt.at<double>(3, 3) = 1.0;
            Grasp_Matrix_rt.push_back(Grasp_matrix_rt);
            //==========================================================//
        }

        for(int i=0;i<num_of_pic;i++)
        {
            cout<<"======================Camera_Matrix_rt "<<i+1<<" ===================="<<endl;
            cout<<Camera_Matrix_rt[i]<<endl;

            cout<<"======================Camera_Matrix_rt2 "<<i+1<<" ===================="<<endl;
            cout<<Camera_Matrix_rt2[i]<<endl;


        }


        int k=1;
        for(int i=0;i<Grasp_Matrix_r.size()-k;i++)
        {
            cv::Mat grasp_mat_rt(4, 4, CV_64FC1);
            cv::Mat camera_mat_rt(4, 4, CV_64FC1);
            cv::Mat camera_mat_rt2(4, 4, CV_64FC1);

            grasp_mat_rt = (Grasp_Matrix_rt[i].inv())*Grasp_Matrix_rt[i+k];
            camera_mat_rt = (Camera_Matrix_rt[i].inv())*Camera_Matrix_rt[i+k];

            camera_mat_rt2 = (Camera_Matrix_rt2[i].inv())*Camera_Matrix_rt2[i+k];

            cout<<"------------------------GraspMatrixRT "<<i+1<<" ------------------"<<endl;
            cout<<grasp_mat_rt<<endl;

            cout<<"------------------------CameraMatrixRT "<<i+1<<" ------------------"<<endl;
            cout<<camera_mat_rt<<endl;
            cout<<camera_mat_rt2<<endl;

            Grasp_Rotate_Transf_Matrix.push_back(grasp_mat_rt);
            Camera_Rotate_Transf_Matrix.push_back(camera_mat_rt);

            Camera_Rotate_Transf_Matrix2.push_back(camera_mat_rt2);

        }

        ofstream file;
        file.open("/home/weili/catkin_ws/src/visual_servo/saved_image/extern_matrix.txt",ios::app|ios::out);
        file<<cameraMatrix<<endl;
        file<<distCoeffs<<endl;
        file<<Hcg<<endl;

        cout<<"begin hand eye calibration!!"<<endl;
        Hcg = Tsai_HandEye( Grasp_Rotate_Transf_Matrix,Camera_Rotate_Transf_Matrix);
        cout<<"------------------------Tsai_HandEye Hand-Camera-Matrix 1------------------"<<endl;
        cout<<Hcg<<endl;
        file<<Hcg<<endl;

        cout<<"begin hand eye calibration!!"<<endl;
        Hcg2 = Tsai_HandEye( Grasp_Rotate_Transf_Matrix,Camera_Rotate_Transf_Matrix2);
        cout<<"------------------------Tsai_HandEye Hand-Camera-Matrix 2------------------"<<endl;
        cout<<Hcg2<<endl;
        file<<Hcg2<<endl;



    }

    Mat Tsai_HandEye(const vector<Mat>& Hgij, const vector<Mat>& Hcij);

    int delta(int x,int y);

 private:
    int cols;
    int rows;
    float distance;    //间距30mm
    std::vector<cv::Point2f> corners,corners2;
    std::vector<std::vector<cv::Point2f> > cornersVect,cornersVect2;
    std::vector<std::vector<cv::Point2f> > cornersVectProj,cornersVectProj2;
    std::vector<cv::Point3f> worldPoints;
    std::vector<std::vector<cv::Point3f> > worldPointsVect;
    int point_counts;
    cv::Mat cameraMatrix,cameraMatrix2,distCoeffs,distCoeffs2;
    cv::Mat tvecMatrix,rotMatrix,JacoMatrix;
    cv::Mat tvecMatrix2,rotMatrix2,JacoMatrix2;
    cv::Mat Hand_Camera_Matrix;
    cv::Mat Camera_Ext;
    cv::Mat Hcg,Hcg2;
    std::vector<cv::Mat> Grasp_Rotate_Transf_Matrix,Grasp_Rotate_Transf_Matrix2;
    std::vector<cv::Mat> Grasp_Rotate_Matrix;
    std::vector<cv::Mat> Grasp_Transf_Matrix;
    std::vector<cv::Mat> Grasp_Matrix_r;
    std::vector<cv::Mat> Grasp_Matrix_t;

    std::vector<cv::Mat> Camera_Rotate_Transf_Matrix,Camera_Rotate_Transf_Matrix2;
    std::vector<cv::Mat> Camera_Rotate_Matrix;
    std::vector<cv::Mat> Camera_Transf_Matrix;
    std::vector<cv::Mat> Camera_Matrix_r,Camera_Matrix_r2;
    std::vector<cv::Mat> Camera_Matrix_t,Camera_Matrix_t2;

    std::vector<cv::Mat> Grasp_Matrix_rt;
    std::vector<cv::Mat> Camera_Matrix_rt,Camera_Matrix_rt2;

    std::vector<cv::Mat> rvecs,rvecs2,tvecs,tvecs2;
    vector<geometry_msgs::Pose> target_poses;
};

Mat HandEyeCalibration::Tsai_HandEye(const vector<Mat>& Hgij,const vector<Mat>& Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Hcg_(4, 4, CV_64FC1);
    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);

    Mat rgij(3, 1, CV_64FC1);
    Mat rcij(3, 1, CV_64FC1);

    double theta_gij;
    double theta_cij;

    Mat rngij(3, 1, CV_64FC1);
    Mat rncij(3, 1, CV_64FC1);

    Mat Pgij(3, 1, CV_64FC1);
    Mat Pcij(3, 1, CV_64FC1);

    Mat tempA(3, 3, CV_64FC1);
    Mat tempb(3, 1, CV_64FC1);

    Mat A;
    Mat b;
    Mat pinA;

    Mat Pcg_prime(3, 1, CV_64FC1);
    Mat Pcg(3, 1, CV_64FC1);
    Mat PcgTrs(1, 3, CV_64FC1);

    Mat Rcg(3, 3, CV_64FC1);

    Mat eyeM = Mat::eye(3, 3, CV_64FC1);

    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat tempAA(3, 3, CV_64FC1);
    Mat tempbb(3, 1, CV_64FC1);

    Mat AA;
    Mat bb;
    Mat pinAA;

    Mat Tcg(3, 1, CV_64FC1);

    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        Rodrigues(Rgij, rgij);
        Rodrigues(Rcij, rcij);

        theta_gij = norm(rgij);
        theta_cij = norm(rcij);

        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;

        Pgij = 2 * sin(theta_gij / 2)*rngij;
        Pcij = 2 * sin(theta_cij / 2)*rncij;

        tempA = skew(Pgij + Pcij);
        tempb = Pcij - Pgij;

        A.push_back(tempA);
        b.push_back(tempb);

    }


    //Compute rotation

    invert(A, pinA, DECOMP_SVD);
    Pcg_prime = pinA * b;
    Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
    PcgTrs = Pcg.t();
    Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));



    //Computer Translation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);


        tempAA = Rgij - eyeM;
        tempbb = Rcg * Tcij - Tgij;

        AA.push_back(tempAA);
        bb.push_back(tempbb);
    }

    invert(AA, pinAA, DECOMP_SVD);
    Tcg = pinAA * bb;

    Rcg.copyTo(Hcg_(Rect(0, 0, 3, 3)));
    Tcg.copyTo(Hcg_(Rect(3, 0, 1, 3)));
    Hcg_.at<double>(3, 0) = 0.0;
    Hcg_.at<double>(3, 1) = 0.0;
    Hcg_.at<double>(3, 2) = 0.0;
    Hcg_.at<double>(3, 3) = 1.0;
    return Hcg_;
}


int HandEyeCalibration::delta(int x,int y)
{
    int result;
    if(x==y)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    return result;
}


int main(int argc, char** argv)
{

    int num_cols = 8;
    int num_rows = 6;
    double dist = 30;

    ifstream ifile;
    ifile.open("/home/weili/catkin_ws/src/visual_servo/saved_image/grasp_pa.txt");
    char buffer[256];

    int num=0;
    int num_of_pic = 6;
    int num_of_pic2 = num_of_pic;
    vector< vector<char> > data_;
    vector<char> data_iter;
    if(!ifile.is_open())
    {cout << "Error opening file"; exit (1);}
    while(num_of_pic)
    {
        data_iter.clear();
        ifile.getline(buffer,100);
        for(int i=0;i<256;i++)
        {
            if(buffer[i]==' ')
            {
                for(int i=0;i<data_iter.size();i++)
                {
                    //cout<<data_iter[i];
                }
                //cout<<endl;
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                continue;
            }
            else if(buffer[i]==';')
            {
                for(int i=0;i<data_iter.size();i++)
                {
                    //cout<<data_iter[i];
                }
                //cout<<endl;
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
        num_of_pic--;
    }


    int num_data = 7*num_of_pic2;
    double ddata[num_data];
    for(int i=0;i<num_data;i++)
    {
        string data;

        for(int j=0;j<data_[i].size();j++)
        {
            data+=data_[i][j];
        }

        ddata[i]=atof(data.c_str());

    }

    vector<geometry_msgs::Pose> target_poses;

    for(int i=0;i<num_of_pic2;i++)
    {
        geometry_msgs::Pose targ_pose;
        targ_pose.position.x = ddata[i*7+0];
        targ_pose.position.y = ddata[i*7+1];
        targ_pose.position.z = ddata[i*7+2];
        targ_pose.orientation.x = ddata[i*7+3];
        targ_pose.orientation.y = ddata[i*7+4];
        targ_pose.orientation.z = ddata[i*7+5];
        targ_pose.orientation.w = ddata[i*7+6];
        target_poses.push_back(targ_pose);
        //cout<<"targ_pose "<<i+1<<":"<<targ_pose<<endl;
    }


    HandEyeCalibration cmc(num_cols,num_rows,dist,target_poses);

    return 0;
}
