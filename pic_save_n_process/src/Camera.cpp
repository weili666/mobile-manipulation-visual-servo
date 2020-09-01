#include <Camera.h>

int num_l = 0;
int num_r = 0;
struct index_
{
    int a1, a2, a3;
};

int _sign_(double x)
{
    if (x >= 0)
    {
        return 1;
    }
    if (x < 0)
    {
        return -1;
    }
}

Camera::Camera(ros::NodeHandle& nh, const cv::Mat& cameraMatrix_, const cv::Mat& distCoeffs_)
{
    nh_=nh;
    cameraMatrix=cameraMatrix_;
    distCoeffs=distCoeffs_;
    camera_se3_pub = nh_.advertise<common::mark_vision>("camera_se3_talker_msg",1000);

    ros::Subscriber pic_sub_l = nh_.subscribe("/camera/left/image_raw", 10, &Camera::PicSaveNProcessLeft, this);
    ros::Subscriber pic_sub_r = nh_.subscribe("/camera/right/image_raw", 10, &Camera::PicSaveNProcessRight, this);
    //ros::Rate loop_rate(1);
    ros::spin();
}

Point Center_cal( vector< vector<Point> > contours, int i )
{
    Point center;
    Point cont;
    vector<Point> contoursi = contours[i];
    cont = contoursi[0];
    vector<Point>::iterator it;
    for(it = contoursi.begin();it != contoursi.end();it++)
    {
        center.x += (*it).x;
        center.y += (*it).y;
    }
    center.x /= contoursi.size();
    center.y /= contoursi.size();
    return center;
}

void RotMat2Quat(cv::Mat* m, double *q)
{
    double q0, q1, q2, q3;
    q0 = 0.5*sqrt(1 + m->at<double>(0, 0) + m->at<double>(1, 1) + m->at<double>(2, 2));
    q1 = 0.5*sqrt(1 + m->at<double>(0, 0) - m->at<double>(1, 1) - m->at<double>(2, 2));
    q2 = 0.5*sqrt(1 - m->at<double>(0, 0) + m->at<double>(1, 1) - m->at<double>(2, 2));
    q3 = 0.5*sqrt(1 - m->at<double>(0, 0) - m->at<double>(1, 1) + m->at<double>(2, 2));
    q[3] = q0;
    q[0] = _sign_(q0)*_sign_(m->at<double>(2, 1) - m->at<double>(1, 2))*q1;
    q[1] = _sign_(q0)*_sign_(m->at<double>(0, 2) - m->at<double>(2, 0))*q2;
    q[2] = _sign_(q0)*_sign_(m->at<double>(1, 0) - m->at<double>(0, 1))*q3;
}

std::vector<Point> Camera::GetCornerFromPic(const Mat &picture_)
{
    //彩色图转灰度图
    Mat src_gray = picture_.clone();
    //对图像进行平滑处理
    blur( src_gray, src_gray, Size(3,3) );
    //使灰度图象直方图均衡化
    equalizeHist( src_gray, src_gray );

    //指定112阀值进行二值化
    Mat threshold_output;
    threshold( src_gray, threshold_output, 112, 255, THRESH_BINARY );

    //需要的变量定义
    //Scalar color = Scalar(1,1,255 );
    vector< vector<Point> > contours,contours2;
    vector<Vec4i> hierarchy;
    Mat drawing = Mat::zeros( src_gray.size(), CV_8UC3 );
    vector<index_> vin;
    //利用二值化输出寻找轮廓
    findContours(threshold_output, contours, hierarchy,  CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0) );
    //寻找轮廓的方法
    int tempindex1 = 0;
    int tempindex2 = 0;

    cout<<"contours size is:"<<contours.size()<<endl;
    for(int i = 0;i<contours.size();i++)
    {
        if(hierarchy[i][2] == -1)
            continue;
        else
            tempindex1 = hierarchy[i][2];                //第一个子轮廓的索引

        if(hierarchy[tempindex1][2] == -1)
            continue;
        else
        {
            tempindex2 = hierarchy[tempindex1][2];        //第二个子轮廓的索引
            //记录搜索到的有两个子轮廓的轮廓并把他们的编号存储
            index_ in;
            in.a1 = i;
            in.a2 = tempindex1;
            in.a3 = tempindex2;
            vin.push_back(in);
        }
    }
    //按面积比例搜索
    cout<<"vin size is :"<<vin.size()<<endl;
    vin_size.push_back(vin.size());
    vector<index_>::iterator it;
    for(it = vin.begin();it != vin.end();)
    {
        vector<Point> out1Contours = contours[it->a1];
        vector<Point> out2Contours = contours[it->a2];
        double lenth1 = arcLength(out1Contours,1);
        double lenth2 = arcLength(out2Contours,1);
        if(abs(lenth1/lenth2-2)>1)
        {
            it = vin.erase(it);
        }
        else
        {
            drawContours( drawing, contours, it->a1,  CV_RGB(255,255,255) , CV_FILLED, 8);
            it++;
        }
    }

    //获取三个定位角的中心坐标
    Point point[4];
    Point point_regular[4];
    int i = 0;
    int sum_xy[4];
    int sum_max = 0;
    int sum_min = 10000;
    int index_max;
    int index_min;
    int index_sec;
    int index_thi;
    vector<Point> pointreturn;
    vector<int> index_middle;
    for(it = vin.begin(),i = 0;it != vin.end();i++,it++)
    {
        point[i] = Center_cal( contours, it->a1 );
        sum_xy[i] = point[i].x + point[i].y;
        if (sum_max < sum_xy[i])
        {
            sum_max = sum_xy[i];
            index_max = i;
        }
        if (sum_min > sum_xy[i])
        {
            sum_min = sum_xy[i];
            index_min = i;
        }
    }
    for(int i = 0; i < 4; i ++)
    {
        if((i!=index_max)&&(i!=index_min))
        {
            index_middle.push_back(i);
        }
    }
    if((point[index_middle[0]].x < point[index_middle[1]].x)||(point[index_middle[0]].y > point[index_middle[1]].y))
    {
        index_sec = index_middle[0];
        index_thi = index_middle[1];
    }
    else
    {
        index_sec = index_middle[1];
        index_thi = index_middle[0];
    }

    pointreturn.push_back(point[index_max]);
    pointreturn.push_back(point[index_sec]);
    pointreturn.push_back(point[index_thi]);
    pointreturn.push_back(point[index_min]);

    cout<<"pointreturn size is:"<<pointreturn.size()<<endl;

     cout<<"point[0].x:"<<pointreturn[0].x<<",point[0].y:"<<pointreturn[0].y<<endl;
     cout<<"point[1].x:"<<pointreturn[1].x<<",point[1].y:"<<pointreturn[1].y<<endl;
     cout<<"point[2].x:"<<pointreturn[2].x<<",point[2].y:"<<pointreturn[2].y<<endl;
     cout<<"point[3].x:"<<pointreturn[3].x<<",point[3].y:"<<pointreturn[3].y<<endl;

     return pointreturn;

}

void Camera::PicSaveNProcessLeft(const sensor_msgs::Image& msg)
{
    cout<<"//*****************begin to save and process left image!********************//"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);
    this->img = image_gray;

    num_l++;
    char buff[100];
    sprintf(buff, "/home/weili/catkin_ws/left_images/saved_left_%d.jpg",num_l);
    cv::imwrite(buff, image_gray);

    this->vin_size.clear();

}

void Camera::PicSaveNProcessRight(const sensor_msgs::Image& msg)
{
    cout<<"//*****************begin to save and process right image!********************//"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    common::mark_vision mark_msgs;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);
    this->img2 = image_gray;

    num_r++;
    char buff[100];
    sprintf(buff, "/home/weili/catkin_ws/right_images/saved_right_%d.jpg",num_r);
    cv::imwrite(buff, image_gray);

    picture1 = this->img;
    picture2 = this->img2;
    cout<<"picture1 size:"<<picture1.size()<<endl;
    cout<<"picture2 size:"<<picture2.size()<<endl;
    //=========================calculate PnP===========================//
    std::vector<Point> scene_corners_l;
    std::vector<Point> scene_corners_r;
    scene_corners_l = GetCornerFromPic(picture1);
    scene_corners_r = GetCornerFromPic(picture2);

    std::vector<Point2f> scene_corners_l_(4);
    std::vector<Point2f> scene_corners_r_(4);
    geometry_msgs::Point scene_corners_1;
    geometry_msgs::Point scene_corners_2;
    geometry_msgs::Point scene_corners_3;
    geometry_msgs::Point scene_corners_4;

    for(int i = 0; i < scene_corners_l.size(); i ++)
    {
        scene_corners_l_[i].x = scene_corners_l[i].x;
        scene_corners_l_[i].y = scene_corners_l[i].y;
        scene_corners_r_[i].x = scene_corners_r[i].x;
        scene_corners_r_[i].y = scene_corners_r[i].y;
    }

    Mat Ho2e_(4, 4, CV_64FC1);
    Mat Hw2o_ = cv::Mat::eye(4, 4, CV_64FC1);;

    Mat r1(3, 1, CV_64FC1);
    Mat r2(3, 1, CV_64FC1);
    Mat r(3, 1, CV_64FC1);
    Mat t1(3, 1, CV_64FC1);
    Mat t2(3, 1, CV_64FC1);
    Mat t(3, 1, CV_64FC1);
    vector<Point3f> pts_3d(4);
    double dx = 0.192;//the width of the object;
    double dy = 0.192;//the height of the object;
    double H = 0.90;
    double dz = 0.192;//the length of the grasp;
    pts_3d[0] = Point3f(dx/2,dy/2,0);
    pts_3d[1] = Point3f(-dx/2,dy/2,0);//Point3f(0.113,0,0);
    pts_3d[2] = Point3f(dx/2,-dy/2,0);//Point3f(0.113,0.063,0);
    pts_3d[3] = Point3f(-dx/2,-dy/2,0);//Point3f(0,0.063,0);

    solvePnP(pts_3d,scene_corners_l_,cameraMatrix,distCoeffs,r1,t1,false);
    solvePnP(pts_3d,scene_corners_r_,cameraMatrix,distCoeffs,r2,t2,false);
    r = (r1 + r2) / 2.0;
    t = (t1 + t2) / 2.0;
    Mat R(3, 3, CV_64FC1);
    Mat R1(3, 3, CV_64FC1);
    Mat R2(3, 3, CV_64FC1);

    cv::Mat s_1(3, 1, CV_64FC1);
    cv::Mat s_2(3, 1, CV_64FC1);
    cv::Mat s_3(3, 1, CV_64FC1);
    cv::Mat s_4(3, 1, CV_64FC1);
    cv::Mat z_1(1, 1, CV_64FC1);
    cv::Mat z_2(1, 1, CV_64FC1);
    cv::Mat z_3(1, 1, CV_64FC1);
    cv::Mat z_4(1, 1, CV_64FC1);
    cv::Mat delta_p1(3, 1, CV_64FC1);
    cv::Mat delta_p2(3, 1, CV_64FC1);
    cv::Mat delta_p3(3, 1, CV_64FC1);
    cv::Mat delta_p4(3, 1, CV_64FC1);
    cv::Mat z_vec(3, 1, CV_64FC1);
    double x_M = 0.0955;
    double y_M = 0.096;
    s_1.at<double>(0, 0) = x_M; s_1.at<double>(1, 0) = y_M; s_1.at<double>(2, 0) = 0;
    s_2.at<double>(0, 0) = -x_M; s_2.at<double>(1, 0) = y_M; s_2.at<double>(2, 0) = 0;
    s_3.at<double>(0, 0) = x_M; s_3.at<double>(1, 0) = -y_M; s_3.at<double>(2, 0) = 0;
    s_4.at<double>(0, 0) = -x_M; s_4.at<double>(1, 0) = -y_M; s_4.at<double>(2, 0) = 0;

    cv::Rodrigues(r,R);
    cv::Rodrigues(r1,R1);
    cv::Rodrigues(r2,R2);


    cout<<"===========================PNP_RESULT=========================="<<endl;
    cout<<"r:"<<r<<endl<<"R:"<<R<<endl<<"t:"<<t<<endl;
    cout<<"r1:"<<r1<<endl<<"R1:"<<R1<<endl<<"t1:"<<t1<<endl;
    cout<<"r2:"<<r2<<endl<<"R2:"<<R2<<endl<<"t2:"<<t2<<endl;
    Hw2o_.at<double>(1, 1) = 0.0;
    Hw2o_.at<double>(1, 2) = 1.0;
    Hw2o_.at<double>(2, 1) = -1.0;
    Hw2o_.at<double>(2, 2) = 0.0;
    Hw2o_.at<double>(2, 3) = H;

    if((vin_size[0]==4)&&(vin_size[1]==4))
    {
        R_camera = R.inv();
        T_camera = -R.inv()*t;

        R_camera.copyTo(Ho2e_(Rect(0, 0, 3, 3)));
        T_camera.copyTo(Ho2e_(Rect(3, 0, 1, 3)));
        Ho2e_.at<double>(3, 0) = 0.0;
        Ho2e_.at<double>(3, 1) = 0.0;
        Ho2e_.at<double>(3, 2) = 0.0;
        Ho2e_.at<double>(3, 3) = 1.0;

        Camera_SE3 = Hw2o_*Ho2e_;
        cout<<"Ho2e:"<<Ho2e_<<endl;
        cout<<"Camera_SE3:"<<Camera_SE3<<endl;
        geometry_msgs::Pose camera_pose;
        double q[4];

        //------------------calc the point position of marker point in camera-----------------//

        scene_corners_1.x = (scene_corners_l[0].x + scene_corners_r[0].x) / 2.0;
        scene_corners_2.x = (scene_corners_l[1].x + scene_corners_r[1].x) / 2.0;
        scene_corners_3.x = (scene_corners_l[2].x + scene_corners_r[2].x) / 2.0;
        scene_corners_4.x = (scene_corners_l[3].x + scene_corners_r[3].x) / 2.0;

        scene_corners_1.y = (scene_corners_l[0].y + scene_corners_r[0].y) / 2.0;
        scene_corners_2.y = (scene_corners_l[1].y + scene_corners_r[1].y) / 2.0;
        scene_corners_3.y = (scene_corners_l[2].y + scene_corners_r[2].y) / 2.0;
        scene_corners_4.y = (scene_corners_l[3].y + scene_corners_r[3].y) / 2.0;

        z_vec.at<double>(0, 0) = R_camera.at<double>(0, 2);
        z_vec.at<double>(1, 0) = R_camera.at<double>(1, 2);
        z_vec.at<double>(2, 0) = R_camera.at<double>(2, 2);

        delta_p1 = s_1 - T_camera;
        z_1 = delta_p1.t()*z_vec;
        delta_p2 = s_2 - T_camera;
        z_2 = delta_p2.t()*z_vec;
        delta_p3 = s_3 - T_camera;
        z_3 = delta_p3.t()*z_vec;
        delta_p4 = s_4 - T_camera;
        z_4 = delta_p4.t()*z_vec;

        scene_corners_1.z = z_1.at<double>(0, 0);
        scene_corners_2.z = z_2.at<double>(0, 0);
        scene_corners_3.z = z_3.at<double>(0, 0);
        scene_corners_4.z = z_4.at<double>(0, 0);

        //------------------------------------------------------------------------------------//

        camera_pose.position.x = Camera_SE3.at<double>(0, 3);
        camera_pose.position.y = Camera_SE3.at<double>(1, 3);
        camera_pose.position.z = Camera_SE3.at<double>(2, 3);
        RotMat2Quat(&Camera_SE3, q);
        camera_pose.orientation.x = q[0];
        camera_pose.orientation.y = q[1];
        camera_pose.orientation.z = q[2];
        camera_pose.orientation.w = q[3];

        mark_msgs.pose = camera_pose;
        mark_msgs.point1 = scene_corners_1;
        mark_msgs.point2 = scene_corners_2;
        mark_msgs.point3 = scene_corners_3;
        mark_msgs.point4 = scene_corners_4;
        mark_msgs.state = 0;
        camera_se3_pub.publish(mark_msgs);
    }
    else if((vin_size[0]==4)&&(vin_size[1]!=4))
    {
        cv::Mat Move_X = cv::Mat::eye(4, 4, CV_64FC1);
        Move_X.at<double>(0, 3) = 0.06;
        R_camera = R1.inv();
        T_camera = -R1.inv()*t1;

        R_camera.copyTo(Ho2e_(Rect(0, 0, 3, 3)));
        T_camera.copyTo(Ho2e_(Rect(3, 0, 1, 3)));
        Ho2e_.at<double>(3, 0) = 0.0;
        Ho2e_.at<double>(3, 1) = 0.0;
        Ho2e_.at<double>(3, 2) = 0.0;
        Ho2e_.at<double>(3, 3) = 1.0;

        Camera_SE3 = Hw2o_*Ho2e_*Move_X;
        cout<<"Ho2e:"<<Ho2e_<<endl;
        cout<<"Camera_SE3:"<<Camera_SE3<<endl;
        geometry_msgs::Pose camera_pose;
        double q[4];

        //------------------calc the point position of marker point in camera-----------------//

        scene_corners_1.x = scene_corners_l[0].x;
        scene_corners_2.x = scene_corners_l[1].x;
        scene_corners_3.x = scene_corners_l[2].x;
        scene_corners_4.x = scene_corners_l[3].x;

        scene_corners_1.y = scene_corners_l[0].y;
        scene_corners_2.y = scene_corners_l[1].y;
        scene_corners_3.y = scene_corners_l[2].y;
        scene_corners_4.y = scene_corners_l[3].y;

        z_vec.at<double>(0, 0) = R_camera.at<double>(0, 2);
        z_vec.at<double>(1, 0) = R_camera.at<double>(1, 2);
        z_vec.at<double>(2, 0) = R_camera.at<double>(2, 2);

        delta_p1 = s_1 - T_camera;
        z_1 = delta_p1.t()*z_vec;
        delta_p2 = s_2 - T_camera;
        z_2 = delta_p2.t()*z_vec;
        delta_p3 = s_3 - T_camera;
        z_3 = delta_p3.t()*z_vec;
        delta_p4 = s_4 - T_camera;
        z_4 = delta_p4.t()*z_vec;

        scene_corners_1.z = z_1.at<double>(0, 0);
        scene_corners_2.z = z_2.at<double>(0, 0);
        scene_corners_3.z = z_3.at<double>(0, 0);
        scene_corners_4.z = z_4.at<double>(0, 0);

        //------------------------------------------------------------------------------------//


        camera_pose.position.x = Camera_SE3.at<double>(0, 3);
        camera_pose.position.y = Camera_SE3.at<double>(1, 3);
        camera_pose.position.z = Camera_SE3.at<double>(2, 3);
        RotMat2Quat(&Camera_SE3, q);
        camera_pose.orientation.x = q[0];
        camera_pose.orientation.y = q[1];
        camera_pose.orientation.z = q[2];
        camera_pose.orientation.w = q[3];

        mark_msgs.pose = camera_pose;
        mark_msgs.point1 = scene_corners_1;
        mark_msgs.point2 = scene_corners_2;
        mark_msgs.point3 = scene_corners_3;
        mark_msgs.point4 = scene_corners_4;
        mark_msgs.state = 1;
        camera_se3_pub.publish(mark_msgs);

    }
    else if((vin_size[0]!=4)&&(vin_size[1]==4))
    {
        cv::Mat Move_X = cv::Mat::eye(4, 4, CV_64FC1);
        Move_X.at<double>(0, 3) = -0.06;
        R_camera = R2.inv();
        T_camera = -R2.inv()*t2;

        R_camera.copyTo(Ho2e_(Rect(0, 0, 3, 3)));
        T_camera.copyTo(Ho2e_(Rect(3, 0, 1, 3)));
        Ho2e_.at<double>(3, 0) = 0.0;
        Ho2e_.at<double>(3, 1) = 0.0;
        Ho2e_.at<double>(3, 2) = 0.0;
        Ho2e_.at<double>(3, 3) = 1.0;

        Camera_SE3 = Hw2o_*Ho2e_*Move_X;
        cout<<"Ho2e:"<<Ho2e_<<endl;
        cout<<"Camera_SE3:"<<Camera_SE3<<endl;
        geometry_msgs::Pose camera_pose;
        double q[4];

        //------------------calc the point position of marker point in camera-----------------//

        scene_corners_1.x = scene_corners_r[0].x;
        scene_corners_2.x = scene_corners_r[1].x;
        scene_corners_3.x = scene_corners_r[2].x;
        scene_corners_4.x = scene_corners_r[3].x;

        scene_corners_1.y = scene_corners_r[0].y;
        scene_corners_2.y = scene_corners_r[1].y;
        scene_corners_3.y = scene_corners_r[2].y;
        scene_corners_4.y = scene_corners_r[3].y;

        z_vec.at<double>(0, 0) = R_camera.at<double>(0, 2);
        z_vec.at<double>(1, 0) = R_camera.at<double>(1, 2);
        z_vec.at<double>(2, 0) = R_camera.at<double>(2, 2);

        delta_p1 = s_1 - T_camera;
        z_1 = delta_p1.t()*z_vec;
        delta_p2 = s_2 - T_camera;
        z_2 = delta_p2.t()*z_vec;
        delta_p3 = s_3 - T_camera;
        z_3 = delta_p3.t()*z_vec;
        delta_p4 = s_4 - T_camera;
        z_4 = delta_p4.t()*z_vec;

        scene_corners_1.z = z_1.at<double>(0, 0);
        scene_corners_2.z = z_2.at<double>(0, 0);
        scene_corners_3.z = z_3.at<double>(0, 0);
        scene_corners_4.z = z_4.at<double>(0, 0);

        //------------------------------------------------------------------------------------//


        camera_pose.position.x = Camera_SE3.at<double>(0, 3);
        camera_pose.position.y = Camera_SE3.at<double>(1, 3);
        camera_pose.position.z = Camera_SE3.at<double>(2, 3);
        RotMat2Quat(&Camera_SE3, q);
        camera_pose.orientation.x = q[0];
        camera_pose.orientation.y = q[1];
        camera_pose.orientation.z = q[2];
        camera_pose.orientation.w = q[3];

        mark_msgs.pose = camera_pose;
        mark_msgs.point1 = scene_corners_1;
        mark_msgs.point2 = scene_corners_2;
        mark_msgs.point3 = scene_corners_3;
        mark_msgs.point4 = scene_corners_4;
        mark_msgs.state = 2;
        camera_se3_pub.publish(mark_msgs);

    }
}
