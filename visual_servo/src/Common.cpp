#include<Common.h>

#define M_pi 3.141592653
using namespace std;

void Quat2RotMat(double *q, cv::Mat* m)
{
    double x = q[0];
    double y = q[1];
    double z = q[2];
    double w = q[3];
    m->at<double>(0, 0) = 1 - 2 * y*y - 2 * z*z;
    m->at<double>(1, 0) = 2 * x*y + 2 * w*z;
    m->at<double>(2, 0) = 2 * x*z - 2 * w*y;
    m->at<double>(0, 1) = 2 * x*y - 2 * w*z;
    m->at<double>(1, 1) = 1 - 2 * x*x - 2 * z*z;
    m->at<double>(2, 1) = 2 * z*y + 2 * w*x;
    m->at<double>(0, 2) = 2 * x*z + 2 * w*y;
    m->at<double>(1, 2) = 2 * y*z - 2 * w*x;
    m->at<double>(2, 2) = 1 - 2 * x*x - 2 * y*y;
}

vector<double> RotMat2Eular(cv::Mat *m)
{

    vector<double> theta;
    double theta_x, theta_y, theta_z;
    theta_x = atan2(m->at<double>(2, 1), m->at<double>(2, 2));
    theta_y = atan2(-m->at<double>(2, 0),sqrt(m->at<double>(2, 1)*m->at<double>(2, 1) + m->at<double>(2, 2)*m->at<double>(2, 2)));
    theta_z = atan2(m->at<double>(1, 0), m->at<double>(0, 0));
    theta.push_back(theta_x);
    theta.push_back(theta_y);
    theta.push_back(theta_z);
    return theta;
}
