#pragma once

//c++ 标准库
#include <fstream>
#include <vector>
#include <iostream>

//  opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

// OpenCV 特征检测模块---opencv2.4.13_nonfree模块

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//  PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

typedef struct CAMERA_INTRINSIC_PARAMETERS
{
  double cx, cy, fx, fy, scale;
}CAMERA_INTRINSIC_PARAMETERS;

typedef struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
}FRAME;

typedef struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
}RESULT_OF_PNP;

void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// function  interface
// image2PointCloud: turn rgb_picture into pointCloud
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d: Change the single image points into 3D points;
// input: 3D-Points[image points(u,v,d)], Point3f(u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);


// 参数读取
class ParameterReader
{
public:
    ParameterReader( string filename="../config/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};













