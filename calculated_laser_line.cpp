/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/
#include <iostream>
#include <utility>
#include <sstream>
#include <string>
#include <unistd.h>
#include <pylon/InstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <GenApi/IEnumeration.h>
#include <pylon/EnumParameter.h>
#include <Base/GCString.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <bits/stdc++.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "include/softwaretriggerconfiguration.h"
#include <time.h>
#include <stdio.h>
#include <ctime>

#include "include/utility.h"

using namespace Pylon;
using namespace std;
using namespace cv;
using namespace GENAPI_NAMESPACE;
// struct intersection{
//     double x,y;
//     double a,b,c;
// };
// typedef struct laser Struct;

int main(int argc, char **argv)
{
    // gain rmatrix and tvec from target board to cam
    ifstream intrin("values/intrinsic.txt");
    vector<double> cameraMatrix_values;
    double val;
    while (intrin >> val)
    {
        cameraMatrix_values.push_back(val);
    }
    ifstream dist("values/distortion.txt");
    vector<double> distCoeffs_values;
    while (dist >> val)
    {
        distCoeffs_values.push_back(val);
    }

    Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
    // Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());
    Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());

    pair<Mat,Mat>vec = laserline::getRvecTvec(); // rmatrix and tvect from target board to cam

    // find target board plane in cam frame
    pair<vector<double>,vector<double>>target = laserline::targetBoardPlane(vec.first, vec.second);
    
    string path_laser_rmatrix = "values/rmatrix_laser_1.txt";
    string path_laser_tvec = "values/tvec_laser_1.txt";

    // find laser plane in cam frame
    // read laser 1
    ifstream rmatrix_1(path_laser_rmatrix);
    vector<double> rmatrix_laser_1_values;
    while (rmatrix_1 >> val)
    {
        rmatrix_laser_1_values.push_back(val);
    }
    ifstream tvecL_1(path_laser_tvec);
    vector<double> tvec_laser_1_values;
    while (tvecL_1 >> val)
    {
        tvec_laser_1_values.push_back(val);
    }
    // read laser 2
    ifstream rmatrix_2("values/rmatrix_laser_2.txt");
    vector<double> rmatrix_laser_2_values;
    while (rmatrix_2 >> val)
    {
        rmatrix_laser_2_values.push_back(val);
    }
    ifstream tvecL_2("values/tvec_laser_2.txt");
    vector<double> tvec_laser_2_values;
    while (tvecL_2 >> val)
    {
        tvec_laser_2_values.push_back(val);
    }
    // read laser 3
    ifstream rmatrix_3("values/rmatrix_laser_3.txt");
    vector<double> rmatrix_laser_3_values;
    while (rmatrix_3 >> val)
    {
        rmatrix_laser_3_values.push_back(val);
    }
    ifstream tvecL_3("values/tvec_laser_3.txt");
    vector<double> tvec_laser_3_values;
    while (tvecL_3 >> val)
    {
        tvec_laser_3_values.push_back(val);
    }
    // find laser plane
    laserline::laser_plane laser_1, laser_2, laser_3;
    laser_1 = laserline::laserPlane(rmatrix_laser_1_values, tvec_laser_1_values);
    laser_2 = laserline::laserPlane(rmatrix_laser_2_values, tvec_laser_2_values);
    laser_3 = laserline::laserPlane(rmatrix_laser_3_values, tvec_laser_3_values);

    // find intersection line between target board plane and laser plane in cam frame
    laserline::intersection line1, line2, line3;
    line1 = laserline::intersectionLine(target.first, laser_1.N_L, target.second, laser_1.P1);
    line2 = laserline::intersectionLine(target.first, laser_2.N_L, target.second, laser_2.P1);
    line3 = laserline::intersectionLine(target.first, laser_3.N_L, target.second, laser_3.P1);

    Mat img (1080,1440, CV_8UC3);
    std::vector<cv::Point2d> laserPoints_1, laserPoints_2, laserPoints_3;

    for(int t=-2000; t<=2000;)
    {
        // Point2d points((69.2212+(-0.499939)*t)+500, -(-121.744+0.850974*t)+500);
        Point2d points((line1.x+line1.a*t)+720, -(line1.y+line1.b*t)+540);
        t = t+100;
        //cout<<"point1: "<<points<<endl;
        laserPoints_1.push_back(points);
    }

    for(int t=-2000; t<=2000;)
    {
        Point2d points((line2.x+line2.a*t)+720, -(line2.y+line2.b*t)+540);
        t = t+100;
        //cout<<"point2: "<<points<<endl;
        laserPoints_2.push_back(points);
    }

    for(int t=-2000; t<=2000;)
    {
        Point2d points((line3.x+line3.a*t)+720, -(line3.y+line3.b*t)+540);
        t = t+100;
        //cout<<"point3: "<<points<<endl;
        laserPoints_3.push_back(points);
    }

    // draw circles on each points
    // for (int n=0; n<100;)
    // {
    //     cv::circle(img,laserPoints_1[n],5,cv::Scalar(0,0,255),-1,8,0);
    //     n = n+10;
    // }
    // draw a line through points
    int thickness = 2;
    int lineType = cv::LINE_8;
    line( img, laserPoints_1[0], laserPoints_1[40], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    line( img, laserPoints_2[0], laserPoints_2[40], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    line( img, laserPoints_3[0], laserPoints_3[40], cv::Scalar( 0, 0, 255 ), thickness, lineType );

    Point3f interPoint1, interPoint2, interPoint3;
    interPoint1 = laserline::intersectionPoint(laser_1.P0, laser_1.C_L, target.first, target.second);
	interPoint2 = laserline::intersectionPoint(laser_2.P0, laser_2.C_L, target.first, target.second);
	interPoint3 = laserline::intersectionPoint(laser_3.P0, laser_3.C_L, target.first, target.second);
	cv::circle( img, Point2d(interPoint1.x+720, -interPoint1.y+540), 5, cv::Scalar(255,0,255), -1, 8, 0 );
	cv::circle( img, Point2d(interPoint2.x+720, -interPoint2.y+540), 5, cv::Scalar(0,255,0), -1, 8, 0 );
    // cv::circle( img, Point2d(interPoint3.x+720, -interPoint3.y+540), 5, cv::Scalar(0,255,0), -1, 8, 0 );
    cout<<endl<<"intersection point: "<< Point3d(interPoint1.x, interPoint1.y, interPoint1.z) << endl<<endl;

    vector<Point3d> interPointArray;
    vector<Point2d> projectedInterPoints;
    interPointArray.push_back(interPoint1);
    projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);
    cout<<endl<<"inter point: "<<projectedInterPoints<<endl;
    cv::circle( img, projectedInterPoints[0], 5, cv::Scalar(0,0,255), -1, 8, 0 );

	cv::circle( img, Point2d(720, 540), 5, cv::Scalar(0,0,255), -1, 8, 0 );
    cv::imshow("Image",img);
    waitKey();
    return EXIT_SUCCESS;
}
