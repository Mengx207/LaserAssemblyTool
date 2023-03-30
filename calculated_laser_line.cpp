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

    // Read laser plane parameters
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
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mat img (1080,1440, CV_8UC3);

    // find laser plane by rmatrix and tvector in camera frame
    laserline::laser_plane laser_1, laser_2, laser_3;
    laser_1 = laserline::laserPlane(rmatrix_laser_1_values, tvec_laser_1_values);
    laser_2 = laserline::laserPlane(rmatrix_laser_2_values, tvec_laser_2_values);
    laser_3 = laserline::laserPlane(rmatrix_laser_3_values, tvec_laser_3_values);

    // find intersection point between laser beam and the target board plane
    Point3f interPoint1, interPoint2, interPoint3; 
    interPoint1 = laserline::intersectionPoint(laser_1.P0, laser_1.laserbeam, target.first, target.second);
	interPoint2 = laserline::intersectionPoint(laser_2.P0, laser_2.laserbeam, target.first, target.second);
	interPoint3 = laserline::intersectionPoint(laser_3.P0, laser_3.laserbeam, target.first, target.second);

    cout<<endl<<"Intersection point between laser beam 1 and target board: "<< Point3d(interPoint1.x, interPoint1.y, interPoint1.z) << endl;
    cout<<endl<<"Intersection point between laser beam 2 and target board: "<< Point3d(interPoint2.x, interPoint2.y, interPoint2.z) << endl;
    cout<<endl<<"Intersection point between laser beam 3 and target board: "<< Point3d(interPoint3.x, interPoint3.y, interPoint3.z) << endl;
    
    // find intersection line between the target board plane and the laser plane in camera frame
    laserline::intersection line1, line2, line3;
    line1 = laserline::intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});
    line2 = laserline::intersectionLine(target.first, laser_2.normalvector, target.second, vector<double>{interPoint2.x, interPoint2.y, interPoint2.z});
    line3 = laserline::intersectionLine(target.first, laser_3.normalvector, target.second, vector<double>{interPoint3.x, interPoint3.y, interPoint3.z});

    std::vector<cv::Point3d> laserline_points_1, laserline_points_2, laserline_points_3;

    for(double t=-100; t<=100;)
    {
        Point3d points_1((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
        Point3d points_2((line2.x0+line2.a*t), (line2.y0+line2.b*t), (line2.z0+line2.c*t));
        Point3d points_3((line3.x0+line3.a*t), (line3.y0+line3.b*t), (line3.z0+line3.c*t));
        t = t+10;
        //cout<<"point1: "<<points<<endl;
        laserline_points_1.push_back(points_1);
        laserline_points_2.push_back(points_2);
        laserline_points_3.push_back(points_3);
    }

    // cout<<endl<<"laser line points in 3D space in camera frame:"<<endl<<laserline_points_1_<<endl;
    vector<Point2d> projectedlaserline_1,projectedlaserline_2,projectedlaserline_3;
    projectPoints(laserline_points_1, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_1);
    projectPoints(laserline_points_2, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_2);
    projectPoints(laserline_points_3, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_3);
    
    cout<<endl<<"points on laser line:"<<endl<<projectedlaserline_1<<endl;


    // draw a line through points
    int thickness = 2;
    int lineType = cv::LINE_8;
    line( img, projectedlaserline_1[0], projectedlaserline_1[19], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    line( img, projectedlaserline_2[0], projectedlaserline_2[19], cv::Scalar( 0, 255, 0 ), thickness, lineType );
    line( img, projectedlaserline_3[0], projectedlaserline_3[19], cv::Scalar( 255, 0, 0 ), thickness, lineType );
    
    // draw the intersection point between laser beam and target board
    vector<Point3d> interPointArray;
    vector<Point2d> projectedInterPoints;
    interPointArray.push_back(interPoint1);
    interPointArray.push_back(interPoint2);
    interPointArray.push_back(interPoint3);
    projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);

    cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<projectedInterPoints<<endl;
    cv::circle( img, projectedInterPoints[0], 3, cv::Scalar(0,0,255), -1, 8, 0 );
    cv::circle( img, projectedInterPoints[1], 5, cv::Scalar(0,255,0), 2, 8, 0 );
    cv::circle( img, projectedInterPoints[2], 7, cv::Scalar(255,0,0), 2, 8, 0 );

	//cv::circle( img, Point2d(720, 540), 5, cv::Scalar(0,0,255), -1, 8, 0 );
    cv::imshow("Image",img);
    waitKey();
    return EXIT_SUCCESS;
}
