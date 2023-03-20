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
    pair<Mat,Mat>vec = laserline::getRvecTvec();
    string path_rmatrix = "values/rmatrix_laser_1.txt";
    string path_tvec = "values/tvec_laser_1.txt";

    // find laser plane in cam frame
    double val;
    // read laser 1
    // ifstream rmatrix_1("values/rmatrix_laser_1.txt");
    ifstream rmatrix_1(path_rmatrix);
    vector<double> rmatrix_laser_1_values;
    while (rmatrix_1 >> val)
    {
        rmatrix_laser_1_values.push_back(val);
    }
    ifstream tvecL_1("values/tvec_laser_1.txt");
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
    pair<vector<double>,vector<double>>laser_1 = laserline::laserPlane(rmatrix_laser_1_values, tvec_laser_1_values);
    pair<vector<double>,vector<double>>laser_2 = laserline::laserPlane(rmatrix_laser_2_values, tvec_laser_2_values);
    pair<vector<double>,vector<double>>laser_3 = laserline::laserPlane(rmatrix_laser_3_values, tvec_laser_3_values);

    // find target board plane in cam frame
    pair<vector<double>,vector<double>>target = laserline::targetBoardPlane(vec.first, vec.second);

    // find intersection line between target board plane and laser plane in cam frame
    laserline::intersection line1, line2, line3;
    line1 = laserline::intersectionLine(target.first, laser_1.first, target.second, laser_1.second);
    line2 = laserline::intersectionLine(target.first, laser_2.first, target.second, laser_2.second);
    line3 = laserline::intersectionLine(target.first, laser_3.first, target.second, laser_3.second);

    Mat img (1000,1000, CV_8UC3);
    std::vector<cv::Point2d> laserPoints_1, laserPoints_2, laserPoints_3;

    for(int t=-2000; t<2000;)
    {
        t = t+40;
        // Point2d points((69.2212+(-0.499939)*t)+500, -(-121.744+0.850974*t)+500);
        Point2d points((line1.x+line1.a*t)+500, -(line1.y+line1.b*t)+500);
        // cout<<"point: "<<points<<endl;
        laserPoints_1.push_back(points);
    }

    for(int t=-2000; t<2000;)
    {
        t = t+40;
        Point2d points((line2.x+line2.a*t)+500, -(line2.y+line2.b*t)+500);
        laserPoints_2.push_back(points);
    }

    for(int t=-2000; t<2000;)
    {
        t = t+40;
        Point2d points((line3.x+line3.a*t)+500, -(line3.y+line3.b*t)+500);
        laserPoints_3.push_back(points);
    }

    // draw circles on each points
    for (int n=0; n<100;)
    {
        cv::circle(img,laserPoints_1[n],5,cv::Scalar(0,0,255),-1,8,0);
        n = n+10;
    }
    // draw a line through points
    int thickness = 2;
    int lineType = cv::LINE_8;
    line( img, laserPoints_1[0], laserPoints_1[90], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    line( img, laserPoints_2[0], laserPoints_2[90], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    line( img, laserPoints_3[0], laserPoints_3[90], cv::Scalar( 0, 0, 255 ), thickness, lineType );
    cv::imshow("Image",img);
    waitKey();
    return EXIT_SUCCESS;
}
