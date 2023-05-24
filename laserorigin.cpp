/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/
#include <iostream>
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
using namespace GENAPI_NAMESPACE;

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif


int main(int argc, char* argv[])
{
	string path_rmatrix = "values/rmatrix_1_newmount.txt";
	string path_tvec = "values/tvec_1_newmount.txt";
	if(argv[1] == string("1")) 
	{
		path_rmatrix = "values/rmatrix_1_newmount.txt";
		path_tvec = "values/tvec_1_newmount.txt";
	}
	if(argv[1] == string("2")) 
	{
		path_rmatrix = "values/rmatrix_2_newmount.txt";
		path_tvec = "values/tvec_2_newmount.txt";
	}
	if(argv[1] == string("3")) 
	{
		path_rmatrix = "values/rmatrix_3_newmount.txt";
		path_tvec = "values/tvec_3_newmount.txt";
	}
	if(argv[1] == string("4"))
	{
		path_rmatrix = "values/rmatrix_4_newmount.txt";
		path_tvec = "values/tvec_4_newmount.txt";
	}

	vector<double> tvec_laser_values;
	vector<double> rmatrix_laser_values;
	double val;

	ifstream rmatrixL(path_rmatrix);
	while (rmatrixL >> val)
	{
		rmatrix_laser_values.push_back(val);
	}
	ifstream tvecL(path_tvec);
	while (tvecL >> val)
	{
		tvec_laser_values.push_back(val*1000);
	}

	vector<double> imgPoint_d1_vector, imgPoint_d2_vector, imgPoint_d3_vector;

	ifstream readPointd1("values/intersections/intersections_d1.txt");
	while (readPointd1 >> val)
	{
		imgPoint_d1_vector.push_back(val);
	}
	Point2d imgPoint_d1;
	imgPoint_d1.x = imgPoint_d1_vector[0];
	imgPoint_d1.y = imgPoint_d1_vector[1];

	ifstream readPointd2("values/intersections/intersections_d2.txt");
	while (readPointd2 >> val)
	{
		imgPoint_d2_vector.push_back(val);
	}
	Point2d imgPoint_d2;
	imgPoint_d2.x = imgPoint_d2_vector[0];
	imgPoint_d2.y = imgPoint_d2_vector[1];

	ifstream readPointd3("values/intersections/intersections_d3.txt");
	while (readPointd3 >> val)
	{
		imgPoint_d3_vector.push_back(val);
	}
	Point2d imgPoint_d3;
	imgPoint_d3.x = imgPoint_d3_vector[0];
	imgPoint_d3.y = imgPoint_d3_vector[1];

	// cout<<endl<<"read imgPoint: "<<imgPoint_vector<<endl;
	laserline::solvePnP_result solvePnP_result_d1,solvePnP_result_d2,solvePnP_result_d3;
	Mat image_captured = imread("images/pattern.png", IMREAD_GRAYSCALE);
	Size patternSize (5,3);

	double squareSize = 6.75;
	solvePnP_result_d2 = laserline::getRvecTvec(image_captured,patternSize,squareSize);

	squareSize = 7.75;
	solvePnP_result_d3 = laserline::getRvecTvec(image_captured,patternSize,squareSize);

	squareSize = 5.75;
	solvePnP_result_d1 = laserline::getRvecTvec(image_captured,patternSize,squareSize);

    Point3d p1 = general::locationCam2Target( imgPoint_d1, solvePnP_result_d1);
	Point3d p2 = general::locationCam2Target( imgPoint_d2, solvePnP_result_d2);
	Point3d p3 = general::locationCam2Target( imgPoint_d3, solvePnP_result_d3);
	cout<<endl<<"3 points: "<<p1<<" "<<p2<<" "<<p3<<endl;
	general::lineEquation(p1,p3,tvec_laser_values);
}