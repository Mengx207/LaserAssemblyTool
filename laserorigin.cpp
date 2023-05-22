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

	vector<double> imgPoint_vector;
	ifstream readPoint("values/intersections/intersections_d2_test.txt");
	while (readPoint >> val)
	{
		imgPoint_vector.push_back(val);
	}
	Point2d imgPoint;
	imgPoint.x = imgPoint_vector[0];
	imgPoint.y = imgPoint_vector[1];

	// cout<<endl<<"read imgPoint: "<<imgPoint_vector<<endl;
	laserline::solvePnP_result solvePnP_result;
	solvePnP_result = laserline::getRvecTvec();
    Point3d p1 = general::locationCam2Target( imgPoint, solvePnP_result);
	Point3d p2 (-13.93, -15.71, 355.31);
	general::lineEquation(p1,p2,tvec_laser_values);
}