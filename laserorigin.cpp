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
	string path_rmatrix = "values/rmatrix_1.txt";
	string path_tvec = "values/tvec_1.txt";
	if(argv[1] == string("1")) 
	{
		path_rmatrix = "values/rmatrix_1.txt";
		path_tvec = "values/tvec_1.txt";
	}
	if(argv[1] == string("2")) 
	{
		path_rmatrix = "values/rmatrix_2.txt";
		path_tvec = "values/tvec_2.txt";
	}
	if(argv[1] == string("3")) 
	{
		path_rmatrix = "values/rmatrix_3.txt";
		path_tvec = "values/tvec_3.txt";
	}
	if(argv[1] == string("4"))
	{
		path_rmatrix = "values/rmatrix_4.txt";
		path_tvec = "values/tvec_4.txt";
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

	vector<Point2d> imgPoint_vector;
	Point2d imgPoint;
	ifstream readPoint("values/intersections/intersections_d1.txt");
	// while (readPoint >> imgPoint)
	// {
	// 	imgPoint_vector.push_back(imgPoint);
	// }

	// ifstream readPoint("values/intersections/intersections_d1.txt");
	// while (readPoint >> imgPoint)
	// {
	// 	imgPoint_vector.push_back(imgPoint);
	// }

	// ifstream readPoint("values/intersections/intersections_d1.txt");
	// while (readPoint >> imgPoint)
	// {
	// 	imgPoint_vector.push_back(imgPoint);
	// }
	// cout<<endl<<"read imgPoint: "<<imgPoint_vector<<endl;
	
	//Point3d p2 (-13.93, -15.71, 355.31);
	//general::lineEquation(p1,p2,tvec_laser_values);
}