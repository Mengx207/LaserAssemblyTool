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
	Mat image_captured_d1, image_captured_d2, image_captured_d3;
	image_captured_d1 = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
	image_captured_d2 = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);
	image_captured_d3 = imread("images/pattern_d3.png", IMREAD_GRAYSCALE);

	Size patternSize (5,3);
	double squareSize = 6.75;
	solvePnP_result_d2 = laserline::getRvecTvec(image_captured_d2,patternSize,squareSize);
	solvePnP_result_d3 = laserline::getRvecTvec(image_captured_d3,patternSize,squareSize);
	solvePnP_result_d1 = laserline::getRvecTvec(image_captured_d1,patternSize,squareSize);

    Point3d p1 = general::locationCam2Target( imgPoint_d1, solvePnP_result_d1);
	Point3d p2 = general::locationCam2Target( imgPoint_d2, solvePnP_result_d2);
	Point3d p3 = general::locationCam2Target( imgPoint_d3, solvePnP_result_d3);
	cout<<endl<<"3 points in camera frame: " << endl <<p1<<" "<<p2<<" "<<p3<<endl;
	general::lineEquation(p1,p3,tvec_laser_values);

	/*Laser plane verification----------------------------------------------------------------------------------------------------*/
	vector<Point3d> start_vector, end_vector;
	ifstream start_d1("values/laserlinetwopoints/start_d1.txt");
	ifstream start_d2("values/laserlinetwopoints/start_d2.txt");
	ifstream start_d3("values/laserlinetwopoints/start_d3.txt");
	ifstream end_d1("values/laserlinetwopoints/end_d1.txt");
	ifstream end_d2("values/laserlinetwopoints/end_d2.txt");
	ifstream end_d3("values/laserlinetwopoints/end_d3.txt");
	double x, y, z;
	char comma;
	
	while (start_d1 >> x >> comma >> y >> comma >> z)
	{
		start_vector.push_back(Point3d(x,y,z));
	}
	while (start_d2 >> x >> comma >> y >> comma >> z)
	{
		start_vector.push_back(Point3d(x,y,z));
	}
	while (start_d3 >> x >> comma >> y >> comma >> z)
	{
		start_vector.push_back(Point3d(x,y,z));
	}
	while (end_d1 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}
	while (end_d2 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}
	while (end_d3 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}
	cout<<endl<< "start_d1: "<< start_vector[0]<<endl;
	cout<<endl<< "start_d2: "<< start_vector[1]<<endl;
	cout<<endl<< "start_d3: "<< start_vector[2]<<endl;
	cout<<endl<< "end_d1: "<< end_vector[0]<<endl;
	cout<<endl<< "end_d2: "<< end_vector[1]<<endl;
	cout<<endl<< "end_d3: "<< end_vector[2]<<endl;
	vector<double> s1e1,s1e2,s1e3,s2e1,s2e2,s2e3,s3e1,s3e2,s3e3;
	s1e1.push_back(end_vector[0].x-start_vector[0].x);
	s1e1.push_back(end_vector[0].y-start_vector[0].y);
	s1e1.push_back(end_vector[0].z-start_vector[0].z);

	s1e2.push_back(end_vector[1].x-start_vector[0].x);
	s1e2.push_back(end_vector[1].y-start_vector[0].y);
	s1e2.push_back(end_vector[1].z-start_vector[0].z);

	s1e3.push_back(end_vector[2].x-start_vector[0].x);
	s1e3.push_back(end_vector[2].y-start_vector[0].y);
	s1e3.push_back(end_vector[2].z-start_vector[0].z);

	s2e1.push_back(end_vector[0].x-start_vector[1].x);
	s2e1.push_back(end_vector[0].y-start_vector[1].y);
	s2e1.push_back(end_vector[0].z-start_vector[1].z);

	s2e2.push_back(end_vector[1].x-start_vector[1].x);
	s2e2.push_back(end_vector[1].y-start_vector[1].y);
	s2e2.push_back(end_vector[1].z-start_vector[1].z);

	s2e3.push_back(end_vector[2].x-start_vector[1].x);
	s2e3.push_back(end_vector[2].y-start_vector[1].y);
	s2e3.push_back(end_vector[2].z-start_vector[1].z);

	s3e1.push_back(end_vector[0].x-start_vector[2].x);
	s3e1.push_back(end_vector[0].y-start_vector[2].y);
	s3e1.push_back(end_vector[0].z-start_vector[2].z);

	s3e2.push_back(end_vector[1].x-start_vector[2].x);
	s3e2.push_back(end_vector[1].y-start_vector[2].y);
	s3e2.push_back(end_vector[1].z-start_vector[2].z);

	s3e3.push_back(end_vector[2].x-start_vector[2].x);
	s3e3.push_back(end_vector[2].y-start_vector[2].y);
	s3e3.push_back(end_vector[2].z-start_vector[2].z);

	vector<double> N112 = laserline::crossProduct(s1e1,s1e2);
	auto max_it = max_element(begin(N112), end(N112));
	vector<double> N123 = laserline::crossProduct(s1e2,s1e3);
	vector<double> N113 = laserline::crossProduct(s1e1,s1e3);
	vector<double> N212 = laserline::crossProduct(s2e1,s2e2);
	vector<double> N223 = laserline::crossProduct(s2e2,s2e3);
	vector<double> N213 = laserline::crossProduct(s2e1,s2e3);
	vector<double> N312 = laserline::crossProduct(s3e1,s3e2);
	vector<double> N323 = laserline::crossProduct(s3e2,s3e3);
	vector<double> N313 = laserline::crossProduct(s3e1,s3e3);

	cout << endl << "N112: " << N112[0]/N112[2] << "," << N112[1]/N112[2] << "," << N112[2]/N112[2] << endl;
	cout << endl << "N123: " << N123[0]/N123[2] << "," << N123[1]/N123[2] << "," << N123[2]/N123[2] << endl;	
	cout << endl << "N113: " << N113[0]/N113[2] << "," << N113[1]/N113[2] << "," << N113[2]/N113[2] << endl;
	cout << endl << "N212: " << N212[0]/N212[2] << "," << N212[1]/N212[2] << "," << N212[2]/N212[2] << endl;
	cout << endl << "N223: " << N223[0]/N223[2] << "," << N223[1]/N223[2] << "," << N223[2]/N223[2] << endl;
	cout << endl << "N213: " << N213[0]/N213[2] << "," << N213[1]/N213[2] << "," << N213[2]/N213[2] << endl;
	cout << endl << "N312: " << N312[0]/N312[2] << "," << N312[1]/N312[2] << "," << N312[2]/N312[2] << endl;
	cout << endl << "N323: " << N323[0]/N323[2] << "," << N323[1]/N323[2] << "," << N323[2]/N323[2] << endl;
	cout << endl << "N313: " << N313[0]/N313[2] << "," << N313[1]/N313[2] << "," << N313[2]/N313[2] << endl;

}