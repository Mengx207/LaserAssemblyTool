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

	// laserline::solvePnP_result solvePnP_result_d1,solvePnP_result_d2,solvePnP_result_d3;
	// Mat image_captured_d1, image_captured_d2, image_captured_d3;
	// image_captured_d1 = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
	// image_captured_d2 = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);
	// image_captured_d3 = imread("images/pattern_d3.png", IMREAD_GRAYSCALE);

	// Size patternSize (7,4);
	// double squareSize = 7;
	// solvePnP_result_d2 = laserline::getRvecTvec(image_captured_d2,patternSize,squareSize);
	// solvePnP_result_d3 = laserline::getRvecTvec(image_captured_d3,patternSize,squareSize);
	// solvePnP_result_d1 = laserline::getRvecTvec(image_captured_d1,patternSize,squareSize);

    // Point3d p1 = general::locationCam2Target( imgPoint_d1, solvePnP_result_d1);
	// Point3d p2 = general::locationCam2Target( imgPoint_d2, solvePnP_result_d2);
	// Point3d p3 = general::locationCam2Target( imgPoint_d3, solvePnP_result_d3);
	// cout<<endl<<"3 points in camera frame: " << endl <<p1<<" "<<p2<<" "<<p3<<endl;
	// general::lineEquation(p1,p3,tvec_laser_values);

	/*Laser plane verification----------------------------------------------------------------------------------------------------*/
	cout<<endl<<"plane verification starts"<<endl;
	vector<Point3d> start_vector, end_vector;
	ifstream start_d1("values/laserlinetwopoints/start_l2_d1.txt");
	ifstream start_d2("values/laserlinetwopoints/start_l2_d2.txt");
	ifstream start_d3("values/laserlinetwopoints/start_l2_d3.txt");
	ifstream end_d1("values/laserlinetwopoints/end_l2_d1.txt");
	ifstream end_d2("values/laserlinetwopoints/end_l2_d2.txt");
	ifstream end_d3("values/laserlinetwopoints/end_l2_d3.txt");
	if(argv[1] == string("1"))
	{
		ifstream start_d1("values/laserlinetwopoints/start_l1_d1.txt");
		ifstream start_d2("values/laserlinetwopoints/start_l1_d2.txt");
		ifstream start_d3("values/laserlinetwopoints/start_l1_d3.txt");
		ifstream end_d1("values/laserlinetwopoints/end_l1_d1.txt");
		ifstream end_d2("values/laserlinetwopoints/end_l1_d2.txt");
		ifstream end_d3("values/laserlinetwopoints/end_l1_d3.txt");
	}
	else if(argv[1] == string("2"))
	{
		ifstream start_d1("values/laserlinetwopoints/start_l2_d1.txt");
		ifstream start_d2("values/laserlinetwopoints/start_l2_d2.txt");
		ifstream start_d3("values/laserlinetwopoints/start_l2_d3.txt");
		ifstream end_d1("values/laserlinetwopoints/end_l2_d1.txt");
		ifstream end_d2("values/laserlinetwopoints/end_l2_d2.txt");
		ifstream end_d3("values/laserlinetwopoints/end_l2_d3.txt");
	}
	else if(argv[1] == string("3"))
	{
		ifstream start_d1("values/laserlinetwopoints/start_l3_d1.txt");
		ifstream start_d2("values/laserlinetwopoints/start_l3_d2.txt");
		ifstream start_d3("values/laserlinetwopoints/start_l3_d3.txt");
		ifstream end_d1("values/laserlinetwopoints/end_l3_d1.txt");
		ifstream end_d2("values/laserlinetwopoints/end_l3_d2.txt");
		ifstream end_d3("values/laserlinetwopoints/end_l3_d3.txt");
	}
	else
	{
		ifstream start_d1("values/laserlinetwopoints/start_l4_d1.txt");
		ifstream start_d2("values/laserlinetwopoints/start_l4_d2.txt");
		ifstream start_d3("values/laserlinetwopoints/start_l4_d3.txt");
		ifstream end_d1("values/laserlinetwopoints/end_l4_d1.txt");
		ifstream end_d2("values/laserlinetwopoints/end_l4_d2.txt");
		ifstream end_d3("values/laserlinetwopoints/end_l4_d3.txt");
	}
	double x, y, z;
	char comma;
	// Read Point3d in vector
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
	
	// Vector for vectors in 3D space from start to end points
	vector<vector<double>> vect3D_collection;

	for(int s=0; s<3; s++)
	{
		vector<double> v1;
		for(int e=0; e<3; e++)
		{
			vector<double> v1;
			v1.push_back(end_vector[e].x-start_vector[s].x);
			v1.push_back(end_vector[e].y-start_vector[s].y);
			v1.push_back(end_vector[e].z-start_vector[s].z);
			// cout<<endl<<v1[0]<<" "<<v1[1]<<" "<<v1[2]<<endl;
			vect3D_collection.push_back(v1);
		}
	}
	double min;
	vector<vector<double>> normalVector_collection;
	for(int i=0; i<3; i++)
	{
		// normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[0],vect3D_collection[1]));
		// min = *min_element(normalVector_collection[0].begin(), normalVector_collection[0].end());
		// transform(normalVector_collection[0].begin(), normalVector_collection[0].end(), normalVector_collection[0].begin(), [min](double &c){ return c/min; });

		// normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[2],vect3D_collection[3]));
		// min = *min_element(normalVector_collection[1].begin(), normalVector_collection[1].end());
		// transform(normalVector_collection[1].begin(), normalVector_collection[1].end(), normalVector_collection[1].begin(), [min](double &c){ return c/min; });

		normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+1]));
		min = *min_element(normalVector_collection[3*i].begin(), normalVector_collection[3*i].end());
		transform(normalVector_collection[3*i].begin(), normalVector_collection[3*i].end(), normalVector_collection[3*i].begin(), [min](double &c){ return c/min; });

		normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+2]));
		min = *min_element(normalVector_collection[3*i+1].begin(), normalVector_collection[3*i+1].end());
		transform(normalVector_collection[3*i+1].begin(), normalVector_collection[3*i+1].end(), normalVector_collection[3*i+1].begin(), [min](double &c){ return c/min; });

		normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[3*i+1],vect3D_collection[3*i+2]));
		min = *min_element(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end());
		transform(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end(), normalVector_collection[3*i+2].begin(), [min](double &c){ return c/min; });
	}

	double xSum = 0; double ySum = 0; double zSum = 0;
	Point3d norm_avg;
	for(int i=0; i<9; i++)
	{
		cout << endl << "normal vectors: " << normalVector_collection[i][0] << "," << normalVector_collection[i][1] << "," << normalVector_collection[i][2]<< endl;
		xSum = xSum + normalVector_collection[i][0];
		ySum = ySum + normalVector_collection[i][1];
		zSum = zSum + normalVector_collection[i][2];
	}
	cout<<endl<<xSum<<","<<ySum<<","<<zSum<<endl;
	norm_avg.x = xSum/9;
	norm_avg.y = ySum/9;
	norm_avg.z = zSum/9;
	cout<<endl<<"Normal vector average: "<< norm_avg<<endl;

	laserline::laser_plane laser_plane;
	laser_plane = laserline::laserPlane(rmatrix_laser_values, tvec_laser_values);
	cout<<endl<<"Normal vector of ideal laser plane: "<< laser_plane.normalvector[0]<<","<<laser_plane.normalvector[1]<<","<<laser_plane.normalvector[2]<<endl;

}