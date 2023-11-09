/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/
#include "utility.h"
#include "imgpro.h"
#include "gencal.h"

using namespace Pylon;
using namespace std;
using namespace GENAPI_NAMESPACE;

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif


int main(int argc, char* argv[])
{
	string path_rmatrix = "values/laser2cam_transformatrix/rmatrix_L1.txt";
	string path_tvec = "values/laser2cam_transformatrix/tvec_L1.txt";
	if(argv[1] == string("1")) 
	{
		path_rmatrix = "values/laser2cam_transformatrix/rmatrix_L1.txt";
		path_tvec = "values/laser2cam_transformatrix/tvec_L1.txt";
	}
	if(argv[1] == string("2")) 
	{
		path_rmatrix = "values/laser2cam_transformatrix/rmatrix_L2.txt";
		path_tvec = "values/laser2cam_transformatrix/tvec_L2.txt";
	}
	if(argv[1] == string("3")) 
	{
		path_rmatrix = "values/laser2cam_transformatrix/rmatrix_L3.txt";
		path_tvec = "values/laser2cam_transformatrix/tvec_L3.txt";
	}
	if(argv[1] == string("4"))
	{
		path_rmatrix = "values/laser2cam_transformatrix/rmatrix_L4.txt";
		path_tvec = "values/laser2cam_transformatrix/tvec_L4.txt";
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
	cout<<endl<<"beam verification-------------------------------------------"<<endl;
	vector<double> imgPoint_d1_vector, imgPoint_d2_vector, imgPoint_d3_vector;

	ifstream readPointd1("values/beam_intersections/beam_intersections_l"+string(argv[1])+"_d1.txt");
	while (readPointd1 >> val)
	{
		imgPoint_d1_vector.push_back(val);
	}
	Point2d imgPoint_d1;
	imgPoint_d1.x = imgPoint_d1_vector[0];
	imgPoint_d1.y = imgPoint_d1_vector[1];

	ifstream readPointd2("values/beam_intersections/beam_intersections_l"+string(argv[1])+"_d2.txt");
	while (readPointd2 >> val)
	{
		imgPoint_d2_vector.push_back(val);
	}
	Point2d imgPoint_d2;
	imgPoint_d2.x = imgPoint_d2_vector[0];
	imgPoint_d2.y = imgPoint_d2_vector[1];

	// vector<double> rvec_target2cam, tvec_target2cam;
	// ifstream rvec_s, tvec_s;
	// rvec_s.open("values/aruco_result/rvec_target2cam.txt"); 
	// while (rvec_s >> val)
	// {
	// 	rvec_target2cam.push_back(val);
	// }

	// tvec_s.open("values/aruco_result/tvec_target2cam.txt"); 
	// while (tvec_s >> val)
	// {
	// 	tvec_target2cam.push_back(val*1000);
	// }

	// Mat rvec = Mat(3, 1, CV_64FC1, rvec_target2cam.data());
	// Mat tvec = Mat(3, 1, CV_64FC1, tvec_target2cam.data());
	// Mat rmatrix;
	// Rodrigues(rvec, rmatrix);
	// vector<Point3d> obj_corners;
	// vector<Point2d> found_corners;
	// ifstream obj ("values/aruco_result/corners_obj.txt");
	// vector<double> reg1;
	// while (obj >> val)
	// {
	// 	reg1.push_back(val);
	// }
	// for(int i=0; i<reg1.size(); i=i+3)
	// {
	// 	Point3d pt;
	// 	pt.x = reg1[i];
	// 	pt.y = reg1[i+1];
	// 	pt.z = reg1[i+2];
	// 	obj_corners.push_back(pt);
	// }

	// ifstream found ("values/aruco_result/corners_img.txt");
	// vector<double> reg2;
	// while (found >> val)
	// {
	// 	reg2.push_back(val);
	// }			
	// for(int i=0; i<reg2.size(); i=i+2)
	// {
	// 	Point2d pt;
	// 	pt.x = reg2[i];
	// 	pt.y = reg2[i+1];
	// 	found_corners.push_back(pt);
	// }

	Mat image_captured = imread("images/pattern.png", IMREAD_GRAYSCALE);
	Size patternSize(7, 4);
	double squareSize;
	solvePnP_result solvePnP_result_d1, solvePnP_result_d2, solvePnP_result_d3;
	squareSize = 7; // ~390mm
	solvePnP_result_d1 = getRvecTvec(image_captured, patternSize, squareSize);
	squareSize = 6.1; // ~340mm
	// squareSize = 5.65; //~315mm
	// squareSize = 5.2; // ~290mm
	solvePnP_result_d2 = getRvecTvec(image_captured, patternSize, squareSize);

    Point3d p1 = locationCam2Target( imgPoint_d1, solvePnP_result_d1);
	Point3d p2 = locationCam2Target( imgPoint_d2, solvePnP_result_d2);
	// Point3d p1 = locationCam2Target( imgPoint_d1, rmatrix, tvec, obj_corners, found_corners);
	// Point3d p2 = locationCam2Target( imgPoint_d2, rmatrix, tvec, obj_corners, found_corners);
	// Point3d p3 = locationCam2Target( imgPoint_d3, rmatrix, tvec, obj_corners, found_corners);

	// cout<<endl<<"3 points in camera frame:   " <<p1<<" "<<p2<<" "<<endl;
	lineEquation(p1,p2,tvec_laser_values);
	cout<<endl<<"designed laser origin: "<<"("<<tvec_laser_values[0]<<", "<<tvec_laser_values[1]<<", "<<tvec_laser_values[2]<<")"<<endl;

	/*Laser plane verification----------------------------------------------------------------------------------------------------*/
	cout<<endl<<endl<<"plane verification-------------------------------------------"<<endl;
	vector<Point3d> start_vector, end_vector;
	ifstream start_d1, start_d2, start_d3, end_d1, end_d2, end_d3;

	std::ostringstream oss;
	std::string path_start, path_end;

	for(int i = 1; i<=2; i++)
	{
		path_start ="values/laserline_3Dpoints/start_l" + string(argv[1]) + "_d" + to_string(i) + ".txt";
		path_end = "values/laserline_3Dpoints/end_l" + string(argv[1]) + "_d" + to_string(i) + ".txt";
		if(i==1){start_d1.open(path_start); end_d1.open(path_end);}
		else if(i==2){start_d2.open(path_start); end_d2.open(path_end);}
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
	while (end_d1 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}
	while (end_d2 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}

	// cout<<endl<< "start_d1: "<< start_vector[0]<<endl;
	// cout<<endl<< "start_d2: "<< start_vector[1]<<endl;
	// cout<<endl<< "end_d1: "<< end_vector[0]<<endl;
	// cout<<endl<< "end_d2: "<< end_vector[1]<<endl;
	
	// Vector for vectors in 3D space from start to end points
	vector<vector<double>> vect3D_collection;

	for(int s=0; s<2; s++)
	{
		vector<double> v1;
		for(int e=0; e<2; e++)
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
		vector<double> NV = crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+1]);
		double d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
		NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
		normalVector_collection.push_back(NV);

		NV = crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+2]);
		d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
		NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
		normalVector_collection.push_back(NV);

		NV = crossProduct(vect3D_collection[3*i+1],vect3D_collection[3*i+2]);
		d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
		NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
		normalVector_collection.push_back(NV);

		// normalVector_collection.push_back(crossProduct(vect3D_collection[3*i+1],vect3D_collection[3*i+2]));
		// min = *min_element(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end());
		// transform(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end(), normalVector_collection[3*i+2].begin(), [min](double &c){ return c/min; });
	}

	double xSum = 0; double ySum = 0; double zSum = 0;
	Point3d norm_avg;
	for(int i=0; i<9; i++)
	{
		// cout << endl << "The actual normal vectors: " << normalVector_collection[i][0] << "," << normalVector_collection[i][1] << "," << normalVector_collection[i][2]<< endl;
		xSum = xSum + normalVector_collection[i][0];
		ySum = ySum + normalVector_collection[i][1];
		zSum = zSum + normalVector_collection[i][2];
	}
	norm_avg.x = xSum/9;
	norm_avg.y = ySum/9;
	norm_avg.z = zSum/9;
	cout<<endl<<"The actual normal vector average: "<< norm_avg<<endl;

	laser_plane laser_plane;
	laser_plane = laserPlane(rmatrix_laser_values, tvec_laser_values);
	cout<<endl<<"The ideal normal vector of ideal laser plane: "<<"["<< laser_plane.normalvector[0]<<","<<laser_plane.normalvector[1]<<","<<laser_plane.normalvector[2]<<"]"<<endl;

}