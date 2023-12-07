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
	string path_rmatrix;
	string path_tvec;
	bool version_V4 = false;

	if(argv[1] == string("V3"))
	{
		version_V4 = false;
		if(argv[2] == string("1")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L1_V3.txt";
			path_tvec = "values/laser_transform/tvec_L1_V3.txt";
		}
		if(argv[2] == string("2")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L2_V3.txt";
			path_tvec = "values/laser_transform/tvec_L2_V3.txt";
		}
		if(argv[2] == string("3")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L3_V3.txt";
			path_tvec = "values/laser_transform/tvec_L3_V3.txt";
		}
		if(argv[2] == string("4"))
		{
			path_rmatrix = "values/laser_transform/rmatrix_L4_V3.txt";
			path_tvec = "values/laser_transform/tvec_L4_V3.txt";
		}
	}
	if(argv[1] == string("V4"))
	{
		version_V4 = true;
		if(argv[2] == string("1")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L1_V4.txt";
			path_tvec = "values/laser_transform/tvec_L1_V4.txt";
		}
		if(argv[2] == string("2")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L2_V4.txt";
			path_tvec = "values/laser_transform/tvec_L2_V4.txt";
		}
		if(argv[2] == string("3")) 
		{
			path_rmatrix = "values/laser_transform/rmatrix_L3_V4.txt";
			path_tvec = "values/laser_transform/tvec_L3_V4.txt";
		}
		if(argv[2] == string("4"))
		{
			path_rmatrix = "values/laser_transform/rmatrix_L4_V4.txt";
			path_tvec = "values/laser_transform/tvec_L4_V4.txt";
		}
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

	ifstream readPointd1("values/real_laserdot/real_laserdot_l"+string(argv[2])+"_d1.txt");
	while (readPointd1 >> val)
	{
		imgPoint_d1_vector.push_back(val);
	}
	Point2d imgPoint_d1;
	imgPoint_d1.x = imgPoint_d1_vector[0];
	imgPoint_d1.y = imgPoint_d1_vector[1];

	ifstream readPointd2("values/real_laserdot/real_laserdot_l"+string(argv[2])+"_d2.txt");
	while (readPointd2 >> val)
	{
		imgPoint_d2_vector.push_back(val);
	}
	Point2d imgPoint_d2;
	imgPoint_d2.x = imgPoint_d2_vector[0];
	imgPoint_d2.y = imgPoint_d2_vector[1];

	Mat image_captured;
	Size patternSize(7, 4);
	double squareSize = 7;
	solvePnP_result solvePnP_result_d1, solvePnP_result_d2;
	image_captured = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
	cv::threshold(image_captured, image_captured, 120, 255, cv::THRESH_BINARY);
	solvePnP_result_d1 = getRvecTvec(image_captured, patternSize, squareSize);
	image_captured = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);
	cv::threshold(image_captured, image_captured, 120, 255, cv::THRESH_BINARY);
	solvePnP_result_d2 = getRvecTvec(image_captured, patternSize, squareSize);
	// Intersection point between laser beam and target borad in camera frame
    Point3d point_d1 = locationCam2Target( imgPoint_d1, solvePnP_result_d1, version_V4 ); 
	Point3d point_d2 = locationCam2Target( imgPoint_d2, solvePnP_result_d2, version_V4 );
	Point3d real_origin = lineEquation(point_d1,point_d2,tvec_laser_values);

	cout<<endl<<"real laser origin: " << real_origin <<endl;
	cout<<endl<<"designed laser origin: "<<"("<<tvec_laser_values[0]<<", "<<tvec_laser_values[1]<<", "<<tvec_laser_values[2]<<")"<<endl;

	/*Laser plane verification----------------------------------------------------------------------------------------------------*/
	cout<<endl<<endl<<"plane verification-------------------------------------------"<<endl;
	vector<Point3d> start_vector, end_vector;
	ifstream start_d1, start_d2, start_d3, end_d1, end_d2, end_d3;

	std::ostringstream oss;
	std::string path_start, path_end;

	for(int i = 1; i<=2; i++)
	{
		path_start ="values/real_laserline_ends/start_l" + string(argv[2]) + "_d" + to_string(i) + ".txt";
		path_end = "values/real_laserline_ends/end_l" + string(argv[2]) + "_d" + to_string(i) + ".txt";
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
		while (end_d1 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}
	while (start_d2 >> x >> comma >> y >> comma >> z)
	{
		start_vector.push_back(Point3d(x,y,z));
	}
	while (end_d2 >> x >> comma >> y >> comma >> z)
	{
		end_vector.push_back(Point3d(x,y,z));
	}

	// Vector for vectors in 3D space from start to end points
	vector<vector<double>> vect3D_collection;
	vector<double> v1,v2,v3;
	//vector 1
	v1.push_back(end_vector[0].x-real_origin.x);
	v1.push_back(end_vector[0].y-real_origin.y);
	v1.push_back(end_vector[0].z-real_origin.z);
	//vector 2
	v2.push_back(start_vector[0].x-real_origin.x);
	v2.push_back(start_vector[0].y-real_origin.y);
	v2.push_back(start_vector[0].z-real_origin.z);
	//vector 3
	v3.push_back(point_d1.x-real_origin.x);
	v3.push_back(point_d1.y-real_origin.y);
	v3.push_back(point_d1.z-real_origin.z);
	vect3D_collection.push_back(v1);
	vect3D_collection.push_back(v2);
	vect3D_collection.push_back(v3);


	vector<double> NV;
	vector<vector<double>> normalVector_collection;
	// collect unit normal vectors between different vectors on the actual laser plane
	for(int i=1; i<=2; i++)
	{
		NV = crossProduct(vect3D_collection[0],vect3D_collection[i]);
		double d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
		NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
		normalVector_collection.push_back(NV);
		// cout<<endl<<"("<<NV[0]<<","<<NV[1]<<","<<NV[2]<<")"<<endl;
	}

	NV = crossProduct(vect3D_collection[1],vect3D_collection[2]);
	double d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
	NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
	normalVector_collection.push_back(NV);
	// cout<<endl<<"("<<NV[0]<<","<<NV[1]<<","<<NV[2]<<")"<<endl;

	double xSum = 0; double ySum = 0; double zSum = 0;
	Point3d norm_avg;
	for(int i=0; i<3; i++)
	{
		// cout << endl << "The actual normal vectors: " << normalVector_collection[i][0] << "," << normalVector_collection[i][1] << "," << normalVector_collection[i][2]<< endl;
		xSum = xSum + normalVector_collection[i][0];
		ySum = ySum + normalVector_collection[i][1];
		zSum = zSum + normalVector_collection[i][2];
	}
	norm_avg.x = xSum/3;
	norm_avg.y = ySum/3;
	norm_avg.z = zSum/3;
	cout<<endl<<"The real plane normal vector: "<< norm_avg<<endl;

	laser_plane laser_plane;
	laser_plane = laserPlane(rmatrix_laser_values, tvec_laser_values);
	cout<<endl<<"The designed plane normal vector: "<<"["<< laser_plane.normalvector[0]<<","<<laser_plane.normalvector[1]<<","<<laser_plane.normalvector[2]<<"]"<<endl;

	system("cd values && mkdir -p verification");
	if(argc == 4)
	{
			ofstream verification ("values/verification/L" + string(argv[2]) + "_" + string(argv[3]) + ".txt");
			verification <<"The design origin: "<<"["<<tvec_laser_values[0]<<", "<<tvec_laser_values[1]<<", "<<tvec_laser_values[2]<<"]"<<endl
			<<"The real origin: "<<real_origin<<endl
			<<"The design NV of laser plane: "<<"["<< laser_plane.normalvector[0]<<","<<laser_plane.normalvector[1]<<","<<laser_plane.normalvector[2]<<"]"<<endl
			<<"The real NV of laser plane: "<<norm_avg<<endl;
	}

}