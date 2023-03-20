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
    // The exit code of the sample application.
     int exitCode = 0;
     // Before using any pylon methods, the pylon runtime must be initialized.
     PylonInitialize();
     const char *err;
   
	try
	{		
		CDeviceInfo info0, info1;
		info0.SetSerialNumber("40113772"); //cam1, right
		//info1.SetSerialNumber("40172226"); //cam2, left
		

	    CInstantCamera camera0( CTlFactory::GetInstance().CreateDevice(info0));	

			
		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration,RegistrationMode_ReplaceAll, Cleanup_Delete);

		//Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0;
		cv::Mat src, cam_frame_temp0;
		camera0.Open();
		
		cv::Point max_point;
		cv::Mat canny_edge, canny_edge_blur, img_grey;

		// These allow us to convert from GrabResultPtr_t to cv::Mat
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;

		int size_avg ;
        int min_size = 1000;
		int size_array[10] = {0};
		int last_size_avg;
		int last_min_size=0;
		cv::Point center_avg;

		vector<cv::Point> center_list;
		double center_total = 0;
			
		INodeMap& nodemap0 = camera0.GetNodeMap();

		CEnumerationPtr(nodemap0.GetNode("ExposureMode"))->FromString("Timed"); 
		CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(50.0);

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
		CEnumParameter(nodemap0, "LineMode").SetValue("Output");
		CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput2");

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
		CEnumParameter(nodemap0, "LineMode").SetValue("Output");
		CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput3");

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
		CBooleanParameter(nodemap0, "LineInverter").SetValue(false);

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
		CBooleanParameter(nodemap0, "LineInverter").SetValue(false);
		CEnumParameter(nodemap0, "LineSelector").SetValue("Line2");
		CEnumParameter(nodemap0, "LineSource").SetValue("ExposureActive");

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");			
		CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
		CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
		CBooleanParameter(nodemap0, "LineInverter").SetValue(true);

		while(1)
		{
			int max_imgs0 = 50;   
			int imgs_taken0 =0;

			camera0.StartGrabbing(max_imgs0*1);

			while (imgs_taken0 < max_imgs0) 
			{	

				CGrabResultPtr ptrGrabResult0;
				CGrabResultPtr ptrGrabResult1;

				sleep(0.1);

				while(camera0.WaitForFrameTriggerReady(1000,TimeoutHandling_ThrowException)==0);			
				CCommandParameter(nodemap0, "TriggerSoftware").Execute();
				
				bool test0 = camera0.RetrieveResult(1000, ptrGrabResult0, TimeoutHandling_ThrowException);

				formatConverter.Convert(pylonImage0, ptrGrabResult0);
//-----------------------Main------Functionalities------Start------From------Here-----------------------------------------------------------------------------				
				cam_frame_temp0 = cv::Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *) pylonImage0.GetBuffer());

				src = cam_frame_temp0.clone();

//=========================================================================================================================================================================
	// gain rmatrix and tvec from target board to cam
    pair<Mat,Mat>vec = laserline::getRvecTvec();
    string path_rmatrix = "values/rmatrix_laser_1.txt";
    string path_tvec = "values/tvec_laser_1.txt";

    // find laser plane in cam frame
    double val;
    // read laser 1
    ifstream rmatrix_1(path_rmatrix);
    vector<double> rmatrix_laser_1_values;
    while (rmatrix_1 >> val)
    {
        rmatrix_laser_1_values.push_back(val);
    }
    ifstream tvecL_1(path_tvec);
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
    std::vector<cv::Point2d> laserPoints_1;
	laserline::intersection line1;
    line1 = laserline::intersectionLine(target.first, laser_1.first, target.second, laser_1.second);
  	for(int t=-2000; t<2000;)
    {
        t = t+40;
        // Point2d points((69.2212+(-0.499939)*t)+500, -(-121.744+0.850974*t)+500);
        Point2d points((line1.x+line1.a*t)+720, -(line1.y+line1.b*t)+540);
        // cout<<"point: "<<points<<endl;
        laserPoints_1.push_back(points);
    }
	cv::Point lineStart = laserPoints_1[0];
	cv::Point lineEnd = laserPoints_1[90];
	// laserdot::CalculatedLine( src, lineStart, lineEnd );

//=========================================================================================================================================================================

			 //----------raw image to greyscale, threshold filter
				cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
				cv::Mat img_grey_filtered;
				cv::threshold(img_grey,img_grey_filtered,250,255,cv::THRESH_OTSU||cv::THRESH_TRIANGLE);	

				laserdot::CalculatedLine( src, lineStart, lineEnd );
				// Number of non_zero pixel
				int non_zero = laserdot::NonZero(img_grey_filtered);

				// Number of bright pixel
				int count = laserdot::PixelCounter(img_grey_filtered);
				// average size
				size_avg = laserdot::SizeAverage(count,0,size_array, center_total);
				//-----------save min_size only when the value in size_array is stable and close to each other
				if(size_avg < min_size && center_total > 20)
				{
					if(abs(size_array[0]-size_array[9]) <=3)
					{
						min_size = size_avg;
					}
				}

			 //---------Draw circles based on collected points	
			 //---------Process when captured images are not empty
			 //---------Clear center list and size array is capture an empty image
			 	double nom_distance,center_distance;
				vector<cv::Vec3f> circles;
				if(non_zero > 50)
				{	
					cv::Point center;
					HoughCircles(img_grey_filtered, circles, cv::HOUGH_GRADIENT,2, 2000,500,10,0,100);
					if(!circles.empty())
					{
						center.x = cvRound(circles[0][0]);
						center.y = cvRound(circles[0][1]);
						center_list.push_back(center);
						center_total ++;
					}
					if (center_total > 30)
					{
						center_list.erase(center_list.begin());
						center_total --;
					}
					
					if (center_total >= 10)
					{
						cv::Point sum  = std::accumulate(center_list.begin(), center_list.end(), cv::Point(0,0));
						center_avg = sum*(1.0/center_total);
						//center_avg = sum*(1.0/200);
						cout<<"Average center: "<< center_avg<<endl;
						sleep(0.1);
						cv::circle( img_grey_filtered, center_avg, 3, cv::Scalar(255,0,0), -1, 8, 0 );
						cv::circle( src, center_avg, 3, cv::Scalar(255,100,0), -1, 8, 0 );
						std::pair<double,double>dist = laserdot::DotToLine(src, lineStart, lineEnd, center_avg);
						nom_distance = dist.first;
						center_distance = dist.second;
					}
					
					// std::pair<double,double>dist = laserdot::DotToLine(src, lineStart, lineEnd, center_avg);
					// nom_distance,center_distance = dist.first,dist.second;
				}
				else
				{
					last_min_size = min_size;
					cout<<"last min size: "<<last_min_size<<endl;
					fill_n(size_array,10,0);
					center_total = 0;
					center_list.clear();
					//fill(center_list.begin(), center_list.end(), cv::Point(0,0));
				}
				laserdot::HMI(src, size_avg, min_size, non_zero, nom_distance, center_distance);
				laserdot::GreenLight(src, last_min_size, size_avg, nom_distance, center_distance);

				cv::imshow("img_grey_filtered", img_grey_filtered);	
				cv::imshow("source window", src);							
				cv::waitKey( 10 );		
				sleep(0.1);
				imgs_taken0++;
			}
			camera0.StopGrabbing();
		}

	}

	catch (const GenericException &e)
	{
		// Error handling
		cerr << "An exception occurred." << endl
		<< e.GetDescription() << endl;
		exitCode = 1;
		
	}
	
	PylonTerminate();
   	return exitCode;
   
}