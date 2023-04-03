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
void laserlineInfo(RotatedRect rect, Point2d cal_center, int cal_angle, Mat drawing);

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
		// camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration,RegistrationMode_ReplaceAll, Cleanup_Delete);	
		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration1,RegistrationMode_ReplaceAll, Cleanup_Delete);

		//Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0;
		cv::Mat src, cam_frame_temp0;
		camera0.Open();

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
		CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(200.0);

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
		
				cam_frame_temp0 = cv::Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *) pylonImage0.GetBuffer());
				src = cam_frame_temp0.clone();

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
				Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());

				// gain rmatrix and tvec from target board to cam
				string path_rmatrix = "values/rmatrix_laser_1.txt";
				string path_tvec = "values/tvec_laser_1.txt";
				if(argv[1] == string("1")) 
				{
					path_rmatrix = "values/rmatrix_laser_1.txt";
					path_tvec = "values/tvec_laser_1.txt";
				}
				if(argv[1] == string("2")) 
				{
					path_rmatrix = "values/rmatrix_laser_2.txt";
					path_tvec = "values/tvec_laser_2.txt";
				}
				if(argv[1] == string("3")) 
				{
					path_rmatrix = "values/rmatrix_laser_3.txt";
					path_tvec = "values/tvec_laser_3.txt";
				}
				if(argv[1] == string("4"))
				{
					path_rmatrix = "values/rmatrix_laser_4.txt";
					path_tvec = "values/tvec_laser_4.txt";
				}

				// Calculate rotation vector and translation vector by a captured image of a pattern
				pair<Mat,Mat>vec = laserline::getRvecTvec();

				// read laser 1
				ifstream rmatrixL(path_rmatrix);
				vector<double> rmatrix_laser_values;
				while (rmatrixL >> val)
				{
					rmatrix_laser_values.push_back(val);
				}
				ifstream tvecL(path_tvec);
				vector<double> tvec_laser_values;
				while (tvecL >> val)
				{
					tvec_laser_values.push_back(val);
				}
				// find target board plane in cam frame
				pair<vector<double>,vector<double>>target = laserline::targetBoardPlane(vec.first, vec.second);

				laserline::laser_plane laser_1, laser_2, laser_3;
				laser_1 = laserline::laserPlane(rmatrix_laser_values, tvec_laser_values);
				
				// find intersection between laser beam and target board
				Point3f interPoint1;
				interPoint1 = laserline::intersectionPoint(laser_1.P0, laser_1.laserbeam, target.first, target.second);
				vector<Point3d> interPointArray;
				vector<Point2d> projectedInterPoints;
				interPointArray.push_back(interPoint1);
				projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);
				// cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<projectedInterPoints<<endl;
				
				// find intersection line between target board plane and laser plane in cam frame
				std::vector<cv::Point3d> laserline_points_1, laserline_points_2, laserline_points_3;
				laserline::intersection line1, line2, line3;
				line1 = laserline::intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});

				for(int t=-100; t<100;)
				{
					t = t+10;
					Point3d points((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
					// cout<<"point: "<<points<<endl;
					laserline_points_1.push_back(points);
				}
				vector<Point2d> projectedlaserline_1,projectedlaserline_2,projectedlaserline_3;
				projectPoints(laserline_points_1, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_1);

				double delta_y = abs(projectedlaserline_1[19].y - projectedlaserline_1[0].y);
				cout<<endl<<delta_y<<endl;
				double delta_x = abs(projectedlaserline_1[19].x - projectedlaserline_1[0].x);
				cout<<endl<<delta_x<<endl;
				int cal_angle = atan(delta_y/delta_x)*180/CV_PI;
				cout<<endl<<cal_angle<<endl;

			//----------raw image to greyscale, threshold filter
				Mat img_grey;
				cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
				Mat line_img = src.clone();
				laserdot::CalculatedLine( line_img, projectedlaserline_1[0], projectedlaserline_1[19] );

				cv::circle( line_img, projectedInterPoints[0], 5, cv::Scalar(0,0,255), -1, 8, 0 );

				cv::Mat threshold_output1,threshold_output2;
				vector<vector<Point> > contours;
  				vector<Vec4i> hierarchy;
				cv::threshold(img_grey,threshold_output1,100,255,cv::THRESH_OTSU||cv::THRESH_TRIANGLE);	
				cv::threshold(img_grey,threshold_output2,150,255,cv::THRESH_BINARY);
				// imshow("threshold1",threshold_output1);
				// imshow("threshold2",threshold_output2);
				findContours( threshold_output2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

				vector<RotatedRect> minRect;
				RotatedRect rect;

				for( int i = 0; i < contours.size(); i++ )
				{ 
					if(contours[i].size() > 30)
					{
						rect = minAreaRect( Mat(contours[i]) );
						minRect.push_back(rect);
						cout<<"Rectangle center: "<<rect.center<<endl;
						cout<<"Rectangle size "<<rect.size<<endl;
						cout<<"Rectangle angle: "<<rect.angle<<endl;
						//cout<<endl<<"contour #"<<i<<endl<<contours[i]<<endl;
					}
				}

				/// Draw contours + rotated rects
				Mat drawing = Mat::zeros( threshold_output2.size(), CV_8UC3 );

				laserlineInfo(rect, projectedInterPoints[0], cal_angle, line_img);

				for( int i = 0; i< contours.size(); i++ )
					{
					Scalar color = Scalar( 0, 0, 255 );
					// contour
					drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
					}

				for( int i = 0; i < minRect.size(); i++)
				{
					Scalar color = Scalar( 255, 255, 0 );
					// rotated rectangle
					Point2f rect_points[4]; 
					minRect[i].points( rect_points );
					// cout<<"rect_points: "<<endl<<rect_points[0]<<rect_points[1]<<rect_points[2]<<rect_points[3]<<endl;

					for( int j = 0; j < 4; j++ )
					{
						line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
					}
				}

				pair<double,double> hough_avg = laserline::HoughAverage(img_grey);
				// laserline::HoughAvgOnImage(line_img,hough_avg);	

				cv::imshow( "Contour and Area", drawing );
				cv::imshow("Laser Plane Alignment GUI Window", line_img);
				// Show in a window					
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

void laserlineInfo(RotatedRect rect, Point2d cal_center, int cal_angle, Mat drawing)
{
	std::string center_print_x, center_print_y, angle_print, cal_center_print_x, cal_center_print_y, cal_angle_print;
	center_print_x = std::to_string(int(rect.center.x));
	center_print_y = std::to_string(int(rect.center.y));
	angle_print = std::to_string(int(rect.angle));
	cal_center_print_x = std::to_string(int(cal_center.x));
	cal_center_print_y = std::to_string(int(cal_center.y));
	cal_angle_print = std::to_string(cal_angle);

	cv::putText(drawing, "Calculated Center: [" + cal_center_print_x + "," + cal_center_print_y + "]", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Calculated Angle: " + cal_angle_print, cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Actual Center: [" + center_print_x + "," + center_print_y + "]", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Actual Angle: " + angle_print, cv::Point(10,120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
}