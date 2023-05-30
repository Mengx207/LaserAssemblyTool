/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/
#include <iostream>
#include <iomanip>
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
#include <pylon/PylonIncludes.h>
#include "include/ConfigurationEventPrinter.h"
#include "include/utility.h"

using namespace Pylon;
using namespace std;
using namespace GENAPI_NAMESPACE;

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif

void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, laserline::uniformity_data uniformity1, Mat drawing);

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
     int exitCode = 0;
     // Before using any pylon methods, the pylon runtime must be initialized.
     PylonInitialize();
     const char *err;
	try
	{	
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		CInstantCamera camera0( tlFactory.CreateFirstDevice() );
        // Print the camera information.
        cout << "Using device " << camera0.GetDeviceInfo().GetModelName() << endl;
        cout << "SerialNumber : " << camera0.GetDeviceInfo().GetSerialNumber() << endl;
        cout << endl;
		cout << "Program is running, select image window and press 'q' to quit."<<endl;
	
		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration1,RegistrationMode_ReplaceAll, Cleanup_Delete);

		//Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0;
		Mat src, cam_frame_temp0;
		Mat line_img;
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

		if(argv[1] == string("1") || argv[1] == string("3")) 
		{
			CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");			
			CBooleanParameter(nodemap0, "LineInverter").SetValue(false);

			CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
			CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
		}
		if(argv[1] == string("2") || argv[1] == string("4")) 
		{
			CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");			
			CBooleanParameter(nodemap0, "LineInverter").SetValue(true);

			CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
			CBooleanParameter(nodemap0, "LineInverter").SetValue(false);
		}

		while(waitKey(10) != 'q')
		{
			int max_imgs0 = 1;   
			int imgs_taken = 0;

			camera0.StartGrabbing(max_imgs0*1);

			while (imgs_taken < max_imgs0) 
			{	
				CGrabResultPtr ptrGrabResult0;
				CGrabResultPtr ptrGrabResult1;
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

				// Calculate rotation vector and translation vector by a captured image of a pattern
				laserline::solvePnP_result solvePnP_result;
				Mat image_captured = imread("images/pattern.png", IMREAD_GRAYSCALE);
				Size patternSize (5,3);
				double squareSize = 6.75;
				solvePnP_result = laserline::getRvecTvec(image_captured,patternSize,squareSize);

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
					tvec_laser_values.push_back(val*1000);
				}
				// find target board plane in cam frame
				pair<vector<double>,vector<double>>target = laserline::targetBoardPlane(solvePnP_result.rmatrix, solvePnP_result.tvec);

				laserline::laser_plane laser_1, laser_2, laser_3;
				laser_1 = laserline::laserPlane(rmatrix_laser_values, tvec_laser_values);
				
				// find intersection between laser beam and target board
				Point3f interPoint1;
				interPoint1 = laserline::intersectionPoint(laser_1.origin, laser_1.beam_dir, target.first, target.second);
				vector<Point3d> interPointArray;
				vector<Point2d> projectedInterPoints;
				interPointArray.push_back(interPoint1);
				projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);
				// cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<projectedInterPoints<<endl;
				
				// find intersection line between target board plane and laser plane in cam frame
				std::vector<cv::Point3d> laserline_points_1;
				laserline::intersection line1, line2, line3;
				line1 = laserline::intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});

				for(int t=-150; t<150;)
				{
					t = t+10;
					Point3d points((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
					// cout<<"point: "<<points<<endl;
					laserline_points_1.push_back(points);
				}
				vector<Point2d> projectedlaserline_1,projectedlaserline_2,projectedlaserline_3;
				projectPoints(laserline_points_1, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_1);

				for(int i=0; i < projectedlaserline_1.size()-1;)
				{
					if((projectedlaserline_1[i].x > 1440.0) || (projectedlaserline_1[i].y > 1080.0) || (projectedlaserline_1[i].x < 0.0) || (projectedlaserline_1[i].y < 0.0))
					{
						projectedlaserline_1.erase(projectedlaserline_1.begin()+i);
					}
					else
					{
						i++;
					}
				}
				// cout<<"two points on line: "<<projectedlaserline_1[0]<<projectedlaserline_1[projectedlaserline_1.size()-2]<<endl;
				
				// Calculated laser line angle
				double delta_y = (projectedlaserline_1[projectedlaserline_1.size()-2].y - projectedlaserline_1[1].y);
				double delta_x = (projectedlaserline_1[projectedlaserline_1.size()-2].x - projectedlaserline_1[1].x);
				double cal_angle = atan(delta_y/delta_x)*180/CV_PI;	
				if (cal_angle < 0)
				{cal_angle = 90 + cal_angle;}

				Mat img_grey;
				cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
				line_img = src.clone();
				laserdot::CalculatedLine( line_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2] );

				cv::circle( line_img, projectedInterPoints[0], 5, cv::Scalar(0,0,255), -1, 8, 0 );
				// cout<<"one point: "<< projectedInterPoints[0]<<endl;

				cv::Mat threshold_output;
				vector<vector<Point> > contours;
  				vector<Vec4i> hierarchy;
				cv::threshold(img_grey,threshold_output,200,255,cv::THRESH_BINARY);

				findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
				vector<RotatedRect> minRect = laserline::findRectangle(contours,100);
				Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
				Mat rotated_image = threshold_output.clone();

				if (minRect.size() >= 1)
				{
					laserline::drawContourRectangle(drawing, contours, minRect);
					circle(line_img, minRect[0].center, 5, Scalar(0,255,0), -1, 8, 0);
					double angle = minRect[0].angle;
					if (minRect[0].size.width < minRect[0].size.height) 
					{
						angle = 90 + angle;
					}

					Mat rotation_matrix = getRotationMatrix2D(minRect[0].center, angle, 1.0);
					warpAffine(threshold_output, rotated_image, rotation_matrix, threshold_output.size());
					laserline::uniformity_data uniformity1;
					uniformity1 = laserline::cropImage(rotated_image);
					// From pixel number to actual diameter on target board
					uniformity1.width_avg = uniformity1.width_avg* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;
					uniformity1.width_max = uniformity1.width_max* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;
					uniformity1.width_min = uniformity1.width_min* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;

					laserlineGUI(minRect[0], projectedInterPoints[0], cal_angle, uniformity1, line_img);
					cv::imshow( "Rotated and Cropped laser line", uniformity1.image_BGR );
				}
				
				// cv::imshow( "Contour and Area", drawing );
				// cv::imshow("threshold",threshold_output);
				cv::imshow("Laser Plane Alignment GUI Window", line_img);	
				imgs_taken ++;
			}
			camera0.StopGrabbing();	
		}
		std::cout << std::endl << "Saving images" << std::endl;	
		system("cd images && mkdir -p saved_laser_plane");
		if (argc == 3)
		{
			imwrite("images/saved_laser_plane/laser_" + string(argv[1]) + "_" + string(argv[2]) + ".jpg", line_img);
		}
		else {imwrite("images/saved_laser_beam/laser_" + string(argv[1]) + ".jpg", line_img);}
		std::cout << "Finish saving" << std::endl;	

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

void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, laserline::uniformity_data uniformity1, Mat drawing)
{
	std::string center_print_x, center_print_y, angle_print, width_print, cal_center_print_x, cal_center_print_y, cal_angle_print, width_avg_print, width_max_print, width_min_print, width_sd_print;
	center_print_x = std::to_string(int(rect.center.x));
	center_print_y = std::to_string(int(rect.center.y));
	angle_print = std::to_string(int(rect.angle));

	std::ostringstream streamObj;
	streamObj << std::fixed;
	streamObj << std::setprecision(2);

	streamObj << uniformity1.width_max;
	width_max_print = streamObj.str();
	streamObj.str("");
	streamObj << uniformity1.width_min;
	width_min_print = streamObj.str();
	streamObj.str("");
	streamObj << uniformity1.width_sd;
	width_sd_print = streamObj.str();
	streamObj.str("");
	streamObj << uniformity1.width_avg;
	width_avg_print = streamObj.str();
	streamObj.str("");

	if(rect.size.width < rect.size.height)
	{width_print = std::to_string(int(rect.size.width));}
	else
	{width_print = std::to_string(int(rect.size.height));}

	cal_center_print_x = std::to_string(int(cal_center.x));
	cal_center_print_y = std::to_string(int(cal_center.y));
	cal_angle_print = std::to_string(cal_angle);

	cv::putText(drawing, "Angle Designed: " + cal_angle_print, cv::Point(1000,600), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Angle Actual: " + angle_print, cv::Point(1000,630), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Width Average: " + width_avg_print + " mm", cv::Point(1000, 660), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Width Standard Deviation: " + width_sd_print, cv::Point(1000, 690), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Maximum Width: " + width_max_print + " mm", cv::Point(1000, 720), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Minimum Width: " + width_min_print + " mm", cv::Point(1000,740), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
	cv::putText(drawing, "Designed Center: [" + cal_center_print_x + "," + cal_center_print_y + "]", cv::Point(1000, 760), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
	// cv::putText(drawing, "Actual Center: [" + center_print_x + "," + center_print_y + "]", cv::Point(1000, 710), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
}
