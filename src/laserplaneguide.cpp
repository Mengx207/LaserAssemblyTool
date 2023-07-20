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

// void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, uniformity_data uniformity1, Mat drawing);

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
     int exitCode = 0;
     // Before using any pylon methods, the pylon runtime must be initialized.
     PylonInitialize();
     const char *err;
	 Mat src, line_img, threshold_output;
	try
	{	
		// CTlFactory& tlFactory = CTlFactory::GetInstance();
		// CInstantCamera camera0( tlFactory.CreateFirstDevice() );
		CDeviceInfo info0, info1;
		info0.SetSerialNumber("40172396");
		info1.SetSerialNumber("40026626");
		CInstantCamera camera0(CTlFactory::GetInstance().CreateDevice(info0));
		CInstantCamera camera1(CTlFactory::GetInstance().CreateDevice(info1));

		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

		camera1.RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		// CTlFactory& tlFactory = CTlFactory::GetInstance();
		// CInstantCamera camera0( tlFactory.CreateFirstDevice() );
		// // Print the camera information.
		cout << "Using device0 " << camera0.GetDeviceInfo().GetModelName() << endl;
		cout << "SerialNumber : " << camera0.GetDeviceInfo().GetSerialNumber() << endl;
		cout << "Using device1 " << camera1.GetDeviceInfo().GetModelName() << endl;
		cout << "SerialNumber : " << camera1.GetDeviceInfo().GetSerialNumber() << endl;

		cout << endl
			 << "Program is running, select image window and press 'q' to quit." << endl;

		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration1, RegistrationMode_ReplaceAll, Cleanup_Delete);

		// Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0, pylonImage1;
		cv::Mat cam_frame_temp0, cam_frame_temp1;
		camera0.Open();
		camera1.Open();

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
			
		INodeMap &nodemap0 = camera0.GetNodeMap();
		INodeMap &nodemap1 = camera1.GetNodeMap();

		CEnumerationPtr(nodemap0.GetNode("ExposureMode"))->FromString("Timed");
		CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(200.0);
		CEnumerationPtr(nodemap1.GetNode("ExposureMode"))->FromString("Timed");
		CFloatPtr(nodemap1.GetNode("ExposureTime"))->SetValue(200.0);

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
		CEnumParameter(nodemap0, "LineMode").SetValue("Output");
		CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput2");
		CEnumParameter(nodemap1, "LineSelector").SetValue("Line3");
		CEnumParameter(nodemap1, "LineMode").SetValue("Output");
		CEnumParameter(nodemap1, "LineSource").SetValue("UserOutput2");

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
		CEnumParameter(nodemap0, "LineMode").SetValue("Output");
		CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput3");
		CEnumParameter(nodemap1, "LineSelector").SetValue("Line4");
		CEnumParameter(nodemap1, "LineMode").SetValue("Output");
		CEnumParameter(nodemap1, "LineSource").SetValue("UserOutput3");

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
		CBooleanParameter(nodemap0, "LineInverter").SetValue(false);
		CEnumParameter(nodemap1, "LineSelector").SetValue("Line3");
		CBooleanParameter(nodemap1, "LineInverter").SetValue(false);

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
		CBooleanParameter(nodemap0, "LineInverter").SetValue(false);
		CEnumParameter(nodemap1, "LineSelector").SetValue("Line4");
		CBooleanParameter(nodemap1, "LineInverter").SetValue(false);

		CEnumParameter(nodemap0, "LineSelector").SetValue("Line2");
		CEnumParameter(nodemap0, "LineSource").SetValue("ExposureActive");
		CEnumParameter(nodemap1, "LineSelector").SetValue("Line2");
		CEnumParameter(nodemap1, "LineSource").SetValue("ExposureActive");

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


		std::vector<double> rvec_target2cam, tvec_target2cam;
		// Mat rvec, rmatrix, tvec;
		vector<Point3d> obj_corners;
		vector<Point2d> found_corners;
		arucoResult aruco_result;

		while(waitKey(10) != 'q')
		{
			int max_imgs0 = 1;   
			int imgs_taken = 0;

			camera0.StartGrabbing(max_imgs0*1);
			camera1.StartGrabbing(max_imgs0*1);

			while (imgs_taken < max_imgs0) 
			{	
				CGrabResultPtr ptrGrabResult0;
				CGrabResultPtr ptrGrabResult1;
				while (camera0.WaitForFrameTriggerReady(10000, TimeoutHandling_ThrowException) == 0);
				while (camera1.WaitForFrameTriggerReady(10000, TimeoutHandling_ThrowException) == 0);
				CCommandParameter(nodemap0, "TriggerSoftware").Execute();
				CCommandParameter(nodemap1, "TriggerSoftware").Execute();
				bool test0 = camera0.RetrieveResult(1000, ptrGrabResult0, TimeoutHandling_ThrowException);
				bool test1 = camera1.RetrieveResult(1000, ptrGrabResult1, TimeoutHandling_ThrowException);
				formatConverter.Convert(pylonImage1, ptrGrabResult1);
				formatConverter.Convert(pylonImage0, ptrGrabResult0);
				cam_frame_temp0 = cv::Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *)pylonImage0.GetBuffer());
				src = cam_frame_temp0.clone();
				cam_frame_temp1 = cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t *) pylonImage1.GetBuffer());
				Mat src1 = cam_frame_temp1.clone();
				imshow ("cam2",src1);
			    // gain rmatrix and tvec from target board to cam
				ifstream intrin("values/camera_matrix/intrinsic.txt");
				vector<double> cameraMatrix_values;
				double val;
				while (intrin >> val)
				{
					cameraMatrix_values.push_back(val);
				}
				ifstream dist("values/camera_matrix/distortion.txt");
				vector<double> distCoeffs_values;
				while (dist >> val)
				{
					distCoeffs_values.push_back(val);
				}
				Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
				Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());
				// gain rmatrix and tvec from target board to cam
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
				aruco_result = readArucoResult();
				pair<vector<double>,vector<double>>target = targetBoardPlane(aruco_result.rmatrix, aruco_result.tvec);

				laser_plane laser_1;
				laser_1 = laserPlane(rmatrix_laser_values, tvec_laser_values);
				
				// find intersection between laser beam and target board
				Point3f interPoint1;
				interPoint1 = intersectionPoint(laser_1.origin, laser_1.beam_dir, target.first, target.second);
				vector<Point3d> interPointArray;
				vector<Point2d> projectedInterPoints;
				interPointArray.push_back(interPoint1);
				projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);
				// cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<projectedInterPoints<<endl;
				
				// find intersection line between target board plane and laser plane in cam frame
				std::vector<cv::Point3d> laserline_points_1;
				intersection line1, line2, line3;
				line1 = intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});

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
				// CalculatedLine( line_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2] );
				line( line_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2],Scalar(200,200,0), 5, LINE_AA );


				cv::circle( line_img, projectedInterPoints[0], 5, cv::Scalar(200,200,0), -1, 8, 0 );
				// cout<<"one point: "<< projectedInterPoints[0]<<endl;

				vector<vector<Point> > contours;
  				vector<Vec4i> hierarchy;
				cv::threshold(img_grey,threshold_output,200,255,cv::THRESH_BINARY);
				// Number of non_zero pixel
				int non_zero = NonZero(threshold_output);

				if (non_zero > 1000)
				{
					findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
					vector<RotatedRect> minRect = findRectangle(contours,100);
					Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
					Mat rotated_image = threshold_output.clone();

					if (minRect.size() >= 1)
					{
						drawContourRectangle(drawing, contours, minRect);
						circle(line_img, minRect[0].center, 5, Scalar(0,255,0), -1, 8, 0);
						double angle = minRect[0].angle;
						if (minRect[0].size.width < minRect[0].size.height) 
						{
							angle = 90 + angle;
						}
						Mat rotation_matrix = getRotationMatrix2D(minRect[0].center, angle, 1.0);
						warpAffine(threshold_output, rotated_image, rotation_matrix, threshold_output.size());;
						uniformity_data uniformity1;
						uniformity1 = cropImage(rotated_image);
						// From pixel number to actual diameter on target board
						uniformity1.width_avg = uniformity1.width_avg* 3.45 * (aruco_result.tvec.at<double>(0,2)/12)/1000;
						uniformity1.width_max = uniformity1.width_max* 3.45 * (aruco_result.tvec.at<double>(0,2)/12)/1000;
						uniformity1.width_min = uniformity1.width_min* 3.45 * (aruco_result.tvec.at<double>(0,2)/12)/1000;
						laserlineGUI(minRect[0], projectedInterPoints[0], cal_angle, uniformity1, line_img);
						// cv::imshow( "Rotated and Cropped laser line", uniformity1.image_BGR );
					}
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
		else if (argc == 4)
		{
			imwrite("images/saved_laser_plane/laser_" + string(argv[1]) + "_" + string(argv[2]) + "_" + string(argv[3]) + "_MD.jpg", line_img);
			imwrite("images/saved_laser_plane/laser_plane_" + string(argv[1]) + "_" + string(argv[2]) + "_" + string(argv[3]) + "_threshold.jpg", threshold_output);

			system("cd values && mkdir -p laserline_3Dpoints && cd laserline_3Dpoints");
			
			pair<Point2f,Point2f> laserline2Points = extractLaserline2Points(threshold_output);
			cout<<"Pair of end points of actual laser line on image plane: "<< laserline2Points.first << ", " << laserline2Points.second<<endl;

			laserline2Points.first = Point2d(0,0);
			laserline2Points.second = Point2d(1440,1080);
			Point3d startCam = locationCam2Target( laserline2Points.first, aruco_result.rmatrix, aruco_result.tvec, aruco_result.obj_corners, aruco_result.found_corners);
			Point3d endCam= locationCam2Target( laserline2Points.second, aruco_result.rmatrix, aruco_result.tvec, aruco_result.obj_corners, aruco_result.found_corners);
		
			if(argv[1] == string("1"))
			{
				if(argv[3] == string("d1"))
				{
					ofstream start_d1("values/laserline_3Dpoints/start_l1_d1.txt");
					start_d1 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d1("values/laserline_3Dpoints/end_l1_d1.txt");
					end_d1 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d2"))
				{
					ofstream start_d2("values/laserline_3Dpoints/start_l1_d2.txt");
					start_d2  << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d2("values/laserline_3Dpoints/end_l1_d2.txt");
					end_d2 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d3"))
				{
					ofstream start_d3("values/laserline_3Dpoints/start_l1_d3.txt");
					start_d3 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d3("values/laserline_3Dpoints/end_l1_d3.txt");
					end_d3 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
			}
			if(argv[1] == string("2"))
			{
				if(argv[3] == string("d1"))
				{
					ofstream start_d1("values/laserline_3Dpoints/start_l2_d1.txt");
					start_d1 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d1("values/laserline_3Dpoints/end_l2_d1.txt");
					end_d1 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d2"))
				{
					ofstream start_d2("values/laserline_3Dpoints/start_l2_d2.txt");
					start_d2  << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d2("values/laserline_3Dpoints/end_l2_d2.txt");
					end_d2 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d3"))
				{
					ofstream start_d3("values/laserline_3Dpoints/start_l2_d3.txt");
					start_d3 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d3("values/laserline_3Dpoints/end_l2_d3.txt");
					end_d3 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
			}
			if(argv[1] == string("3"))
			{
				if(argv[3] == string("d1"))
				{
					ofstream start_d1("values/laserline_3Dpoints/start_l3_d1.txt");
					start_d1 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d1("values/laserline_3Dpoints/end_l3_d1.txt");
					end_d1 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d2"))
				{
					ofstream start_d2("values/laserline_3Dpoints/start_l3_d2.txt");
					start_d2  << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d2("values/laserline_3Dpoints/end_l3_d2.txt");
					end_d2 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d3"))
				{
					ofstream start_d3("values/laserline_3Dpoints/start_l3_d3.txt");
					start_d3 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d3("values/laserline_3Dpoints/end_l3_d3.txt");
					end_d3 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
			}
			if(argv[1] == string("4"))
			{
				if(argv[3] == string("d1"))
				{
					ofstream start_d1("values/laserline_3Dpoints/start_l4_d1.txt");
					start_d1 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d1("values/laserline_3Dpoints/end_l4_d1.txt");
					end_d1 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d2"))
				{
					ofstream start_d2("values/laserline_3Dpoints/start_l4_d2.txt");
					start_d2  << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d2("values/laserline_3Dpoints/end_l4_d2.txt");
					end_d2 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
				else if(argv[3] == string("d3"))
				{
					ofstream start_d3("values/laserline_3Dpoints/start_l4_d3.txt");
					start_d3 << startCam.x <<","<< startCam.y <<","<< startCam.z;
					ofstream end_d3("values/laserline_3Dpoints/end_l4_d3.txt");
					end_d3 << endCam.x <<","<< endCam.y <<","<< endCam.z;
				}
			}
			
		}
		else {imwrite("images/saved_laser_plane/laser_" + string(argv[1]) + ".jpg", line_img);}
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