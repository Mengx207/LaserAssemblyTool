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
#include <pylon/PylonGUI.h>
#endif

// void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, uniformity_data uniformity1, Mat drawing);

int main(int argc, char *argv[])
{
	// The exit code of the sample application.
	int exitCode = 0;
	// Before using any pylon methods, the pylon runtime must be initialized.
	PylonInitialize();
	const char *err;
	try
	{
		CTlFactory &tlFactory = CTlFactory::GetInstance();
		CInstantCamera camera0(tlFactory.CreateFirstDevice());
		// Print the camera information.
		cout <<endl<< "Using device " << camera0.GetDeviceInfo().GetModelName() << endl;
		cout << "SerialNumber : " << camera0.GetDeviceInfo().GetSerialNumber() << endl;
		cout << "Double tap q to quit without saving."<<endl<<"Tap q then s to quit and save." << endl;

		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration1, RegistrationMode_ReplaceAll, Cleanup_Delete);

		// Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0;
		Mat src, cam_frame_temp0, line_img, threshold_output;
		camera0.Open();

		// These allow us to convert from GrabResultPtr_t to cv::Mat
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;

		int size_avg;
		int min_size = 1000;
		int size_array[10] = {0};
		int last_size_avg;
		int last_min_size = 0;
		cv::Point center_avg;
		bool version_V4 = false;

		vector<cv::Point> center_list;
		double center_total = 0;

		INodeMap &nodemap0 = camera0.GetNodeMap();

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

		if (argv[2] == string("1") || argv[2] == string("3"))
		{
			CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
			CBooleanParameter(nodemap0, "LineInverter").SetValue(false);

			CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
			CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
		}
		if (argv[2] == string("2") || argv[2] == string("4"))
		{
			CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
			CBooleanParameter(nodemap0, "LineInverter").SetValue(true);

			CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
			CBooleanParameter(nodemap0, "LineInverter").SetValue(false);
		}

		std::vector<double> rvec_target2cam, tvec_target2cam;
		Mat rvec, rmatrix, tvec;
		solvePnP_result solvePnP_result;
		bool rect_find_flag = false;

		while (waitKey(10) != 'q')
		{
			int max_imgs0 = 1;
			int imgs_taken = 0;

			camera0.StartGrabbing(max_imgs0 * 1);

			while (imgs_taken < max_imgs0)
			{
				CGrabResultPtr ptrGrabResult0;
				CGrabResultPtr ptrGrabResult1;
				while (camera0.WaitForFrameTriggerReady(1000, TimeoutHandling_ThrowException) == 0)
					;
				CCommandParameter(nodemap0, "TriggerSoftware").Execute();
				bool test0 = camera0.RetrieveResult(1000, ptrGrabResult0, TimeoutHandling_ThrowException);
				formatConverter.Convert(pylonImage0, ptrGrabResult0);

				cam_frame_temp0 = cv::Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *)pylonImage0.GetBuffer());
				src = cam_frame_temp0.clone();
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
				string path_L_rmatrix;
				string path_L_tvec;
				
				if (argv[1] == string("V3"))
				{
					version_V4 = false;
					if (argv[2] == string("1"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L1_V3.txt";
						path_L_tvec = "values/laser_transform/tvec_L1_V3.txt";
					}
					if (argv[2] == string("2"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L2_V3.txt";
						path_L_tvec = "values/laser_transform/tvec_L2_V3.txt";
					}
					if (argv[2] == string("3"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L3_V3.txt";
						path_L_tvec = "values/laser_transform/tvec_L3_V3.txt";
					}
					if (argv[2] == string("4"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L4_V3.txt";
						path_L_tvec = "values/laser_transform/tvec_L4_V3.txt";
					}
				}
				else if (argv[1] == string("V4"))
				{
					version_V4 = true;
					if (argv[2] == string("1"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L1_V4.txt";
						path_L_tvec = "values/laser_transform/tvec_L1_V4.txt";
					}
					if (argv[2] == string("2"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L2_V4.txt";
						path_L_tvec = "values/laser_transform/tvec_L2_V4.txt";
					}
					if (argv[2] == string("3"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L3_V4.txt";
						path_L_tvec = "values/laser_transform/tvec_L3_V4.txt";
					}
					if (argv[2] == string("4"))
					{
						path_L_rmatrix = "values/laser_transform/rmatrix_L4_V4.txt";
						path_L_tvec = "values/laser_transform/tvec_L4_V4.txt";
					}
				}
				// Calculate rotation vector and translation vector by a captured image of a pattern
				Mat image_captured;
				Size patternSize(7, 4);
				double squareSize = 7.044;
				image_captured = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
	
				if (argc == 5)
				{
					if (argv[4] == string("d1"))
					{
						image_captured = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
					}
					else if (argv[4] == string("d2"))
					{
						image_captured = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);
					}
					else{
						cout<<endl<<"Invalid Distance"<<endl;
					}
				}
				cv::threshold(image_captured, image_captured, 120, 255, cv::THRESH_BINARY);
				imshow("image_captured", image_captured);
				solvePnP_result = getRvecTvec(image_captured, patternSize, squareSize);
				cout<<endl<<"tvec:"<<endl<<solvePnP_result.tvec<<endl;
				cout<<endl<<"rvec:"<<endl<<solvePnP_result.rvec<<endl;

				// read laser 1
				ifstream rmatrixL(path_L_rmatrix);
				vector<double> rmatrix_laser_values;
				while (rmatrixL >> val)
				{
					rmatrix_laser_values.push_back(val);
				}
				ifstream tvecL(path_L_tvec);
				vector<double> tvec_laser_values;
				while (tvecL >> val)
				{
					tvec_laser_values.push_back(val * 1000);
				}
				// find target board plane in cam frame
				pair<vector<double>, vector<double>> target = targetBoardPlane(solvePnP_result.rmatrix, solvePnP_result.tvec);

				laser_plane laser_plane_result;
				laser_plane_result = laserPlane(rmatrix_laser_values, tvec_laser_values);

				// find intersection between laser beam and target board
				Point3f intersection_point;
				intersection_point = intersectionPoint(laser_plane_result.origin, laser_plane_result.beam_dir, target.first, target.second);
				vector<Point3d> intersection_point_vector;
				vector<Point2d> intersection_point_projected;
				intersection_point_vector.push_back(intersection_point);
				projectPoints(intersection_point_vector, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), cameraMatrix, distCoeffs, intersection_point_projected);
				// cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<intersection_point_projected<<endl;

				// find intersection line between target board plane and laser plane in cam frame
				std::vector<cv::Point3d> laserlinepoints_vector;
				intersection line1, line2, line3;
				line1 = intersectionLine(target.first, laser_plane_result.normalvector, target.second, vector<double>{intersection_point.x, intersection_point.y, intersection_point.z});
				for (int t = -150; t < 150;)
				{
					t = t + 10;
					Point3d points((line1.x0 + line1.a * t), (line1.y0 + line1.b * t), (line1.z0 + line1.c * t));
					// cout<<"point: "<<points<<endl;
					laserlinepoints_vector.push_back(points);
				}
				vector<Point2d> laserlinepoints_projected;
				projectPoints(laserlinepoints_vector, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), cameraMatrix, distCoeffs, laserlinepoints_projected);

				double x_max = 0;
				double x_min = 1440;
				int i_max, i_min, i_start, i_end;
				//filter our the points that out of the image size and find the min and max value in points vector
				for (int i = 0; i < laserlinepoints_projected.size();)
				{
					if ((laserlinepoints_projected[i].x > 1440) || (laserlinepoints_projected[i].y > 1080) || (laserlinepoints_projected[i].x < 0) || (laserlinepoints_projected[i].y < 0))
					{
						laserlinepoints_projected.erase(laserlinepoints_projected.begin() + i);
					}
					else
					{
						if(laserlinepoints_projected[i].x > x_max)
						{
							x_max = laserlinepoints_projected[i].x;
							i_max = i;
						}
						if(laserlinepoints_projected[i].x < x_min)
						{
							x_min = laserlinepoints_projected[i].x;
							i_min = i;
						}
						i++;
					}
				}

				if (i_max > i_min)
				{
					i_start = i_min;
					i_end = i_max;
				}
				else 
				{
					i_start = i_max;
					i_end = i_min;
				}
	
				// Calculated laser line angle
				double delta_y = (laserlinepoints_projected[i_end].y - laserlinepoints_projected[i_start].y);
				double delta_x = (laserlinepoints_projected[i_end].x - laserlinepoints_projected[i_start].x);
				double cal_angle = atan(delta_y / delta_x) * 180 / CV_PI;

				Mat img_grey;
				cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
				line_img = src.clone();

				vector<vector<Point>> contours;
				vector<Vec4i> hierarchy;
				cv::threshold(img_grey, threshold_output, 200, 255, cv::THRESH_BINARY);

				findContours(threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
				vector<RotatedRect> Rect_vector = findRectangle(contours, 100);
				Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
				Mat rotated_image = threshold_output.clone();
				uniformity_data uniformity1;
				int status = 0;
				if (Rect_vector.size() >= 1)
				{
					rect_find_flag = true;
					drawContourRectangle(drawing, contours, Rect_vector);
					// circle(line_img, Rect_vector[0].center, 5, Scalar(0,0,250), -1, 8, 0);
					double angle = Rect_vector[0].angle;
					if (Rect_vector[0].size.width < Rect_vector[0].size.height)
					{
						angle = 90 + angle;
					}
					Mat rotation_matrix = getRotationMatrix2D(Rect_vector[0].center, angle, 1.0);
					warpAffine(threshold_output, rotated_image, rotation_matrix, threshold_output.size());
					uniformity1 = cropImage(rotated_image);
					// From pixel number to actual diameter on target board
					uniformity1.width_avg = uniformity1.width_avg * 3.45 * (solvePnP_result.tvec.at<double>(0, 2) / 12) / 1000;
					uniformity1.width_max = uniformity1.width_max * 3.45 * (solvePnP_result.tvec.at<double>(0, 2) / 12) / 1000;
					uniformity1.width_min = uniformity1.width_min * 3.45 * (solvePnP_result.tvec.at<double>(0, 2) / 12) / 1000;
					// cv::imshow( "Rotated and Cropped laser line", uniformity1.image_BGR );
					extractLaserline2Points(threshold_output);
				}
				else
				{
					rect_find_flag = false;
					Rect_vector.empty();
					RotatedRect fake_rect = RotatedRect((Point2f(0,0)), Size2f(1,1), float (0));
					Rect_vector.push_back(fake_rect);
				}
				status = laserlineGUI(Rect_vector, intersection_point_projected[0], cal_angle, uniformity1, line_img);
				if (argv[4] == string("d2"))
				{
					putText(line_img, "THIS TARGET POSSITION IS FOR MEASUREMENT ONLY", cv::Point(300, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(250,250,250),2);
					putText(line_img, "DO NOT ADJUST THE LASER", cv::Point(300, 550), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(250,250,250),2);
				}

				if(status == 0)
				{line(line_img, laserlinepoints_projected[i_start], laserlinepoints_projected[i_end], Scalar(0, 0, 250), 3, LINE_AA);}
				else if(status == 1)
				{line(line_img, laserlinepoints_projected[i_start], laserlinepoints_projected[i_end], Scalar(0, 250, 0), 3, LINE_AA);} // green line
				// cout<<endl<<"laser line points: "<<endl<<laserlinepoints_projected<<endl;
				cv::circle(line_img, intersection_point_projected[0], 5, cv::Scalar(0, 0, 250), -1, 8, 0);
				// cout<<endl<<"intersection point: "<<endl<<intersection_point_projected[0]<<endl;

				// cv::imshow( "Contour and Area", drawing );
				// cv::imshow("threshold",threshold_output);
				cv::imshow("Laser Plane Alignment GUI Window", line_img);
				imgs_taken++;
			}
			camera0.StopGrabbing();
		}
		char command = waitKey();
		if(command == 's')
		{
			system("cd images && mkdir -p saved_laser_plane");
			if (argc == 4)
			{
				imwrite("images/saved_laser_plane/laser_" + string(argv[2]) + "_" + string(argv[3]) + ".jpg", line_img);
			}
			else if (argc == 5) // save data to folder for future verification and record
			{
				imwrite("images/saved_laser_plane/laser_" + string(argv[2]) + "_" + string(argv[3]) + "_" + string(argv[4]) + ".jpg", line_img);
				// imwrite("images/saved_laser_plane/laser_plane_" + string(argv[2]) + "_" + string(argv[3]) + "_" + string(argv[4]) + "_threshold.jpg", threshold_output);

				system("cd values && mkdir -p real_laserline_ends && cd real_laserline_ends");

				pair<Point2f, Point2f> laserline2Points;
				// laserline2Points.first = Point2d(0, 0);
				// laserline2Points.second = Point2d(1440, 1080);
				Point3d startCam(0,0,0);
				Point3d endCam(0,0,0);

				if(rect_find_flag == true)
				{
					laserline2Points = extractLaserline2Points(threshold_output);
					// cout << "Pair of end points of actual laser line on image plane: " << laserline2Points.first << ", " << laserline2Points.second << endl;
					startCam = locationCam2Target(laserline2Points.first, solvePnP_result, version_V4);
					endCam = locationCam2Target(laserline2Points.second, solvePnP_result, version_V4);
				}

				if (argv[2] == string("1"))
				{
					if (argv[4] == string("d1"))
					{
						ofstream start_d1("values/real_laserline_ends/start_l1_d1.txt");
						start_d1 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d1("values/real_laserline_ends/end_l1_d1.txt");
						end_d1 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
					else if (argv[4] == string("d2"))
					{
						ofstream start_d2("values/real_laserline_ends/start_l1_d2.txt");
						start_d2 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d2("values/real_laserline_ends/end_l1_d2.txt");
						end_d2 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
				}
				if (argv[2] == string("2"))
				{
					if (argv[4] == string("d1"))
					{
						ofstream start_d1("values/real_laserline_ends/start_l2_d1.txt");
						start_d1 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d1("values/real_laserline_ends/end_l2_d1.txt");
						end_d1 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
					else if (argv[4] == string("d2"))
					{
						ofstream start_d2("values/real_laserline_ends/start_l2_d2.txt");
						start_d2 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d2("values/real_laserline_ends/end_l2_d2.txt");
						end_d2 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
				}
				if (argv[2] == string("3"))
				{
					if (argv[4] == string("d1"))
					{
						ofstream start_d1("values/real_laserline_ends/start_l3_d1.txt");
						start_d1 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d1("values/real_laserline_ends/end_l3_d1.txt");
						end_d1 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
					else if (argv[4] == string("d2"))
					{
						ofstream start_d2("values/real_laserline_ends/start_l3_d2.txt");
						start_d2 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d2("values/real_laserline_ends/end_l3_d2.txt");
						end_d2 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
				}
				if (argv[2] == string("4"))
				{
					if (argv[4] == string("d1"))
					{
						ofstream start_d1("values/real_laserline_ends/start_l4_d1.txt");
						start_d1 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d1("values/real_laserline_ends/end_l4_d1.txt");
						end_d1 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
					else if (argv[4] == string("d2"))
					{
						ofstream start_d2("values/real_laserline_ends/start_l4_d2.txt");
						start_d2 << startCam.x << "," << startCam.y << "," << startCam.z;
						ofstream end_d2("values/real_laserline_ends/end_l4_d2.txt");
						end_d2 << endCam.x << "," << endCam.y << "," << endCam.z;
					}
				}
			}
			else
			{
				imwrite("images/saved_laser_plane/laser_" + string(argv[2]) + ".jpg", line_img);
			}
			cout<<endl<<"Save and Quit"<<endl;
		}
		else if (command == 'q')
		{cout<<endl<<"Quit without saving"<<endl;}
		
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
