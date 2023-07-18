/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/

#include "include/utility.h"
#include "include/imgpro.h"
#include "include/gencal.h"

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
	 Mat src, dot_img;
   
	try
	{		
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		CInstantCamera camera0( tlFactory.CreateFirstDevice() );
        // Print the camera information.
        cout << "Using device " << camera0.GetDeviceInfo().GetModelName() << endl;
        cout << "SerialNumber : " << camera0.GetDeviceInfo().GetSerialNumber() << endl;
		cout << endl << "Program is running, select image window and press 'q' to quit."<<endl;
			
		camera0.RegisterConfiguration(new CSoftwareTriggerConfiguration1,RegistrationMode_ReplaceAll, Cleanup_Delete);

		//Create a pylon image that will be used to create an opencv image
		CPylonImage pylonImage0;
		cv::Mat cam_frame_temp0;
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
		vector<cv::Point> center_rect_list;
		double center_total = 0;
		double center_rect_count = 0;
			
		INodeMap& nodemap0 = camera0.GetNodeMap();

		CEnumerationPtr(nodemap0.GetNode("ExposureMode"))->FromString("Timed"); 
		CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(30.0);

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

		vector<double> rmatrix_laser_values;
		vector<double> tvec_laser_values;
		Point centerImage;

		while(waitKey(10) != 'q')
		{
			// cout<<endl<<"--------------------------------------------------------------------------------"<<endl;
			int max_imgs0 = 1;   
			int imgs_taken0 =0;
			camera0.StartGrabbing(max_imgs0*1);

			while (imgs_taken0 < max_imgs0) 
			{	
				CGrabResultPtr ptrGrabResult0;
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
				string path_rmatrix = "values/rmatrix_L1.txt";
				string path_tvec = "values/tvec_L1.txt";
				if(argv[1] == string("1")) 
				{
					path_rmatrix = "values/rmatrix_L1.txt";
					path_tvec = "values/tvec_L1.txt";
				}
				if(argv[1] == string("2")) 
				{
					path_rmatrix = "values/rmatrix_L2.txt";
					path_tvec = "values/tvec_L2.txt";
				}
				if(argv[1] == string("3")) 
				{
					path_rmatrix = "values/rmatrix_L3.txt";
					path_tvec = "values/tvec_L3.txt";
				}
				if(argv[1] == string("4"))
				{
					path_rmatrix = "values/rmatrix_L4.txt";
					path_tvec = "values/tvec_L4.txt";
				}

				// Calculate rotation vector and translation vector by a captured image of a pattern
				solvePnP_result solvePnP_result;
				Mat image_captured;
				if(argc == 4)
				{
					if(argv[3] == string("d1"))
					{
						image_captured = imread("images/pattern_d1.png", IMREAD_GRAYSCALE);
					}
					if(argv[3] == string("d2"))
					{
						image_captured = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);
					}
					if(argv[3] == string("d3"))
					{
						image_captured = imread("images/pattern_d3.png", IMREAD_GRAYSCALE);
					}
				}
				else {image_captured = imread("images/pattern_d2.png", IMREAD_GRAYSCALE);}

				Size patternSize (7,4);
				double squareSize = 7;
				solvePnP_result = getRvecTvec(image_captured, patternSize, squareSize);

				// read laser 1
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
				// find target board plane in cam frame
				std::vector<double> rvec_target2cam, tvec_target2cam;
				ifstream rvec_s, tvec_s;
				rvec_s.open("rvec_target2cam.txt"); 
				while (rvec_s >> val)
				{
					rvec_target2cam.push_back(val);
				}
				for(int i=0; i<rvec_target2cam.size(); i++)
				{cout<<endl<<rvec_target2cam[i]<<endl;}

				tvec_s.open("tvec_target2cam.txt"); 
				while (tvec_s >> val)
				{
					tvec_target2cam.push_back(val*1000);
				}
				for(int i=0; i<tvec_target2cam.size(); i++)
				{cout<<endl<<tvec_target2cam[i]<<endl;}

				Mat rvec, rmatrix, tvec;
				rvec = Mat(3, 1, CV_64FC1, rvec_target2cam.data());
				tvec = Mat(3, 1, CV_64FC1, tvec_target2cam.data());

				Rodrigues(rvec, rmatrix);
				cout<<endl<<tvec<<endl;
				cout<<endl<<rmatrix<<endl;
				cout<<endl<<solvePnP_result.tvec<<endl;
				cout<<endl<<solvePnP_result.rmatrix<<endl;

				pair<vector<double>,vector<double>>target = targetBoardPlane(solvePnP_result.rmatrix, solvePnP_result.tvec);
				// pair<vector<double>,vector<double>>target = targetBoardPlane(rmatrix, tvec);

				laser_plane laser_1;
				laser_1 = laserPlane(rmatrix_laser_values, tvec_laser_values);
				Point3f interPoint1;
				interPoint1 = intersectionPoint(laser_1.origin, laser_1.beam_dir, target.first, target.second);
				
				// find intersection line between target board plane and laser plane in cam frame
				std::vector<cv::Point3d> laserline_points_1, laserline_points_2, laserline_points_3;
				intersection line1, line2, line3;
				line1 = intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});
				for(int t=-150; t<150;)
				{
					t = t+10;
					Point3d points((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
					// cout<<"point: "<<points<<endl;
					laserline_points_1.push_back(points);
				}
				vector<Point2d> projectedlaserline_1;
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

			//----------raw image to greyscale, threshold filter
				cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
				dot_img = src.clone();

				cv::Mat img_grey_filtered_dot;
				cv::threshold(img_grey,img_grey_filtered_dot,250,255,cv::THRESH_OTSU||cv::THRESH_TRIANGLE);	
				line( dot_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2],Scalar(200,200,0), 5, LINE_AA );

				
				vector<Point3d> interPointArray;
				vector<Point2d> interPointsImage_vector;
				interPointArray.push_back(interPoint1);
				projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,interPointsImage_vector);
				// cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<interPointsImage_vector<<endl;
				cv::circle( dot_img, interPointsImage_vector[0], 5, cv::Scalar(200,200,0), -1, 8, 0 );


				// Number of non_zero pixel
				int non_zero = NonZero(img_grey_filtered_dot);

				// Number of bright pixel
				int count = PixelCounter(img_grey_filtered_dot);
				// average size
				size_avg = SizeAverage(count,0,size_array, center_rect_count);
				//-----------save min_size only when the value in size_array is stable and close to each other
				if(size_avg < min_size && center_rect_count > 20)
				{
					if(abs(size_array[0]-size_array[9]) <=3)
					{
						min_size = size_avg;
					}
				}
	
			 //---------Process when captured images are not empty
			 //---------Clear center list and size array is capture an empty image
			 	double nom_distance,center_distance;
				vector<cv::Vec3f> circles;
				Mat threshold_output;
				cv::threshold(img_grey,threshold_output,200,255,cv::THRESH_BINARY);
				Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );

				if(non_zero > 50)
				{
					// cv::Point center;
					// HoughCircles(img_grey_filtered_dot, circles, cv::HOUGH_GRADIENT,2, 2000,500,10,0,100);
					// if(!circles.empty())
					// {
					// 	center.x = cvRound(circles[0][0]);
					// 	center.y = cvRound(circles[0][1]);
					// 	circle (drawing, center, 2, cv::Scalar(0,0,255), -1, 8, 0);
					// 	center_list.push_back(center);
					// 	center_total ++;
					// }
					// if (center_total > 30)
					// {
					// 	center_list.erase(center_list.begin());
					// 	center_total --;
					// }
					
					// if (center_total >= 10)
					// {
					// 	cv::Point sum  = std::accumulate(center_list.begin(), center_list.end(), cv::Point(0,0));
					// 	center_avg = sum*(1.0/center_total);

					// 	sleep(0.1);

					// 	cv::circle( dot_img, center_avg, 3, cv::Scalar(255,100,0), -1, 8, 0 );

					// 	// std::pair<double,double>dist = DotToLine(src, line_1_Start, line_1_End, center_avg, Point2d(interPoint1.x+720,-interPoint1.y+540));
					// 	std::pair<double,double>dist = DotToLine(dot_img, projectedlaserline_1[0], projectedlaserline_1[19], center_avg, interPointsImage_vector[0]);
					// 	nom_distance = dist.first;
					// 	center_distance = dist.second;
					// }
					
					// Test another way of finding center of laser dot
					vector<vector<Point> > contours;
					vector<Vec4i> hierarchy;
					Point center_rect_avg;
					findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
					if(contours.size() > 0)
					{
						vector<RotatedRect> minRect = findRectangle(contours,20);
						if(minRect.size() == 1)
						{	
							center_rect_list.push_back(minRect[0].center);
							center_rect_count++;
							drawContourRectangle(drawing, contours, minRect);
							cv::circle( dot_img, minRect[0].center, 3, cv::Scalar(100,255,0), -1, 8, 0 );
							// cout<<endl<<"minRect center: "<< minRect[0].center<<endl;
						}
						if (center_rect_count > 30)
						{
							center_rect_list.erase(center_rect_list.begin());
							center_rect_count --;
						}
						if (center_rect_count >= 10)
						{
							Point sum = accumulate(center_rect_list.begin(), center_rect_list.end(), Point(0,0));
							center_rect_avg = sum*(1.0/center_rect_count);

							circle(dot_img, center_rect_avg, 3, Scalar(255,0,255), -1, 8, 0);
							std::pair<double,double>dist = DotToLine(dot_img, projectedlaserline_1[0], projectedlaserline_1[19], center_rect_avg, interPointsImage_vector[0]);
							nom_distance = dist.first;
							center_distance = dist.second;
							centerImage = center_rect_avg;
						}
					}

				}
				else
				{
					last_min_size = min_size;
					fill_n(size_array,10,0);
					center_rect_count = 0;
					center_rect_list.clear();
					//fill(center_list.begin(), center_list.end(), cv::Point(0,0));
				}

				// Point3d point_1 = locationCam2Target(interPointsImage_vector[0], solvePnP_result);

				HMI(dot_img, size_avg, min_size, non_zero, nom_distance, center_distance);
				GreenLight(dot_img, last_min_size, size_avg, nom_distance, center_distance);
				
				// cv::imshow("img_grey_filtered_dot", img_grey_filtered_dot);	
				// cv::imshow("threshold output", threshold_output);
				// cv::imshow("Contour and Rectangle", drawing);	  
				cv::imshow("Laser Beam Alignment Window", dot_img);							
				imgs_taken0++;
			}
			camera0.StopGrabbing();
		}
		system("cd images && mkdir -p saved_laser_beam");
		if (argc == 3)
		{
			imwrite("images/saved_laser_beam/laser_" + string(argv[1]) + "_" + string(argv[2]) + ".jpg", dot_img);
		}
		else if (argc == 4)
		{
			imwrite("images/saved_laser_beam/laser_" + string(argv[1]) + "_" + string(argv[2]) + "_" + string(argv[3]) + ".jpg", dot_img);	
			
			// save center_rect_avg for future use
			system("cd values && mkdir -p intersections && cd intersections");
			ofstream interpoint("values/intersections/intersections_l"+string(argv[1])+"_"+string(argv[3])+".txt");
			interpoint << centerImage.x <<" ";
			interpoint << centerImage.y;
			// if(argv[3] == string("d1"))
			// {
			// 	ofstream interpoint("values/intersections/intersections_d1.txt");
			// 	interpoint << centerImage.x <<" ";
			// 	interpoint << centerImage.y;
			// }
			// if(argv[3] == string("d2"))
			// {
			// 	ofstream interpoint("values/intersections/intersections_d2.txt");
			// 	interpoint << centerImage.x <<" ";
			// 	interpoint << centerImage.y;
			// }
			// if(argv[3] == string("d3"))
			// {
			// 	ofstream interpoint("values/intersections/intersections_d3.txt");
			// 	interpoint << centerImage.x <<" ";
			// 	interpoint << centerImage.y;
			// }
		}
		else {imwrite("images/saved_laser_beam/laser_" + string(argv[1]) + ".jpg", dot_img);}

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