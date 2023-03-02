/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/

#include "lib/myHeader.h"
#include "utility/include/utility.h"
#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif

// Number of images to be grabbed.s
static const uint32_t c_countOfImagesToGrab = 10;

// Limits the amount of cameras used for grabbing.
static const size_t c_maxCamerasToUse = 2;

//If camera features file is used
//const char Filename[] = "acA2440-75um_23663771.pfs";
using namespace GENAPI_NAMESPACE;

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
		CPylonImage pylonImage0, pylonImage1;

		//Mat src, lft1_img, rgt0_img, rgt1_img, cam_frame_temp0, cam_frame_temp1;
		Mat src, cam_frame_temp0;
		camera0.Open();
		
		Point max_point;
		Mat canny_edge, canny_edge_blur, img_grey;

		// If camera features file is used
		//CFeaturePersistence::Load( Filename, &cameras[0].GetNodeMap(), true ); 

		
		// These allow us to convert from GrabResultPtr_t to cv::Mat
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;

		int size_avg ;
        int min_size = 1000;
		int size_array[10] = {0};
		int last_size_avg;
		int last_min_size=0;
		Point center_avg;

		vector<cv::Point> center_list;
		double center_total = 0;

		while(1)
		{
			int max_imgs0 = 50;   
			int imgs_taken0 =0;
	
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
			
			camera0.StartGrabbing(max_imgs0*1);

			while (imgs_taken0< max_imgs0) 
			{	

				CGrabResultPtr ptrGrabResult0;
				CGrabResultPtr ptrGrabResult1;
		
	         //----------Triggering laser and capturing image continously-----------------------------------------
				CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");			
				CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
				CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
				CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
				sleep(0.1);

				while(camera0.WaitForFrameTriggerReady(1000,TimeoutHandling_ThrowException)==0);			
				CCommandParameter(nodemap0, "TriggerSoftware").Execute();
				
				bool test0 = camera0.RetrieveResult(1000, ptrGrabResult0, TimeoutHandling_ThrowException);

				formatConverter.Convert(pylonImage0, ptrGrabResult0);
//-----------------------Main------Functionalities------Start------From------Here-----------------------------------------------------------------------------				
				cam_frame_temp0 = Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *) pylonImage0.GetBuffer());

				src = cam_frame_temp0.clone();
				//Test intersection function
				int N1[3] = {10,8,3};
				int N2[3] = {2,6,5};
				int point1[3] = {10,5,5};
				int point2[3] = {2,2,2};
    			//intersection::intersectionLine(N1, N2, point1, point2);


				laserdot::CalculatedLine( src, Point( 300, 200 ), Point( 700, 900 ) );

			 //----------raw image to greyscale, threshold filter
				cvtColor(src, img_grey, COLOR_BGR2GRAY);
				Mat img_grey_filtered;
				threshold(img_grey,img_grey_filtered,250,255,THRESH_OTSU||THRESH_TRIANGLE);	

				// Number of non_zero pixel
				int non_zero = laserdot::NonZero(img_grey_filtered, 0);

				// Number of bright pixel
				int count = laserdot::PixelCounter(img_grey_filtered,0);
				// average size
				size_avg = laserdot::SizeAverage(count,0,size_array, center_total);
				//-----------save min_size only when the value in size_array is stable and close to each other
				if(size_avg < min_size && center_total > 30)
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
				vector<Vec3f> circles;
				if(non_zero > 20)
				{	
					HoughCircles(img_grey_filtered, circles, HOUGH_GRADIENT,2, 2000,500,10,0,100);
					sleep(0.1);
					Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
					center_list.push_back(center);
					center_total ++;
					if (center_total >= 10)
					{
						Point sum  = std::accumulate(center_list.begin(), center_list.end(), Point(0,0));
						center_avg = sum*(1.0/center_total);
						cout<<"Average center: "<< center_avg<<endl;
						sleep(0.1);
						circle( img_grey_filtered, center_avg, 3, Scalar(255,0,0), -1, 8, 0 );
						circle( src, center_avg, 3, Scalar(255,100,0), -1, 8, 0 );
					}
					std::pair<double,double>dist = laserdot::DotToLine(src,  Point( 300, 200 ), Point( 700, 900 ), center_avg);
					nom_distance,center_distance = dist.first,dist.second;
				}
				else
				{
					last_min_size = min_size;
					cout<<"last min size: "<<last_min_size<<endl;
					fill_n(size_array,10,0);
					center_total = 0;
					fill(center_list.begin(), center_list.end(), Point(0,0));
				}
				laserdot::HMI(src, size_avg, min_size, non_zero, nom_distance, center_distance);
				laserdot::GreenLight(src, last_min_size, size_avg, nom_distance, center_distance);

				imshow("img_grey_filtered", img_grey_filtered);	
				imshow("source window", src);							
				waitKey( 10 );		
				sleep(0.1);
				imgs_taken0++;
			}
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