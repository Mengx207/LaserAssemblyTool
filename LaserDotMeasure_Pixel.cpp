/* 666
	Assembly guidance tool
	Help people achieve a more accurate result 
	of laser focus, laser line alignment and camera focus.

*/

#include "myHeader.h"

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Namespace for CV objects
using namespace cv;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 10;

// Limits the amount of cameras used for grabbing.
static const size_t c_maxCamerasToUse = 2;

//If camera features file is used
//const char Filename[] = "acA2440-75um_23663771.pfs";
using namespace GENAPI_NAMESPACE;

char window_name[] = "Laser Dot Finder";

void MyLine( Mat img, Point start, Point end );
void HMI(Mat img, int size, int min_size, int non_zero);
void GreenLight(int avg_last, int avg);
int NonZero (Mat img, int count);
int PixelCounter(Mat img, int count);
int SizeAverage (int count, int size_avg,int size_array[]);

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

		Mat src, lft1_img, rgt0_img, rgt1_img, cam_frame_temp0, cam_frame_temp1;
		camera0.Open();
		
		Point max_point;
		Mat canny_edge, canny_edge_blur, img_grey;

		// If camera features file is used
		//CFeaturePersistence::Load( Filename, &cameras[0].GetNodeMap(), true ); 

		
		// These allow us to convert from GrabResultPtr_t to cv::Mat
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;

		int size_avg = 0;
        int min_size = 10000;
		int size_array[10] = {0};
		int last_size_avg;
		int last_min_size;

		while(1)
		{
			int max_imgs0 = 50;   
			int imgs_taken0 =0;
	
			INodeMap& nodemap0 = camera0.GetNodeMap();

			CEnumerationPtr(nodemap0.GetNode("ExposureMode"))->FromString("Timed"); 
			CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(200.0);
						
			//double d = CFloatParameter(nodemap0, "SensorReadoutTime").GetValue();
			//cout << "readout   "  << d <<endl;

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
				
				cam_frame_temp0 = Mat(ptrGrabResult0->GetHeight(), ptrGrabResult0->GetWidth(), CV_8UC3, (uint8_t *) pylonImage0.GetBuffer());

				src = cam_frame_temp0.clone();

			 //----------raw image to greyscale, threshold filter
				cvtColor(src, img_grey, COLOR_BGR2GRAY);
				Mat img_grey_filtered;
				threshold(img_grey,img_grey_filtered,200,255,THRESH_OTSU||THRESH_TRIANGLE);	

				// Number of non_zero pixel
				int non_zero = NonZero(img_grey_filtered, 0);

				// Number of bright pixel
				int count = PixelCounter(img_grey_filtered,0);
				cout<<"pixel count: "<<count<<endl;

				size_avg = SizeAverage(count,0,size_array);
				//cout<<"average size: "<< size_avg<<endl;

				if(size_avg < min_size && (min_size-size_avg <=2 || min_size-size_array[0] > 50) && size_array[9]>0)
				{
					min_size = size_avg;
				}

			 //---------Draw circles based on collected points	
				vector<Vec3f> circles;
				HoughCircles(img_grey_filtered, circles, HOUGH_GRADIENT,4, 1000,500,10,0,100);
				sleep(0.1);
				
				if(non_zero != 0)
				{	
					Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
					//radius = cvRound(circles[0][2]);
					circle( img_grey_filtered, center, 3, Scalar(255,0,0), -1, 8, 0 );
					circle( src, center, 3, Scalar(255,100,0), -1, 8, 0 );
					//cout<<"center: "<<center<<endl;
				}
				else
				{
					last_min_size = min_size;
					cout<<"last minum size: "<<last_min_size<<endl;
					last_size_avg = size_avg;
					fill_n(size_array,10,0);
					//min_size = 10000;
				}

				MyLine( src, Point( 300, 200 ), Point( 700, 900 ) );

				HMI(src, size_avg, min_size, non_zero);

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

//---------Draw the desired laser line
void MyLine( Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = LINE_8;

  line( img, start, end, Scalar( 0, 0, 255 ), thickness, lineType );
}

//----------Use Canny to find the Canny edges of objects in image
//Canny edge is a good way to count non-zero pixel
int NonZero(Mat img, int count)
{
	Mat canny_edge, canny_edge_blur;
	Canny(img, canny_edge, 100, 200, 5, false);
	GaussianBlur( canny_edge, canny_edge_blur, Size(5, 5), 2, 2 );
	count = countNonZero(canny_edge_blur);
	return count;
}

//-----------Measure the size of laser dor by counting pixel after threshold
int PixelCounter(Mat img, int count)
{
	Mat img_nominal;
	img.convertTo(img_nominal, CV_32F);
	for(int i=0; i<img_nominal.rows; i++)
	{
		for(int j=0; j<img_nominal.cols; j++)
		{
			if(img_nominal.at<float>(i,j)<=0.0)
			{
				count++;
				//cout<<img_nominal.at<float>(i,j)<<endl;
				//cout<<i<<","<<j<<endl;
			}
		}
	}
	return count;
}

//----------Print information in window
void HMI(Mat img, int size, int min_size, int non_zero)
{
	std::string size_print = "No value";
	std::string min_size_print = "No value";
	if(non_zero != 0)		
	{
		size_print = std::to_string(size);
		min_size_print = std::to_string(min_size);
	}
	putText(img, "Laser focus tool", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Laser dot size : "+size_print, Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Min size : "+min_size_print, Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Status : ", Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
}

void GreenLight(Mat img, int avg_last, int avg)
{
	if(avg_last-avg > 0)
	{
		circle( img, Point(120,110), 20, Scalar(0,255,0), -1, 8, 0 );
	}
	else
	{
		circle( img, Point(120,110), 20, Scalar(0,0,255), -1, 8, 0 );
	}
}

int SizeAverage (int count, int size_avg, int size_array[])
{
	for(int i=9; i>0; i--)
	{
		size_array[i] = size_array[i-1];
	}
	size_array[0] = count;

	int size_sum = 0;
	for(int i=0; i<10; i++)
	{
		size_sum = size_sum + size_array[i];
	}
	if(size_array[9]!=0)
	{
		size_avg = size_sum/10;
		return size_avg;
	}
}
