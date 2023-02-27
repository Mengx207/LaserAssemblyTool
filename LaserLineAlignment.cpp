/* 
	Assembly guidance tool - Laser line alignment
	Help user achieve an accurate result of laser line location, orientation and quality
*/

#include "myHeader.h"

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 10;

// Limits the amount of cameras used for grabbing.
static const size_t c_maxCamerasToUse = 2;

//If camera features file is used
//const char Filename[] = "acA2440-75um_23663771.pfs";
using namespace GENAPI_NAMESPACE;

void MyLine( Mat img, Point start, Point end );
void HMI(Mat img, int size, int min_size, int non_zero, int nom_distance, int center_distance);
void GreenLight(Mat img, int last, int current, int nom_distance, int center_distance);
int NonZero (Mat img, int count);
int PixelCounter(Mat img, int count);
int SizeAverage (int count, int size_avg,int size_array[], int center_total);
int ClearList(vector<cv::Point> center_list, int center_total, int size_array[], int min);
void DotToLine(Mat img, Point start, Point end, Point center, double nom_distance, double center_distance);

struct {
	double nom_distance, center_distance;
} dotLine;

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
		Mat canny_edge, canny_edge_blur, img_grey, img_grey_blur;

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
		long valid_num = 0;

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

			while (imgs_taken0 < max_imgs0) 
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
				threshold(img_grey,img_grey_filtered,250,255,THRESH_OTSU||THRESH_TRIANGLE);	

				// Number of non_zero pixel
				int non_zero = NonZero(img_grey_filtered, 0);

				// Number of bright pixel
				int count = PixelCounter(img_grey_filtered,0);

				// average size
				size_avg = SizeAverage(count,0,size_array, valid_num);
				//-----------save min_size only when the value in size_array is stable and close to each other
				if(size_avg < min_size && center_total > 30)
				{
					if(abs(size_array[0]-size_array[9]) <=3)
					{
						min_size = size_avg;
					}
				}

				// Standard Hough Line Transform
				Mat houghline = cv::Mat::zeros({img_grey_filtered.size()},CV_8UC3);
				GaussianBlur(img_grey_filtered,img_grey_blur, Size(5, 5), 0, 0, BORDER_DEFAULT);
			    vector<Vec4i> linesP; // will hold the results of the detection
				HoughLinesP(img_grey_blur, linesP, 1, CV_PI/180, 50, 500, 1 ); // runs the actual detection
				// Draw the lines
				for( size_t i = 0; i < linesP.size(); i++ )
				{
					Vec4i l = linesP[i];
					line( houghline, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 3, LINE_AA);
				}

			 //---------Clear center list and size array is capture an empty image
				if(non_zero > 20)
				{	
					valid_num ++;
				}
				else
				{
					valid_num = 0;
					last_min_size = min_size;
					cout<<"last minum size: "<<last_min_size<<endl;
					fill_n(size_array,10,0);
				}
				//cout<<valid_num<<endl;

				MyLine( src, Point( 300, 200 ), Point( 700, 900 ) );
				//DotToLine(src,  Point( 300, 200 ), Point( 700, 900 ), center_avg, dotLine.nom_distance, dotLine.center_distance);
				//HMI(src, size_avg, min_size, non_zero, dotLine.nom_distance, dotLine.center_distance);
				//GreenLight(src, last_min_size, size_avg, dotLine.nom_distance, dotLine.center_distance);

				//imshow("img_grey_filtered", img_grey_filtered);	
				imshow("source window", src);
				imshow("filtered", img_grey_filtered);		
				imshow("filtered blurred", img_grey_blur);		
				imshow("houghline", houghline);			
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
void HMI(Mat img, int size, int min_size, int non_zero, int nom_distance, int center_distance)
{
	std::string size_print = "No value";
	std::string min_size_print = "No value";
	std::string nom_distance_print = "No value";
	std::string center_distance_print = "No value";
	if(non_zero >20)		
	{
		size_print = std::to_string(size);
		min_size_print = std::to_string(min_size);
		nom_distance_print = std::to_string(nom_distance);
		center_distance_print = std::to_string(center_distance);
	}
	putText(img, "Laser Focus:", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Laser Dot Size: "+size_print, Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Last Dot Size: "+min_size_print, Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Laser Focus Status: ", Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);

	putText(img, "Laser Dot Location:", Point(500, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Nominal Distance: "+nom_distance_print, Point(500, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Distance from Center: "+center_distance_print, Point(500, 80), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);
	putText(img, "Dot Location Status: ", Point(500, 120), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255),2);

}

void GreenLight(Mat img, int last, int current, int nom_distance, int center_distance)
{
	if(last-current > 0 || abs(last-current) < 5)
	{
		circle( img, Point(300,110), 20, Scalar(0,255,0), -1, 8, 0 );
	}
	else
	{
		circle( img, Point(300,110), 20, Scalar(0,0,255), -1, 8, 0 );
	}

	if(nom_distance < 50 && center_distance < 50)
	{
		circle( img, Point(800,110), 20, Scalar(0,255,0), -1, 8, 0 );
	}
	else
	{
		circle( img, Point(800,110), 20, Scalar(0,0,255), -1, 8, 0 );
	}
}

int SizeAverage (int count, int size_avg, int size_array[], int center_total)
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

	if(size_array[9]!=0 && center_total > 50)
	{
		size_avg = size_sum/10;
		return size_avg;
	}
}

int ClearList(vector<cv::Point> center_list, int center_total, int size_array[], int min)
{
	int last_min = min;
	cout<<"last minum size: "<<last_min<<endl;
	fill_n(size_array,10,0);
	center_total = 0;
	fill(center_list.begin(), center_list.end(), Point(0,0));
	return last_min;
}

void DotToLine(Mat img, Point start, Point end, Point center, double nom_distance, double center_distance)
{
	LineIterator laserline(img, start, end, 8);
	vector<Vec3b> buf(laserline.count);
	vector<double> distance_list;
	vector<cv::Point> point_list;
	for(int i = 0; i < laserline.count; i++, ++laserline)
	{
		point_list.push_back(laserline.pos());
		double distance = norm(center-laserline.pos());
		distance_list.push_back(distance);
	}
	double min_distance = *min_element(distance_list.begin(), distance_list.end());
	vector<double>::iterator result = min_element(distance_list.begin(), distance_list.end());
	int num = distance(distance_list.begin(), result);

	line( img, center, point_list[num], Scalar( 255, 255, 0 ), 1, 8 );
	dotLine.nom_distance = min_distance;
	dotLine.center_distance = norm(point_list[num]-point_list[(laserline.count)/2]);
	Point laserline_center = point_list[(laserline.count)/2];
	circle( img, laserline_center, 5, Scalar(0,0,255), -1, 8, 0 );

	cout << "min point at: " << point_list[num] <<endl;
	cout<<"center of line: "<<laserline_center<<endl;
	cout<<"nominal_distance: "<<nom_distance<<endl;
	cout<<"distance from line center: "<<center_distance<<endl;
}