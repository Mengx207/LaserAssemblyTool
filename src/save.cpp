/* 
	arg[1]: set number
	exposure time = 7500us
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


int main(int argc, char* argv[])
{
    // The exit code of the sample application.
     int exitCode = 0;
     // Before using any pylon methods, the pylon runtime must be initialized.
     PylonInitialize();
     const char *err;
	 Mat src;
   
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
	
	// These allow us to convert from GrabResultPtr_t to cv::Mat
	CImageFormatConverter formatConverter;
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	INodeMap& nodemap0 = camera0.GetNodeMap();
	CEnumerationPtr(nodemap0.GetNode("ExposureMode"))->FromString("Timed"); 
	CFloatPtr(nodemap0.GetNode("ExposureTime"))->SetValue(7500.0);

	// CBooleanParameter(nodemap0, "EnableAcquisitionFrameRate").SetValue(true);
	// CFloatPtr(nodemap0.GetNode("AcquisitionFrameRate"))->SetValue(100.0);
	int setnum = stoi(argv[1]);
	int set = setnum % 3;
	cout<<endl<<set<<endl;
	switch(set)
	{
		case 0:
			CBooleanParameter(nodemap0, "AcquisitionFrameRateEnable").SetValue(true);
			CFloatParameter(nodemap0, "AcquisitionFrameRate").SetValue(100.0);
		case 1:
			CBooleanParameter(nodemap0, "AcquisitionFrameRateEnable").SetValue(true);
			CFloatParameter(nodemap0, "AcquisitionFrameRate").SetValue(10.0);
		case 2:
			CBooleanParameter(nodemap0, "AcquisitionFrameRateEnable").SetValue(true);
			CFloatParameter(nodemap0, "AcquisitionFrameRate").SetValue(50.0);
	}
	
	CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");
	CEnumParameter(nodemap0, "LineMode").SetValue("Output");
	CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput2");

	CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");
	CEnumParameter(nodemap0, "LineMode").SetValue("Output");
	CEnumParameter(nodemap0, "LineSource").SetValue("UserOutput3");

	CEnumParameter(nodemap0, "LineSelector").SetValue("Line3");			
	CBooleanParameter(nodemap0, "LineInverter").SetValue(true);
	CEnumParameter(nodemap0, "LineSelector").SetValue("Line4");			
	CBooleanParameter(nodemap0, "LineInverter").SetValue(true);

	int max_imgs0 = 20;   
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
		imgs_taken0 = imgs_taken0+1;
		cout<<endl<<imgs_taken0<<endl;
		system("cd images && mkdir -p LED_imageset");
		imwrite("images/LED_imageset/imageset_" + string(argv[1]) + "_" + to_string(imgs_taken0) + ".jpg", src);
	}
	camera0.StopGrabbing();

}
