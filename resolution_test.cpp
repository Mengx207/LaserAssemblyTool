
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

using namespace Pylon;
using namespace std;
using namespace cv;
using namespace GENAPI_NAMESPACE;

#ifdef PYLON_WIN_BUILD
#   include <pylon/PylonGUI.h>
#endif

int main(int argc, char* argv[])
{
	cv::Mat mono8 = cv::imread("images/mono8.png");
	cv::Mat mono12 = cv::imread("images/mono12.png");
	cv::cvtColor(mono8,mono8, cv::COLOR_BGR2GRAY);
	cv::cvtColor(mono12,mono12,cv::COLOR_BGR2GRAY);
	Vec3b intensity8, intensity12;
	for (int i=1; i<=720;)
	{ i = i+ 10;
		for(int j=1; j<=540;)
		{
			j = j+10;
			intensity8 = mono8.at<Vec3b>(i,j);
			intensity12 = mono12.at<Vec3b>(i,j);
			cout << "mono 8 intensity at (" << i <<","<< j<<") = " << intensity8 << endl;
			cout << "mono 12 intensity at (" << i <<","<< j<<") = " << intensity12 << endl <<endl;
		}	
	}
}