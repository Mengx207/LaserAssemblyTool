/* 
	Assembly guidance tool - Laser dot alignment
	Help user achieve an accurate result of laser focus and laser location

	Measure laser dot's size for focusing 
	Measure nomal distance between laser dot and calculated laser line
	Meausre distance from the center of laser line to the laser dot along the line
*/
#include <iostream>
#include <utility>
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
using namespace cv;
using namespace GENAPI_NAMESPACE;


int main(int argc, char **argv)
{
    // Find planes
    pair<Mat,Mat>vec = laserline::getRvecTvec();
    cout<<vec.first<<endl<<vec.second<<endl;
    
    pair<Mat,Mat>laser = laserline::laserPlane();
    cout<<laser.first<<endl<<laser.second<<endl;

    pair<Mat,Mat>target = laserline::targetBoardPlane(vec.first, vec.second);
    cout<<target.first<<endl<<target.second<<endl;

    Mat N_B = target.first;
    Mat O_B = target.second;

    return EXIT_SUCCESS;
}
