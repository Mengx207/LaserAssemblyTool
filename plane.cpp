#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <plane.h>

cv::Mat Plane::getRvec(cv::Mat cameraMatrix, cv::Mat distCoeffs,
                       vector<Point3f>boardPoints, vector<Point2f> centers,
                       cv::Mat rvec)
{
    cv::solvePnP(boardPoints, centers, cameraMatrix, distCoeffs, rvec, tvec);
}