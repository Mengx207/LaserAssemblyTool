#ifndef PLANE_H
#define PLANE_H


#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

class Plane
{
public:
    Plane()
    {

    }
    ~Plane() =default;
    cv::Mat getRvec(cv::Mat rvec, cv::Mat tvec);
}

#endif
