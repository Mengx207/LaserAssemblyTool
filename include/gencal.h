#ifndef GENCAL_H
#define GENCAL_H

#include "imgpro.h"
#include "utility.h"

struct laser_beam_return{
    int min_size;
    int last_min_size;
    int size_array[10];
    double center_rect_count;
    vector<Point> center_rect_list;
    Mat dot_img;
};
struct laser_plane_return{
    Mat line_img;
    Mat threshold_output;
    struct solvePnP_result solvePnP_result;
};
struct laser_verification_return{
    Point3d actual_origin;
    Point3d ideal_origin;
    Point3d actual_normalV;
    Point3d ideal_normalV;
    Point3d delta_origin;
    Point3d delta_normalV;
};
struct arucoResult{
    Mat rmatrix;
    Mat rvec;
    Mat tvec;
    vector<Point3d> obj_corners;
    vector<Point2d> found_corners;
};

// Point3d locationCam2Target(Point2d imagePoint, solvePnP_result solvePnP_result);
Point3d locationCam2Target(Point2d imagePoint, Mat rmatrix, Mat tvec, vector<Point3d>obj_corners, vector<Point2d>found_corners);
Point3d lineEquation(Point3d p1, Point3d p2, vector<double> tvec_laser_values);
pair<Point2d, Point2d> extractLaserline2Points(Mat whiteline);
void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, uniformity_data uniformity1, Mat drawing);
arucoResult readArucoResult();


#endif // GENCAL_H