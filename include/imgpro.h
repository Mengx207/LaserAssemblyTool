#ifndef IMGPRO_H
#define IMGPRO_H

#include "utility.h"

using namespace GENAPI_NAMESPACE;

struct solvePnP_result{
       Mat rvec, rmatrix, tvec;
       vector<Point2f> corners_found;
       vector<Point3f> corners_created;
   };
struct laser_plane{
        vector<double> normalvector;
        vector<double> origin;
        vector<double> beam_dir;
    };
struct intersection{
        double x0,y0,z0;
        double a,b,c;
    };
struct uniformity_data{
        Mat image_BGR;
        double width_avg, width_max, width_min, width_sd;
    };

int PixelCounter(Mat img);
int SizeAverage (int count, int size_avg, int size_array[10], int center_total);
void Calculatedline(Mat img, Point start, Point end);
int NonZero(Mat img);
void HMI(Mat img, int size, int min_size, int non_zero, int nom_distance, int center_distance);
void GreenLight(Mat img, int last, int current, int nom_distance, int center_distance);
int ClearList(vector<cv::Point> center_list, int center_total, int size_array[10], int min);
pair<double,double> DotToLine(Mat img, Point start, Point end, Point center, Point interPoint);

vector<Point3f> createChessBoardCorners(Size2i patternsize, double squareSize);
solvePnP_result getRvecTvec(Mat image_captured, Size patternsize, double squareSize);
pair<vector<double>,vector<double>> targetBoardPlane(Mat &rmatrix, Mat &tvec);
laser_plane laserPlane(vector<double> rmatrix_laser_values, vector<double> tvec_laser_values);
int dotProduct(double vect_A[], double vect_B[]);
vector<double> crossProduct(vector<double> vect_A, vector<double> vect_B);
intersection intersectionLine(vector<double> N_B, vector<double> N_L, vector<double> point_B, vector<double> point_L);
Point3f intersectionPoint(vector<double>P0, vector<double>C_L, vector<double>N_B, vector<double>point_B);
vector<RotatedRect> findRectangle(vector<vector<Point> > contours, int sensitivity);
void drawContourRectangle(Mat drawing, vector<vector<Point>>contours, vector<RotatedRect> minRect);
uniformity_data cropImage(Mat image);
double findSquareWidth(Mat tiles);

#endif // IMGPRO_H