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
#include <time.h>
#include <stdio.h>
#include <ctime>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <fstream> 
#include <pylon/PylonIncludes.h>
#include "ConfigurationEventPrinter.h"
using namespace cv;
using namespace std;
using namespace Pylon;

namespace laserdot
{
    //Count bright pixel
    int PixelCounter(cv::Mat img)
    {
       cv::Mat img_nominal;
       int count=0;
        img.convertTo(img_nominal, CV_32F);
        for(int i=0; i<img_nominal.rows; i++)
        {
            for(int j=0; j<img_nominal.cols; j++)
            {
                if(img_nominal.at<float>(i,j)<=0.0)
                {
                    count++;
                    //cout<<img_nominal.at<float>(i,j)<<endl;
                }
            }
        }
        return count;
    }
    //Find the average size of laser dot
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

        if(size_array[9]!=0 && center_total > 20)
        {
            size_avg = size_sum/10;
            return size_avg;
        }
    }

    //---------Draw the calculated laser line
    void CalculatedLine(cv::Mat img, cv::Point start, cv::Point end)
    {
        int thickness = 4;
        int lineType = cv::LINE_AA;
        line( img, start, end, cv::Scalar( 0, 0, 255 ), thickness, lineType );
    }

    //----------Use Canny to find the Canny edges of objects in image
    //Canny edge is a good way to count non-zero pixel
    int NonZero(cv::Mat img)
    {
        cv::Mat canny_edge, canny_edge_blur;
        cv::Canny(img, canny_edge, 100, 200, 5, false);
        cv::GaussianBlur( canny_edge, canny_edge_blur, cv::Size(5, 5), 2, 2 );
        int count = cv::countNonZero(canny_edge_blur);
        return count;
    }

    //----------Print information in window
    void HMI(cv::Mat img, int size, int min_size, int non_zero, int nom_distance, int center_distance)
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
        cv::putText(img, "Laser Beam Focus", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Laser Dot Size: "+size_print + " pixel", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Previous Min Dot Size: "+min_size_print + " pixel", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        // cv::putText(img, "Laser Focus Status: ", cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

        cv::putText(img, "Laser Beam Location", cv::Point(500, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Normal Distance: "+nom_distance_print+ " pixel", cv::Point(500, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Distance from Center: "+center_distance_print+ " pixel", cv::Point(500, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Dot Location Status: ", cv::Point(500, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

    }

    void GreenLight(cv::Mat img, int last, int current, int nom_distance, int center_distance)
    {
        // if(last-current > 0 || abs(last-current) < 5)
        // {
        //     cv::circle( img, cv::Point(300,110), 20, cv::Scalar(0,255,0), -1, 8, 0 );
        // }
        // else
        // {
        //     cv::circle( img, cv::Point(300,110), 20, cv::Scalar(0,0,255), -1, 8, 0 );
        // }

        if(nom_distance < 2 && center_distance < 600)
        {
            cv::circle( img, cv::Point(800,120), 20, cv::Scalar(0,255,0), -1, 8, 0 );
        }
        else
        {
            cv::circle( img, cv::Point(800,120), 20, cv::Scalar(0,0,255), -1, 8, 0 );
        }
    }

    int ClearList(vector<cv::Point> center_list, int center_total, int size_array[], int min)
    {
        int last_min = min;
        //cout<<"last min size: "<<last_min<<endl;
        fill_n(size_array,10,0);
        center_total = 0;
        fill(center_list.begin(), center_list.end(), cv::Point(0,0));
        return last_min;
    }

    std::pair<double,double> DotToLine(cv::Mat img, cv::Point start, cv::Point end, cv::Point center, cv::Point interPoint)
    {
        cv::LineIterator laserline(img, start, end, 8);
        vector<cv::Vec3b> buf(laserline.count);
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
        cv::line( img, center, point_list[num], cv::Scalar( 255, 255, 0 ), 1, 8 );
        // dotLine.nom_distance = min_distance;
        // dotLine.center_distance = norm(point_list[num]-point_list[(laserline.count)/2]);
        // std::pair<double,double>dist(min_distance,norm(point_list[num]-point_list[(laserline.count)/2])) ;
        std::pair<double,double>dist(min_distance,norm(point_list[num]-interPoint)) ;
        // cv::Point laserline_center = point_list[(laserline.count)/2];
        // cv::circle( img, laserline_center, 5, cv::Scalar(0,0,255), -1, 8, 0 );
        return dist;

        // cout << "min point at: " << point_list[num] <<endl;
        // cout<<"center of line: "<<laserline_center<<endl;
        // cout<<"nominal_distance: "<<nom_distance<<endl;
        // cout<<"distance from line center: "<<center_distance<<endl;
    }
}

namespace laserline
{

    vector<Point3f> createChessBoardCorners(Size2i patternsize, double squareSize)
    {
        vector<Point3f> centered_board_corners;
        int count;
        for( int i = 0; i < patternsize.height; i++ )
        {
            for( int j = 0; j < patternsize.width; j++ )
            {
                centered_board_corners.push_back(Point3f((j*squareSize)-2*squareSize, (i*squareSize)-squareSize, 0.0));
            }
        }

        return centered_board_corners;
    }

    struct solvePnP_result{
        Mat rvec, rmatrix, tvec;
        vector<Point2f> corners_found; 
        vector<Point3f> corners_created;
    };

    solvePnP_result getRvecTvec(Mat image_captured, Size patternsize, double squareSize)
    {
        // load one captured image whose content is the chessboard pattern
        // Mat image_captured = imread("images/image_captured.png", IMREAD_GRAYSCALE);
        // Mat image_captured = imread("images/pattern_image.png", IMREAD_GRAYSCALE);
        Mat image_corners(image_captured.rows, image_captured.cols, IMREAD_GRAYSCALE);

        if (image_captured.empty())
        {
            cout << "Error opening image" << endl;
        }
        // Size patternsize(7, 4);
        vector<Point2f> corners_found; 
        SimpleBlobDetector::Params params;
        params.maxArea = 10e4;
        Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
        bool patternfound = findChessboardCorners(image_captured, patternsize, corners_found, CALIB_CB_ASYMMETRIC_GRID);
        // cout <<endl<<endl<< "Corners found: " << endl << corners_found << endl << endl;

        drawChessboardCorners(image_corners, patternsize, Mat(corners_found), patternfound);
        
        // create chessboard pattern
        // double squareSize = 7;
        vector<Point3f> corners_created = createChessBoardCorners(patternsize, squareSize);
        // cout << "created pattern corners in mm: " << endl << corners_created << endl;

        // import camera matrix and distortion coefficients from txt file
        ifstream intrin("values/intrinsic.txt");
        vector<double> cameraMatrix_values;
        double val;
        while (intrin >> val)
        {
            cameraMatrix_values.push_back(val);
        }
        ifstream dist("values/distortion.txt");
        vector<double> distCoeffs_values;
        while (dist >> val)
        {
            distCoeffs_values.push_back(val);
        }
        Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
        Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());

        // Get rvec, tvec by solvePnP()
        Mat rvec, tvec;
        // corners_created:created pattern corners in mm    /corners_found: corners found on the loaded image in image coordinates system
        solvePnP(corners_created, corners_found, cameraMatrix, distCoeffs, rvec, tvec);
        // cout << "tvec from the target board origin to cam:" << endl << tvec << endl;
        // cout << "rvec from the target board origin to cam:" << endl << rvec << endl;
        double distance = sqrt(tvec.at<double>(0) * tvec.at<double>(0) + tvec.at<double>(1) * tvec.at<double>(1) + tvec.at<double>(2) * tvec.at<double>(2));

        // Convert rvec to rmatrix
        Mat rmatrix;
        Rodrigues(rvec, rmatrix);
        // cout << "rmatrix from the target board to cam:" << endl << rmatrix << endl;
        solvePnP_result result;
        result.rmatrix = rmatrix;
        result.rvec = rvec;
        result.tvec = tvec;
        result.corners_created = corners_created;
        result.corners_found = corners_found;
        return result;
        // pair<Mat,Mat>vec(rmatrix,tvec) ; // rmatrix = vec.first tvec = vec.second
        // return vec;
    }

    std::pair<vector<double>,vector<double>> targetBoardPlane(Mat rmatrix, Mat tvec)
    {
        vector<double> p_000 
        { tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2) };
        vector<double> p_001 
        { rmatrix.at<double>(2)+tvec.at<double>(0), rmatrix.at<double>(5)+tvec.at<double>(1), rmatrix.at<double>(8)+tvec.at<double>(2) };
        //One point on the target board in camera frame
        vector<double> p_110 
        { rmatrix.at<double>(0)+rmatrix.at<double>(1)+tvec.at<double>(0), rmatrix.at<double>(3)+rmatrix.at<double>(4)+tvec.at<double>(1), rmatrix.at<double>(6)+rmatrix.at<double>(7)+tvec.at<double>(2) };
        
        //Normal vector of the target board in camera frame
        vector<double> N_B = {
            p_001[0] - p_000[0],
            p_001[1] - p_000[1],
            p_001[2] - p_000[2]
        };
        //A vector on the plane
        vector<double> P_B = {
            p_110[0] - p_000[0],
            p_110[1] - p_000[1],
            p_110[2] - p_000[2]
        };

        // Convert vector to Mat
        Mat NormalV_B = Mat(1, 3, CV_64FC1, N_B.data());
        Mat point_B = Mat(1, 3, CV_64FC1, p_110.data());
        Mat point_B_O = Mat(1, 3, CV_64FC1, p_000.data());
        pair<Mat,Mat>target(NormalV_B, point_B_O);

        // cout <<endl<< "The Target Board: " << endl;
        // cout << "normal vector: " << endl
        //     << NormalV_B << endl;
        // cout << "origin point: " << endl
        //     << point_B_O << endl << endl;
        
        pair<vector<double>,vector<double>> target_board_values(N_B,p_000);
        return target_board_values;
    }

    struct laser_plane{
        vector<double> normalvector;
        vector<double> origin;
        vector<double> beam_dir;
    };
    laser_plane laserPlane(vector<double> rmatrix_laser_values, vector<double> tvec_laser_values)
    {
        Mat rmatrix_L = Mat(3, 3, CV_64FC1, rmatrix_laser_values.data());
        Mat tvec_L = Mat(3, 1, CV_64FC1, tvec_laser_values.data());

        vector<double>p_000_L = {
            tvec_L.at<double>(0),
            tvec_L.at<double>(1),
            tvec_L.at<double>(2)
        };
        vector<double> p_001_L {
            rmatrix_L.at<double>(2)+tvec_L.at<double>(0),
            rmatrix_L.at<double>(5)+tvec_L.at<double>(1),
            rmatrix_L.at<double>(8)+tvec_L.at<double>(2)
        };
        vector<double> p_010_L {
            rmatrix_L.at<double>(1)+tvec_L.at<double>(0),
            rmatrix_L.at<double>(4)+tvec_L.at<double>(1),
            rmatrix_L.at<double>(7)+tvec_L.at<double>(2)
        };
        //One point on the target board in camera frame
        vector<double> p_110_L {
            rmatrix_L.at<double>(0)+rmatrix_L.at<double>(1)+tvec_L.at<double>(0),
            rmatrix_L.at<double>(3)+rmatrix_L.at<double>(4)+tvec_L.at<double>(1),
            rmatrix_L.at<double>(6)+rmatrix_L.at<double>(7)+tvec_L.at<double>(2)
        };
        vector<double> p_100_L {
            rmatrix_L.at<double>(0)+tvec_L.at<double>(0),
            rmatrix_L.at<double>(3)+tvec_L.at<double>(1),
            rmatrix_L.at<double>(6)+tvec_L.at<double>(2)
        };
        vector<double> beam_dir = {
            p_001_L[0] - p_000_L[0],
            p_001_L[1] - p_000_L[1],
            p_001_L[2] - p_000_L[2]
        };
        // laser plane center line equation in camera frame: x=a1+b1*t, y=a2+b2*t, z=a3+b3*t
        // x = p_000_L[0] + C_L[0]*t
        // y = p_000_L[1] + C_L[1]*t
        // z = p_000_L[2] + C_L[2]*t
        //cout<<endl<<"laser center beam equation: "<<endl<<"x="<<p_000_L[0]<<"+("<<C_L[0]<<")*t"<<endl<<
        //"y="<<p_000_L[1]<<"+("<<C_L[1]<<")*t"<<endl<<"z="<<p_000_L[2]<<"+("<<C_L[2]<<")*t"<<endl<<endl;

        //Normal vector of the target board in camera frame
        // vector<double> normalvector = {
        //     p_001_L[0] - p_000_L[0],
        //     p_001_L[1] - p_000_L[1],
        //     p_001_L[2] - p_000_L[2]
        // };   
        vector<double> normalvector = {
            p_100_L[0] - p_000_L[0],
            p_100_L[1] - p_000_L[1],
            p_100_L[2] - p_000_L[2]
        };   
        // Convert vector to Mat
        Mat NormalV_L = Mat(1, 3, CV_64FC1, normalvector.data());
        Mat point_L = Mat(1, 3, CV_64FC1, p_110_L.data());
        Mat point_L_O = Mat(1, 3, CV_64FC1, p_000_L.data());

        laser_plane laser_values;
        laser_values.normalvector = normalvector;
        laser_values.origin = p_000_L;
        // laser_values.P1 = p_110_L;
        laser_values.beam_dir = beam_dir;
        // cout<<endl<<"laser normal vector: "<< normalvector[0] << "," << normalvector[1] <<"," << normalvector[2] << endl;
        // cout<<"laser origin: "<< p_000_L[0] << "," << p_000_L[1] <<"," << p_000_L[2] << endl;
        // cout<<"laser direction: "<< beam_dir[0] << "," << beam_dir[1] <<"," << beam_dir[2] << endl;
        return laser_values;
    }

    int dotProduct(double vect_A[], double vect_B[])
    {
        double product = 0;
        for (int i = 0; i < 3; i++)
        {
            product = product + vect_A[i] * vect_B[i];
        }
        return product;
    }
 
    // cross product of two vector array.
    vector<double> crossProduct(vector<double> vect_A, vector<double> vect_B)
    
    {
        vector<double> cross_P;
        cross_P.push_back(vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]);
        cross_P.push_back(vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]);
        cross_P.push_back(vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]);
        return cross_P;
    }

    
    struct intersection{
        double x0,y0,z0;
        double a,b,c;
    };
    intersection intersectionLine(vector<double> N_B, vector<double> N_L, vector<double> point_B, vector<double> point_L)
    {
        double a1,b1,c1,a2,b2,c2;
        a1 = N_B[0];
        b1 = N_B[1];
        c1 = N_B[2];
        a2 = N_L[0];
        b2 = N_L[1];
        c2 = N_L[2];	
        vector<double> cross_P;
        //find the plane equations
        // cout<<endl<<"Target board plane equation: "<<a1<<"*(x-"<<point_B[0]<<")+"<<b1<<"*(y-"<<point_B[1]<<")+"<<c1<<"*(z-"<<point_B[2]<<") = 0"<<endl;
        // cout<<"Laser plane equation: "<<a2<<"(x-"<<point_L[0]<<")+"<<b2<<"*(y-"<<point_L[1]<<")+"<<c2<<"*(z-"<<point_L[2]<<") = 0"<<endl;
        cross_P = crossProduct(N_B, N_L);
        // cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")"<<endl<<endl;
        double x0,y0,z0;
        x0 = point_L[0];
        y0 = point_L[1];
        z0 = point_L[2];
        // cout<<"Intersection line of two planes:"<<"r=("<<x0<<"+t*"<<cross_P[0]<<")*i+("<<y0<<"+t*"<<cross_P[1]<<")*j+("<<z0<<"+"<<cross_P[2]<<"*t)*k"<<endl;
        // cout<<endl<<"One point on intersection line: r0 = ("<<x0<<","<<y0<<","<<z0<<")"<<endl;

        intersection line;
        line.x0 = x0;
        line.a = cross_P[0];
        line.y0 = y0;
        line.b = cross_P[1];
        line.z0 = z0;
        line.c = cross_P[2];
        return line;
    }
    
    Point3f intersectionPoint(vector<double>P0, vector<double>C_L, vector<double>N_B, vector<double>point_B)
    {
        // target plane equation: N_B[0]*(x-point_B[0])+N_B[1]*(y-point_B[1])+N_B[2]*(z-point_B[2]) = 0;
        // laser beam center line equation:
        // x = P0[0] + C_L[0]*t
        // y = P0[1] + C_L[1]*t
        // z = P0[2] + C_L[2]*t

        // N_B[0]*((P0[0]+C_L[0]*t)-point_B[0]) + N_B[1]*((P0[1]+C_L[1]*t)-point_B[1]) + N_B[2]*((P0[2]+C_L[2]*t)-point_B[2]) = 0;
        // N_B[0]*P0[0] + N_B[0]*C_L[0]*t - N_B[0]*point_B[0] + N_B[1]*P0[1] + N_B[1]*C_L[1]*t - N_B[1]*point_B[1] + N_B[2]*P0[2] + N_B[2]*C_L[2]*t - N_B[2]*point_B[2] = 0;
        // t*(N_B[0]*C_L[0]+N_B[1]*C_L[1]+N_B[2]*C_L[2]) = N_B[0]*point_B[0] + N_B[1]*point_B[1] + N_B[2]*point_B[2] - N_B[0]*P0[0] - N_B[1]*P0[1] - N_B[2]*P0[2];
        double t = (N_B[0]*point_B[0] + N_B[1]*point_B[1] + N_B[2]*point_B[2] - N_B[0]*P0[0] - N_B[1]*P0[1] - N_B[2]*P0[2]) / (N_B[0]*C_L[0]+N_B[1]*C_L[1]+N_B[2]*C_L[2]);
        //cout<<endl<<"t = "<<t<<endl;
        // Point2d interPoint (P0[0]+C_L[0]*t, P0[1]+C_L[1]*t);
        Point3f interPoint (P0[0]+C_L[0]*t, P0[1]+C_L[1]*t, P0[2]+C_L[2]*t);
        // cout<<endl<<"Intersection point between laser beam and target board: "<< interPoint<<endl;
        return interPoint;
    }


    vector<RotatedRect> findRectangle(vector<vector<Point> > contours, int sensitivity)
    {
        vector<RotatedRect> minRect;
        RotatedRect rect;
        for( int i = 0; i < contours.size(); i++ )
        { 
            /* Any contour with too small size will be regard as noise, can limit the noise level be increase the contour size threshold  */
            if(contours[i].size() > sensitivity) 
            {
                rect = minAreaRect( Mat(contours[i]) );
                minRect.push_back(rect);
                // cout<<"Rectangle center: "<<rect.center<<endl;
                // cout<<"Rectangle size "<<rect.size<<endl;
                // cout<<"Rectangle angle: "<<rect.angle<<endl;
                Point2f vertices[4];
                rect.points(vertices);
                // cout<<"Rectangle vertices: "<<vertices[0]<<","<<vertices[1]<<","<<vertices[2]<<","<<vertices[3]<<endl;
            }
        }
        return minRect;
    }

    void drawContourRectangle(Mat drawing, vector<vector<Point>>contours, vector<RotatedRect> minRect)
    {
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( 0, 0, 255 );
            // contour
            cv::drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        }

        for( int i = 0; i < minRect.size(); i++)
        {
            Scalar color = Scalar( 255, 255, 0 );
            // rotated rectangle
            Point2f rect_points[4]; 
            minRect[i].points( rect_points );

            for( int j = 0; j < 4; j++ )
            {
                line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
            }
            circle (drawing, minRect[i].center, 2, cv::Scalar(0,255,0), -1, 8, 0);
        }
    }

    struct uniformity_data{
        Mat image_BGR;
        double width_avg, width_max, width_min, width_sd;
    };

    double findSquareWidth(Mat tiles)
    {
        vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
        findContours( tiles, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        vector<RotatedRect> minRect = laserline::findRectangle(contours,50);
        if(minRect.size()>0)
        {
            if (minRect[0].size.height > minRect[0].size.width)
            {
                //cout<<"Width of rectangle: "<<minRect[0].size.width<<endl;
                return minRect[0].size.width;
            }
            else
            {
                //cout<<"Width of rectangle: "<<minRect[0].size.height<<endl;
                return minRect[0].size.height;
            }
        }
        else
        {return 0;}
    }

    uniformity_data cropImage(Mat image)
    {
        Mat image_BGR;
        cvtColor(image,image_BGR,COLOR_GRAY2BGR);

        Mat image_copy = image.clone();
        int imgheight = image_copy.rows;
        int imgwidth = image_copy.cols;
        int M = 215;
        int N = 287;
        int x1 = 0;
        int y1 = 0;
        vector<double> rectWidth;
        for (int y = 0; y<imgheight+1; y=y+M)
        {
            for (int x = 0; x<imgwidth+1; x=x+N)
            {
                if ((imgheight - y) < M || (imgwidth - x) < N)
                {
                    break;
                }
                y1 = y + M;
                x1 = x + N;
                string a = to_string(x);
                string b = to_string(y);
        
                    // crop the patches of size MxN
                    Mat tiles = image_copy(Range(y, y+M), Range(x, x+N));
                    rectWidth.push_back(findSquareWidth(tiles));
                    system("cd images && mkdir -p saved_patches");
                    //save each patches into file directory
                    imwrite("images/saved_patches/tile" + a + '_' + b + ".jpg", tiles);  
                    rectangle(image_BGR, Point(x,y), Point(x1,y1), Scalar(0,255,0), 1);  
            }
        }

        uniformity_data uniformity1;
        uniformity1.image_BGR = image_BGR;
        double total = 0;
        double count = 0;
        double max = 0;
        double min = 1000;
        vector<double> nonEmptyRect;

        for (int i=0; i<rectWidth.size(); i++)
        {
            if(rectWidth[i] > 0)
            {
                nonEmptyRect.push_back(rectWidth[i]);
                total = total + rectWidth[i];
                count++;
                if (rectWidth[i]>max)
                {
                    max = rectWidth[i];
                }
                if (rectWidth[i]<min)
                {
                    min = rectWidth[i];
                }
            }
        }
        uniformity1.width_max = max;
        uniformity1.width_min = min;
        uniformity1.width_avg = total/count;
        
        float tot = 0;
        for (int j=0; j < nonEmptyRect.size(); j++)
        {
            tot = tot + (nonEmptyRect[j] - uniformity1.width_avg)*(nonEmptyRect[j] - uniformity1.width_avg);
        }
        uniformity1.width_sd = sqrt(tot/nonEmptyRect.size());
        return uniformity1;
       
    }
}

namespace general
{

    Point3d locationCam2Target(Point2d imagePoint, laserline::solvePnP_result solvePnP_result)
    {
        Point3f corner_created_max_x = solvePnP_result.corners_created[0];
        Point3f corner_created_max_y = solvePnP_result.corners_created[0];
        Point2f corner_found_max_x = solvePnP_result.corners_found[0];
        Point2f corner_found_max_y = solvePnP_result.corners_found[0];

        Point3f corner_created_min_x = solvePnP_result.corners_created[0];
        Point3f corner_created_min_y = solvePnP_result.corners_created[0];
        Point2f corner_found_min_x = solvePnP_result.corners_found[0];
        Point2f corner_found_min_y = solvePnP_result.corners_found[0];

        for(int i = 1; i < solvePnP_result.corners_created.size(); i++)
        {
            if(solvePnP_result.corners_created[i].x > corner_created_max_x.x)
            {
                corner_created_max_x = solvePnP_result.corners_created[i];
            }

            if(solvePnP_result.corners_created[i].y > corner_created_max_y.y)
            {
                corner_created_max_y = solvePnP_result.corners_created[i];
            }

            if(solvePnP_result.corners_found[i].x > corner_found_max_x.x)
            {
                corner_found_max_x = solvePnP_result.corners_found[i];
            }

            if(solvePnP_result.corners_found[i].y > corner_found_max_y.y)
            {
                corner_found_max_y = solvePnP_result.corners_found[i];
            }
            
            if(solvePnP_result.corners_created[i].x < corner_created_min_x.x)
            {
                corner_created_min_x = solvePnP_result.corners_created[i];
            }
            if(solvePnP_result.corners_created[i].y < corner_created_min_y.y)
            {
                corner_created_min_y = solvePnP_result.corners_created[i];
            }
            if(solvePnP_result.corners_found[i].x < corner_found_min_x.x)
            {
                corner_found_min_x = solvePnP_result.corners_found[i];
            }
            if(solvePnP_result.corners_found[i].y < corner_found_min_y.y)
            {
                corner_found_min_y = solvePnP_result.corners_found[i];
            }
        }

        float xRange_created = corner_created_max_x.x - corner_created_min_x.x;
        float yRange_created = corner_created_max_y.y - corner_created_min_y.y;
        float xRange_found = corner_found_max_x.x - corner_found_min_x.x;
        float yRange_found = corner_found_max_y.y - corner_found_min_y.y;
        // cout<< endl<<"magnifier calculation: "<<endl<<xRange_created<< endl << yRange_created<< endl<< xRange_found<<endl<<yRange_found<<endl;
        float magnifier = (xRange_found + yRange_found)/(xRange_created+yRange_created);
        // cout<<endl<<"magnifier: "<< magnifier<<endl;

        Point oneCorner = imagePoint;
        cout<<endl<<"Dot coordinates on image plane (imageframe): "<<endl<<oneCorner<<endl;

        float corners_found_mid_x = (corner_found_max_x.x + corner_found_min_x.x)/2;
        float corners_found_mid_y = (corner_found_max_y.y + corner_found_min_y.y)/2;
        // cout<<endl<<"center point of pattern: "<<corners_found_mid_x<<" "<<corners_found_mid_y<<endl;

        Point3d cornerTargetFrame = Point3d((oneCorner.x-corners_found_mid_x) / magnifier, (oneCorner.y-corners_found_mid_y) / magnifier, 0);
        cout <<"Dot coordinates on target board (targetframe): "<<endl<<cornerTargetFrame <<endl;
        
        Mat transMatrix; // translation matrix from target board frame to image frame
        hconcat(solvePnP_result.rmatrix, solvePnP_result.tvec, transMatrix);
        Mat arr = Mat::zeros(1,4,CV_64F);
        arr.at<double>(3) = 1;
        vconcat(transMatrix, arr, transMatrix);
        // cout<<endl<<"transform matrix: "<<endl<<transMatrix<<endl;
        
        Mat cornerTF = Mat(4,1,CV_64F); //Translate Point3d into 4x1 matrix
        cornerTF.at<double>(0,0) = cornerTargetFrame.x;
        cornerTF.at<double>(1,0) = cornerTargetFrame.y;
        cornerTF.at<double>(2,0) = cornerTargetFrame.z;
        cornerTF.at<double>(3,0) = 1;
        Mat cornerCamFrame = transMatrix*cornerTF; //Same dot expressed in Image Frame
        cout<<"Dot coordinates on target board (cam frame)"<<endl<<cornerCamFrame<<endl;
        Point3d pointCamFrame;
        pointCamFrame.x = cornerCamFrame.at<double>(0);
        pointCamFrame.y = cornerCamFrame.at<double>(1);
        pointCamFrame.z = cornerCamFrame.at<double>(2);
        return pointCamFrame;

    }

    void lineEquation(Point3d p1, Point3d p2, vector<double> tvec_laser_values)
    {
        double l = p2.x - p1.x;
        double m = p2.y - p1.y;
        double n = p2.z - p1.z;
        // Cartesian form:
        // (x-p2.x)/l = (y-p2.y)/m = (z-p2.z)/n
        // Vector form:
        // (p2.x*i + p2.y*j + p2.z*k) + t*(l*i + m*j + n*k) = (p2.x + t*l)*i + (p2.y + t*m)*j + (p2.z + t*n)*k
        cout<< "Vector equation from two 3D points: (" << l<<"*t + "<<p2.x<<", "<<m<<"*t + "<<p2.y<<", "<<n<<"*t + "<<p2.z<<")"<<endl;
        Mat tvec_L = Mat(3, 1, CV_64FC1, tvec_laser_values.data());
        double t = (tvec_L.at<double>(2) - p2.z)/n;
        cout<<"Retrace along the line back to the laser origin: (" << p2.x + t*l<<","<<p2.y + t*m<<","<<p2.z + t*n<<")"<<endl<<endl;


    }


}