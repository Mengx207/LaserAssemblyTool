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
using namespace cv;
using namespace std;

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
        int lineType = cv::LINE_8;
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
        cv::putText(img, "Laser Focus:", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Laser Dot Size: "+size_print, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Previous Min Dot Size: "+min_size_print, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        // cv::putText(img, "Laser Focus Status: ", cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

        cv::putText(img, "Laser Dot Location:", cv::Point(500, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Nominal Distance: "+nom_distance_print, cv::Point(500, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Distance from Center: "+center_distance_print, cv::Point(500, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
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

        if(nom_distance < 2 && center_distance < 50)
        {
            cv::circle( img, cv::Point(800,110), 20, cv::Scalar(0,255,0), -1, 8, 0 );
        }
        else
        {
            cv::circle( img, cv::Point(800,110), 20, cv::Scalar(0,0,255), -1, 8, 0 );
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

    std::pair<Mat,Mat> getRvecTvec()
    {
        // load one captured image whose content is the chessboard pattern
        // Mat image_captured = imread("images/image_captured.png", IMREAD_GRAYSCALE);
        Mat image_captured = imread("images/pattern_image.png", IMREAD_GRAYSCALE);
        Mat image_corners(image_captured.rows, image_captured.cols, IMREAD_GRAYSCALE);

        if (image_captured.empty())
        {
            cout << "Error opening image" << endl;
        }
        Size patternsize(5, 3);
        vector<Point2f> corners_found; 
        SimpleBlobDetector::Params params;
        params.maxArea = 10e4;
        Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
        bool patternfound = findChessboardCorners(image_captured, patternsize, corners_found, CALIB_CB_ASYMMETRIC_GRID);
        // cout <<endl<<endl<< "Corners found: " << endl << corners_found << endl << endl;

        drawChessboardCorners(image_corners, patternsize, Mat(corners_found), patternfound);
        
        // create chessboard pattern
        double squareSize = 4.5; // 10% square size in mm
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
        // cout << "tvec from the target board to cam:" << endl << tvec << endl;
        // cout << "rvec from the target board to cam:" << endl << rvec << endl;
        double distance = sqrt(tvec.at<double>(0) * tvec.at<double>(0) + tvec.at<double>(1) * tvec.at<double>(1) + tvec.at<double>(2) * tvec.at<double>(2));
        // cout << "distance between target board and camera: " << distance << "mm" << endl;
        // Convert rvec to rmatrix
        Mat rmatrix;
        Rodrigues(rvec, rmatrix);
        // cout << "rmatrix from the target board to cam: " << endl << rmatrix << endl << endl;
        pair<Mat,Mat>vec(rmatrix,tvec) ; // rmatrix = vec.first tvec = vec.second
        return vec;
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
        // cout << "one point on plane: " << endl
        //     << point_B << endl;
        // cout << "(normal vector) * (vector on plane):         " << N_B[0] * P_B[0] + N_B[1] * P_B[1] + N_B[2] * P_B[2] <<endl<<endl;
        pair<vector<double>,vector<double>> target_board_values(N_B,p_000);
        return target_board_values;
    }

    struct laser_plane{
        vector<double> normalvector;
        // vector<double> V_L;
        vector<double> origin;
        // vector<double> P1;
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
            p_100_L[0] - p_000_L[0],
            p_100_L[1] - p_000_L[1],
            p_100_L[2] - p_000_L[2]
        };
        // laser plane center line equation in camera frame: x=a1+b1*t, y=a2+b2*t, z=a3+b3*t
        // x = p_000_L[0] + C_L[0]*t
        // y = p_000_L[1] + C_L[1]*t
        // z = p_000_L[2] + C_L[2]*t
        //cout<<endl<<"laser center beam equation: "<<endl<<"x="<<p_000_L[0]<<"+("<<C_L[0]<<")*t"<<endl<<
        //"y="<<p_000_L[1]<<"+("<<C_L[1]<<")*t"<<endl<<"z="<<p_000_L[2]<<"+("<<C_L[2]<<")*t"<<endl<<endl;

        //Normal vector of the target board in camera frame
        vector<double> normalvector = {
            p_001_L[0] - p_000_L[0],
            p_001_L[1] - p_000_L[1],
            p_001_L[2] - p_000_L[2]
        };   
        vector<double> V_L = {
            p_110_L[0] - p_000_L[0],
            p_110_L[1] - p_000_L[1],
            p_110_L[2] - p_000_L[2]
        }; 
        // Convert vector to Mat
        Mat NormalV_L = Mat(1, 3, CV_64FC1, normalvector.data());
        Mat point_L = Mat(1, 3, CV_64FC1, p_110_L.data());
        Mat point_L_O = Mat(1, 3, CV_64FC1, p_000_L.data());

        laser_plane laser_values;
        laser_values.normalvector = normalvector;
        // laser_values.V_L = V_L;
        laser_values.origin = p_000_L;
        // laser_values.P1 = p_110_L;
        laser_values.beam_dir = beam_dir;
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
        cout<<endl<<"Target board plane equation: "<<a1<<"*(x-"<<point_B[0]<<")+"<<b1<<"*(y-"<<point_B[1]<<")+"<<c1<<"*(z-"<<point_B[2]<<") = 0"<<endl;
        cout<<"Laser plane equation: "<<a2<<"(x-"<<point_L[0]<<")+"<<b2<<"*(y-"<<point_L[1]<<")+"<<c2<<"*(z-"<<point_L[2]<<") = 0"<<endl;
        cross_P = crossProduct(N_B, N_L);
        // cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")"<<endl<<endl;
        double x0,y0,z0;
        x0 = point_L[0];
        y0 = point_L[1];
        z0 = point_L[2];
        cout<<"Intersection line of two planes__:"<<"r=("<<x0<<"+t*"<<cross_P[0]<<")*i+("<<y0<<"+t*"<<cross_P[1]<<")*j+("<<z0<<"+"<<cross_P[2]<<"*t)*k"<<endl;
        //cout<<endl<<"One point on intersection line: r0 = ("<<x<<","<<y<<",0)"<<endl;

        // cout<<"Intersection line of two planes:"<<endl<<"r=("<<x<<"+t*"<<cross_P[0]<<")*i+("<<y<<"+t*"<<cross_P[1]<<")*j+("<<cross_P[2]<<"*t)*k"<<endl;
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
        Point3f linecenter (P0[0]+C_L[0]*t, P0[1]+C_L[1]*t, P0[2]+C_L[2]*t);
        //cout<<endl<<"line center: "<< linecenter<<endl;
        return linecenter;
    }



    // std::pair<double, double> HoughAverage(cv::Mat src)
    // {
    //     std::pair<double, double> hough_avg;
    //     cv::Mat dst, cdst;
    //     if(src.empty()){
    //         std::cout << "Error opening image" << std::endl;
    //         std::exit(0);
    //     }

    //     cv::Canny(src, dst, 250, 255, 3);
    //     // cv::GaussianBlur( dst, dst, cv::Size(5, 5), 2, 2 );
    //     cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);

    //     // Standard Hough Line Transform
    //     std::vector<cv::Vec2f> lines; // will hold the results of the detection
    //     cv::HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    //     // Draw the lines
    //     float rho_avg = 0, theta_avg = 0;
    //     int counter = 0;
    //     for( size_t i = 0; i < lines.size(); i++ )
    //     {
    //         float rho = lines[i][0], theta = lines[i][1];
    //         rho_avg+=rho;
    //         theta_avg+=theta;
    //         counter++;
    //         // std::cout << rho << "\t" << theta << "\t" << std::endl;
    //         cv::Point pt1, pt2;
    //         double a = cos(theta), b = sin(theta);
    //         double x0 = a*rho, y0 = b*rho;
    //         pt1.x = cvRound(x0 + 1000*(-b));
    //         pt1.y = cvRound(y0 + 1000*(a));
    //         pt2.x = cvRound(x0 - 1000*(-b));
    //         pt2.y = cvRound(y0 - 1000*(a));
    //         line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);		
    //     }
    //     hough_avg.first = rho_avg/(float)counter;
    //     hough_avg.second = theta_avg/(float)counter;
    //     std::cout << "Rho avg: " << hough_avg.first << "\tTheta avg: " << hough_avg.second << "\t" << std::endl;
    //     // imshow("Canny Edge", dst);
    //     return hough_avg;
    // }

    // void HoughAvgOnImage(cv::Mat src, std::pair<double,double> houghAvg)
    // {
    //     if(src.empty()){
    //         std::cout << "Error opening image" << std::endl;
    //         std::exit(0);
    //     }
    //     // Draw Hough Avg Line
    //     cv::Point pt1, pt2;
    //     double a = cos(houghAvg.second), b = sin(houghAvg.second);
    //     double x0 = a*houghAvg.first, y0 = b*houghAvg.first;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line(src, pt1, pt2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
    //     // cv::waitKey(0);
    // }


    vector<RotatedRect> findRectangle(vector<vector<Point> > contours)
    {
        vector<RotatedRect> minRect;
        RotatedRect rect;
        for( int i = 0; i < contours.size(); i++ )
        { 
            /* Any contour with too small size will be regard as noise, can limit the noise level be increase the contour size threshold  */
            if(contours[i].size() > 100) 
            {
                rect = minAreaRect( Mat(contours[i]) );
                minRect.push_back(rect);
                cout<<"Rectangle center: "<<rect.center<<endl;
                cout<<"Rectangle size "<<rect.size<<endl;
                cout<<"Rectangle angle: "<<rect.angle<<endl;
                Point2f vertices[4];
                rect.points(vertices);
                cout<<"Rectangle vertices: "<<vertices[0]<<","<<vertices[1]<<","<<vertices[2]<<","<<vertices[3]<<endl;
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
        }
    }

}