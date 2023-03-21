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
        int thickness = 2;
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
        cv::putText(img, "Last Dot Size: "+min_size_print, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        //cv::putText(img, "Laser Focus Status: ", cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

        cv::putText(img, "Laser Dot Location:", cv::Point(500, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Nominal Distance: "+nom_distance_print, cv::Point(500, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Distance from Center: "+center_distance_print, cv::Point(500, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        //cv::putText(img, "Dot Location Status: ", cv::Point(500, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

    }

    void GreenLight(cv::Mat img, int last, int current, int nom_distance, int center_distance)
    {
        if(last-current > 0 || abs(last-current) < 5)
        {
            cv::circle( img, cv::Point(300,110), 20, cv::Scalar(0,255,0), -1, 8, 0 );
        }
        else
        {
            cv::circle( img, cv::Point(300,110), 20, cv::Scalar(0,0,255), -1, 8, 0 );
        }

        if(nom_distance < 50 && center_distance < 50)
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
    vector<Point3f> createChessBoardCorners(Size2i board_shape, double squareSize)
    {
        vector<Point3f> centered_board_corners;
        int count;
        for( int i = 0; i < board_shape.height; i++ )
        {
            for( int j = 0; j < board_shape.width; j++ )
            {
                centered_board_corners.push_back(Point3f((j*squareSize)-2*squareSize, (i*squareSize)-squareSize, 0.0));
            }
        }
        return centered_board_corners;
    }

    std::pair<Mat,Mat> getRvecTvec()
    {
        Mat image_dot = imread("images/image_captured.png", IMREAD_GRAYSCALE);
        Mat image_dot_center(image_dot.rows, image_dot.cols, IMREAD_GRAYSCALE);
        Mat board_points(1000, 1000, IMREAD_GRAYSCALE);

        if (image_dot.empty())
        {
            cout << "Error opening image" << endl;
        }
        Size patternsize(5, 3); // how to define the size of asymmetric pattern?
        vector<Point2f> centers; // center of feature dots
        SimpleBlobDetector::Params params;
        params.maxArea = 10e4;
        Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
        bool patternfound = findChessboardCorners(image_dot, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID);
        // cout << "image points: " << endl
        //     << centers << endl
        //     << endl;
        drawChessboardCorners(image_dot_center, patternsize, Mat(centers), patternfound);

        Size2i board_shape(5, 3); // 5X3 corners
        double squareSize = 4.5; // 10% square size
        vector<Point3f> boardPoints = createChessBoardCorners(board_shape, squareSize);
        vector<Point2f> boardPoints_2D;
        for (int n = 0; n < boardPoints.size(); n++)
        {
            boardPoints_2D.push_back(Point2f(10 * boardPoints[n].x + 400, -(10 * boardPoints[n].y) + 200)); // to show the target board feature dot, reverse y axis, zoom and shift to center of image
        }
        drawChessboardCorners(board_points, board_shape, Mat(boardPoints_2D), patternfound);
        // cout << "target board points: " << endl
        //     << boardPoints << endl
        //     << "size of board: " << boardPoints.size() << endl
        //     << endl;

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
        solvePnP(boardPoints, centers, cameraMatrix, distCoeffs, rvec, tvec);
        //cout << "rvec from board to cam:" << endl << rvec << endl << endl;
        cout << "tvec from board to cam:" << endl
            << tvec << endl;
        double distance = sqrt(tvec.at<double>(0) * tvec.at<double>(0) + tvec.at<double>(1) * tvec.at<double>(1) + tvec.at<double>(2) * tvec.at<double>(2));
        // cout << "distance between target board and camera: " << distance << "mm" << endl;
        // Convert rvec to rmatrix
        Mat rmatrix;
        Rodrigues(rvec, rmatrix);
        cout << endl << "rmatrix from board to cam: " << endl << rmatrix << endl << endl;
        pair<Mat,Mat>vec(rmatrix,tvec) ;
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

        cout <<endl<< "The Target Board: " << endl;
        cout << "normal vector: " << endl
            << NormalV_B << endl;
        cout << "origin point: " << endl
            << point_B_O << endl;
        // cout << "one point on plane: " << endl
        //     << point_B << endl;
        // cout << "(normal vector) * (vector on plane):         " << N_B[0] * P_B[0] + N_B[1] * P_B[1] + N_B[2] * P_B[2] <<endl<<endl;
        pair<vector<double>,vector<double>> target_board_values(N_B,p_000);
        return target_board_values;
    }

    struct laser_plane{
    vector<double> N_L;
    vector<double> V_L;
    vector<double> P0;
    vector<double> C_L;
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
        vector<double> C_L = {
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
        vector<double> N_L = {
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
        Mat NormalV_L = Mat(1, 3, CV_64FC1, N_L.data());
        Mat point_L = Mat(1, 3, CV_64FC1, p_110_L.data());
        Mat point_L_O = Mat(1, 3, CV_64FC1, p_000_L.data());
        cout << "The Laser Plane: " << endl;
        cout << "normal vector: " << endl
            << NormalV_L << endl;
        cout << "origin point: " << endl
            << point_L_O << endl;
        // cout << "one point on the laser plane: " << endl
        //     << point_L << endl;
        // cout << "(normal vector) * (vector on plane):          " << N_L[0] * P_L[0] + N_L[1] * P_L[1] + N_L[2] * P_L[2] << endl;
        laser_plane laser_values;
        laser_values.N_L = N_L;
        laser_values.V_L = V_L;
        laser_values.P0 = p_000_L;
        laser_values.C_L = C_L;
        
        // pair<Mat,Mat>laser(NormalV_L, point_L_O);
        // cout<<endl<<laser.first<<endl<<laser.second<<endl;
        // pair<vector<double>,vector<double>> laser_plane_values(N_L,p_000_L);

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
        double x,y;
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
        double x,y,z;
        //cout<<endl<<"Target board plane equation: "<<a1<<"(x-"<<point_B[0]<<")+"<<b1<<"*(y-"<<point_B[1]<<")+"<<c1<<"*(z-"<<point_B[2]<<") = 0"<<endl;
        //cout<<"Laser plane equation: "<<a2<<"(x-"<<point_L[0]<<")+"<<b2<<"*(y-"<<point_L[1]<<")+"<<c2<<"*(z-"<<point_L[2]<<") = 0"<<endl;
        // cout << dotProduct(N_B, N_L) << endl;
        cross_P = crossProduct(N_B, N_L);
        //cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")";
        // To find a point on intersection line, use two plane equations and set z=0
        c1 = a1*point_B[0]+b1*point_B[0]+c1*point_B[0];
        c2 = a2*point_L[0]+b2*point_L[0]+c2*point_L[0];
        x = (c1*b2-b1*c2)/(a1*b2-b1*a2);
        y = (a1*c2-c1*a2)/(a1*b2-b1*a2);
        //cout<<endl<<"One point on intersection line: r0 = ("<<x<<","<<y<<",0)"<<endl;

        double a,b,c;
        char t;
        a = x+t*cross_P[0];
        b = y+t*cross_P[1];
        c = t*cross_P[2];
        //cout<<"Intersection line(vector equation) of two planes:"<<endl<<"r= a*i+b*j+c*k"<<endl;
        //cout<<"a="<<x<<"+t*"<<cross_P[0]<<endl;
        //cout<<"b="<<y<<"+t*"<<cross_P[1]<<endl;
        //cout<<"c=t*"<<cross_P[2]<<endl;
        cout<<endl<<"Intersection line of two planes:"<<endl<<"r=("<<x<<"+t*"<<cross_P[0]<<")*i+("<<y<<"+t*"<<cross_P[1]<<")*j+("<<cross_P[2]<<"*t)*k"<<endl;
        intersection line;
        line.x = x;
        line.a = cross_P[0];
        line.y = y;
        line.b = cross_P[1];
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

}