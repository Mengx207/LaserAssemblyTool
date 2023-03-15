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

using namespace std;

//using namespace cv;


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
        cv::putText(img, "Laser Focus Status: ", cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

        cv::putText(img, "Laser Dot Location:", cv::Point(500, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Nominal Distance: "+nom_distance_print, cv::Point(500, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Distance from Center: "+center_distance_print, cv::Point(500, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
        cv::putText(img, "Dot Location Status: ", cv::Point(500, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);

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
        cout<<"last min size: "<<last_min<<endl;
        fill_n(size_array,10,0);
        center_total = 0;
        fill(center_list.begin(), center_list.end(), cv::Point(0,0));
        return last_min;
    }

    std::pair<double,double> DotToLine(cv::Mat img, cv::Point start, cv::Point end, cv::Point center)
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
        std::pair<double,double>dist(min_distance,norm(point_list[num]-point_list[(laserline.count)/2])) ;
        cv::Point laserline_center = point_list[(laserline.count)/2];
        cv::circle( img, laserline_center, 5, cv::Scalar(0,0,255), -1, 8, 0 );
        return dist;

        // cout << "min point at: " << point_list[num] <<endl;
        // cout<<"center of line: "<<laserline_center<<endl;
        // cout<<"nominal_distance: "<<nom_distance<<endl;
        // cout<<"distance from line center: "<<center_distance<<endl;
    }
}

namespace intersection
{
    int dotProduct(int vect_A[], int vect_B[])
    {
        int product = 0;
        for (int i = 0; i < 3; i++)
        {
            product = product + vect_A[i] * vect_B[i];
        }
        return product;
    }
 
    // cross product of two vector array.
    void crossProduct(int vect_A[], int vect_B[], int cross_P[])
    
    {
        cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
        cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }

    void intersectionLine(int N1[3], int N2[3], int point1[3], int point2[3])
    {
    //     //Known normal vectors for two planes
    //     N1[3] = { 10,8,3 }; 
    //     N2[3] = { 2,6,5 };
    //     //Known one point on each plane
    //     point1[3] = { 10,5,5 };
    //     point2[3] = { 2,2,2 };

        double a1,b1,c1,a2,b2,c2;
        a1 = N1[0];
        b1 = N1[1];
        c1 = N1[2];
        a2 = N2[0];
        b2 = N2[1];
        c2 = N2[2];	
        int cross_P[3];
        //find the plane equations
        double x,y,z;
        //a1*(x-point1[0])+b1*(y-point1[0])+c1*(z-point1[0]) = 0;
        //a2*(x-point2[0])+b2*(y-point2[0])+c2*(z-point2[0]) = 0;
        //a1*x+b1*y+c1*z = (a1*point1[0]+b1*point1[0]+c1*point1[0]);
        //a2*x+b2*y+c2*z = (a2*point2[0]+b2*point2[0]+c2*point2[0]);	
        cout<<"Plane1 equation: "<<a1<<"(x-"<<point1[0]<<")+"<<b1<<"*(y-"<<point1[1]<<")+"<<c1<<"*(z-"<<point1[2]<<") = 0"<<endl;
        cout<<"Plane2 equation: "<<a2<<"(x-"<<point2[0]<<")+"<<b2<<"*(y-"<<point2[1]<<")+"<<c2<<"*(z-"<<point2[2]<<") = 0"<<endl;
        // dotProduct function call
        // cout << "Dot product:";
        // cout << dotProduct(N1, N2) << endl;
        // crossProduct function call
        crossProduct(N1, N2, cross_P);
        cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")";
        // To find a point on intersection line, use two plane equations and set z=0
        // a1*x+b1*y = (a1*point1[0]+b1*point1[0]+c1*point1[0]);
        // a2*x+b2*y = (a2*point2[0]+b2*point2[0]+c2*point2[0]);
        c1 = a1*point1[0]+b1*point1[0]+c1*point1[0];
        c2 = a2*point2[0]+b2*point2[0]+c2*point2[0];
        x = (c1*b2-b1*c2)/(a1*b2-b1*a2);
        y = (a1*c2-c1*a2)/(a1*b2-b1*a2);
        cout<<endl<<"One point on intersection line: r0 = ("<<x<<","<<y<<",0)"<<endl;
        
        // Plug v and r0 into vector equation
        // r = (x*i + y*j + 0*k) + t*(cross_P[0]*i+cross_P[1]*j+cross_P[2]*k)
        // r = (x+t*cross_P[0])*i + (y+t*cross_P[1])*j + (t*cross_P[2])*k
        double a,b,c;
        char t;
        a = x+t*cross_P[0];
        b = y+t*cross_P[1];
        c = t*cross_P[2];
        cout<<"Intersection line(vector equation) of two planes:"<<endl<<"r= a*i+b*j+c*k"<<endl;
        cout<<"a="<<x<<"+t*"<<cross_P[0]<<endl;
        cout<<"b="<<y<<"+t*"<<cross_P[1]<<endl;
        cout<<"c=t*"<<cross_P[2]<<endl;
        cout<<"r=("<<x<<"+t*"<<cross_P[0]<<")*i+("<<y<<"+t*"<<cross_P[1]<<")*j+("<<cross_P[2]<<")*k"<<endl;

    }
}