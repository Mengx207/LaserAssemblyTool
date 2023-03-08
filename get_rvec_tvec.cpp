#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
using namespace cv;
using namespace std;
vector<Point3f> createBoardPoints(Size2i board_shape, double diagonal_spacing);

int main(int argc, char ** argv)
{
    Mat image_dot = imread( "images/image_captured2.png", IMREAD_GRAYSCALE);
    // Mat gray = imread( "symmetric_dots.png", IMREAD_GRAYSCALE);
    Mat image_dot_center(image_dot.rows, image_dot.cols, IMREAD_GRAYSCALE);
    Mat board_points(1000, 1000, IMREAD_GRAYSCALE);

    if( image_dot.empty()){
        cout << "Error opening image" << endl;
        return EXIT_FAILURE;
    }
    Size patternsize(5,11); // how to define the size of asymmetric pattern?
    vector<Point2f> centers; // center of feature dots
    SimpleBlobDetector::Params params;
    params.maxArea = 10e4;
    // Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);
    Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
    bool patternfound = findCirclesGrid(image_dot,patternsize,centers, CALIB_CB_ASYMMETRIC_GRID, blobDetector);
    cout<<"image points: "<<endl<<centers<<endl<<endl;
    drawChessboardCorners(image_dot_center, patternsize, Mat(centers), patternfound);

    Size2i board_shape(5,11);
    double diagonal_spacing = 9;
    vector<Point3f>boardPoints = createBoardPoints(board_shape, diagonal_spacing);
    vector<Point2f>boardPoints_2D;
    for(int n=0; n<boardPoints.size(); n++)
    {
        boardPoints_2D.push_back(Point2f(10*boardPoints[n].x+400,-(10*boardPoints[n].y)+200)); // to show the target board feature dot, reverse y axis, zoom and shift to center of image
    }
    drawChessboardCorners(board_points, board_shape, Mat(boardPoints_2D), patternfound);
    cout<<"target board points: "<<endl<<boardPoints<<endl<<"size of board: "<<boardPoints.size()<<endl<<endl;

    vector<double> cameraMatrix_values
    { 3.4714076499814091e+03, 0., 7.5181741352412894e+02, 
        0., 3.4711767048332676e+03, 5.4514783904300646e+02, 
        0., 0., 1.  };
    vector<double> distCoeffs_values
    { -1.8430923287702131e-01, -4.2906853550556068e-02, -2.1393762247926785e-04, 2.9790668148119045e-04, 5.9981578839159733e+00};

    Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
    Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());
    // Get rvec, tvec by solvePnP()
    Mat rvec, tvec;
    solvePnP(boardPoints, centers, cameraMatrix, distCoeffs, rvec, tvec);
    cout<<"rvec:"<<endl<<rvec<<endl<<endl<<"tvec:"<<endl<<tvec<<endl;
    double distance = sqrt(tvec.at<double>(0)*tvec.at<double>(0)+tvec.at<double>(1)*tvec.at<double>(1)+tvec.at<double>(2)*tvec.at<double>(2));
    cout<<"distance between target board and camera: "<<distance<<"mm"<<endl;
    // Convert rvec to rmatrix
    Mat rmatrix;
    Rodrigues(rvec,rmatrix);
    cout<<endl<<"rmatrix: "<<endl<<rmatrix<<endl<<endl;
    // Target board's normal vector and one point on board in camera frame
    // Nomal vector from board's origin in board frame: (0,0,1)
    // One point on the board (X_b Y_b plane): (1,1,0)
    vector<double> N_B {
        rmatrix.at<double>(2)+tvec.at<double>(0), 
        rmatrix.at<double>(5)+tvec.at<double>(1),
        rmatrix.at<double>(8)+tvec.at<double>(2)
    };
    vector<double> p_B {
        rmatrix.at<double>(0)+rmatrix.at<double>(1)+tvec.at<double>(0),
        rmatrix.at<double>(3)+rmatrix.at<double>(4)+tvec.at<double>(1),
        rmatrix.at<double>(6)+rmatrix.at<double>(7)+tvec.at<double>(2)
    };
    Mat NormalV_B = Mat(1,3,CV_64FC1,N_B.data());
    Mat point_B = Mat(1,3,CV_64FC1,p_B.data());

    cout<<"normal vector of the target board: "<<endl<<NormalV_B<<endl<<endl;
    cout<<"one point on the target board: "<<endl<<point_B<<endl;

    imshow("Captured Image", image_dot);    // Show the result
    imshow("Captured Image Centers", image_dot_center);
    imshow("Board Points", board_points);
    waitKey();
    return EXIT_SUCCESS;
}

// vector<Point3f> createBoardPoints(Size2i board_shape, double diagonal_spacing)
// {
//     double spacing = diagonal_spacing/sqrt(2);
//     vector<Point3f> centered_board_points(board_shape.area());
//     for(int n=0; n<centered_board_points.size(); ++n)
//     {
//         int row_n = n/board_shape.width;
//         int col_n = n%board_shape.width;
//         centered_board_points[n].x = (float)(2*col_n+row_n%2+0.5-board_shape.width)*spacing;
//         centered_board_points[n].y = (float)(row_n - (board_shape.height - 1) / 2.) * spacing;
//         centered_board_points[n].z = 0.0;
//     } 
//     return centered_board_points;

// }

vector<Point3f> createBoardPoints(Size2i board_shape, double diagonal_spacing)
{
    double spacing = diagonal_spacing/sqrt(2);
    vector<Point3f> centered_board_points(board_shape.area());
    for(int n=0; n<centered_board_points.size(); ++n)
    {
        int row_n = n/board_shape.width;
        int col_n = n%board_shape.width;
        centered_board_points[n].x = (float)(row_n - (board_shape.width - 1) / 2.) * spacing;
        centered_board_points[n].y = (float)(2*col_n+row_n%2+0.5-board_shape.height)*spacing;
        centered_board_points[n].z = 0.0;
    } 
    return centered_board_points;

}
