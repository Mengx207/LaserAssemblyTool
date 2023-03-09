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


    /* Target board's normal vector and one point on board
    Nomal vector from board's origin (0,0,0) to (0,0,1) in board frame
    One point on the target board plane (X_board Y_board plane): (1,1,0)
    Convert them to camera frame*/
    vector<double> p_000 {
        tvec.at<double>(0), 
        tvec.at<double>(1),
        tvec.at<double>(2)
    };
    vector<double> p_001 {
        rmatrix.at<double>(2)+tvec.at<double>(0), 
        rmatrix.at<double>(5)+tvec.at<double>(1),
        rmatrix.at<double>(8)+tvec.at<double>(2)
    };
    //Normal vector of the target board in camera frame
    vector<double> N_B = {
        p_001[0] - p_000[0],
        p_001[1] - p_000[1],
        p_001[2] - p_000[2]
    };
    //One point on the target board in camera frame
    vector<double> p_110 {
        rmatrix.at<double>(0)+rmatrix.at<double>(1)+tvec.at<double>(0),
        rmatrix.at<double>(3)+rmatrix.at<double>(4)+tvec.at<double>(1),
        rmatrix.at<double>(6)+rmatrix.at<double>(7)+tvec.at<double>(2)
    };
    //Convert vector to Mat
    Mat NormalV_B = Mat(1,3,CV_64FC1,N_B.data());
    Mat point_B = Mat(1,3,CV_64FC1,p_110.data());

    cout<<"normal vector of the target board: "<<endl<<NormalV_B<<endl;
    cout<<"one point on the target board: "<<endl<<point_B<<endl<<endl;


   /* Locate laser plane*/
   vector<double> rmatrix_L_values = {
    -9.54694166520905E-12,-0.500000000000731,0.866025403784017,
    -0.173648177666924,0.852868531952858,0.492403876505388,
    -0.984807753012209,-0.150383733175655,-8.68240888417309E-02
   };
   vector<double> tvec_L_values = {
    -1.32492787807391E-10, 66.0767121304625, 2.07983966640096
   };
   Mat rmatrix_L = Mat(3, 3, CV_64FC1, rmatrix_L_values.data());
   Mat tvec_L = Mat(3,1, CV_64FC1, tvec_L_values.data());

    vector<double> p_000_L {
        tvec_L.at<double>(0), 
        tvec_L.at<double>(1),
        tvec_L.at<double>(2)
    };
    vector<double> p_001_L {
        rmatrix_L.at<double>(2)+tvec_L.at<double>(0), 
        rmatrix_L.at<double>(5)+tvec_L.at<double>(1),
        rmatrix_L.at<double>(8)+tvec_L.at<double>(2)
    };
    //Normal vector of the target board in camera frame
    vector<double> N_L = {
        p_001_L[0] - p_000_L[0],
        p_001_L[1] - p_000_L[1],
        p_001_L[2] - p_000_L[2]
    };
    //One point on the target board in camera frame
    vector<double> p_110_L {
        rmatrix_L.at<double>(0)+rmatrix_L.at<double>(1)+tvec_L.at<double>(0),
        rmatrix_L.at<double>(3)+rmatrix_L.at<double>(4)+tvec_L.at<double>(1),
        rmatrix_L.at<double>(6)+rmatrix_L.at<double>(7)+tvec_L.at<double>(2)
    };
    //Convert vector to Mat
    Mat NormalV_L = Mat(1,3,CV_64FC1,N_L.data());
    Mat point_L = Mat(1,3,CV_64FC1,p_110_L.data());

    cout<<"normal vector of the laser plane: "<<endl<<NormalV_L<<endl;
    cout<<"one point on the laser plane: "<<endl<<point_L<<endl;

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
