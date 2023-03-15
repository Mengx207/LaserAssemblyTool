/*
Input: 
camera matrix, distortion coefficient
One captured image
rmatrix, tvec from laser plane to cam
Output: 
rvec, tvec from target board to cam
target board's normal vector and origin
target board's normal vector and origin

Get rvec and tvec from the target board to camera from camera matix, distortion coefficient.
Find the target board plane's normal vector and one point on the plane in camera frame

*/

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream> 
using namespace cv;
using namespace std;
vector<Point3f> createBoardPoints(Size2i board_shape, double diagonal_spacing);
vector<Point3f> createChessBoardCorners(Size2i board_shape, double squareSize);

int main(int argc, char **argv)
{
    // Mat image_dot = imread("images/image_captured2.png", IMREAD_GRAYSCALE);
    Mat image_dot = imread("images/image_captured.png", IMREAD_GRAYSCALE);
    Mat image_dot_center(image_dot.rows, image_dot.cols, IMREAD_GRAYSCALE);
    Mat board_points(1000, 1000, IMREAD_GRAYSCALE);

    if (image_dot.empty())
    {
        cout << "Error opening image" << endl;
        return EXIT_FAILURE;
    }
    Size patternsize(5, 3); // how to define the size of asymmetric pattern?
    vector<Point2f> centers; // center of feature dots
    SimpleBlobDetector::Params params;
    params.maxArea = 10e4;
    // Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);
    Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
    // bool patternfound = findCirclesGrid(image_dot, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID, blobDetector);
    bool patternfound = findChessboardCorners(image_dot, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID);
    cout << "image points: " << endl
         << centers << endl
         << endl;
    drawChessboardCorners(image_dot_center, patternsize, Mat(centers), patternfound);

    // Size2i board_shape(5, 11);
    // double diagonal_spacing = 9;
    // vector<Point3f> boardPoints = createBoardPoints(board_shape, diagonal_spacing);
    Size2i board_shape(5, 3); // 5X3 corners
    double squareSize = 4.5; // 10% square size
    vector<Point3f> boardPoints = createChessBoardCorners(board_shape, squareSize);
    vector<Point2f> boardPoints_2D;
    for (int n = 0; n < boardPoints.size(); n++)
    {
        boardPoints_2D.push_back(Point2f(10 * boardPoints[n].x + 400, -(10 * boardPoints[n].y) + 200)); // to show the target board feature dot, reverse y axis, zoom and shift to center of image
    }
    drawChessboardCorners(board_points, board_shape, Mat(boardPoints_2D), patternfound);
    cout << "target board points: " << endl
         << boardPoints << endl
         << "size of board: " << boardPoints.size() << endl
         << endl;

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
    cout << "rvec from board to cam:" << endl
         << rvec << endl << endl;
    cout << "tvec from board to cam:" << endl
         << tvec << endl;
    double distance = sqrt(tvec.at<double>(0) * tvec.at<double>(0) + tvec.at<double>(1) * tvec.at<double>(1) + tvec.at<double>(2) * tvec.at<double>(2));
    cout << "distance between target board and camera: " << distance << "mm" << endl;
    // Convert rvec to rmatrix
    Mat rmatrix;
    Rodrigues(rvec, rmatrix);
    cout << endl << "rmatrix from board to cam: " << endl << rmatrix << endl << endl;

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
    //One point on the target board in camera frame
    vector<double> p_110 {
        rmatrix.at<double>(0)+rmatrix.at<double>(1)+tvec.at<double>(0),
        rmatrix.at<double>(3)+rmatrix.at<double>(4)+tvec.at<double>(1),
        rmatrix.at<double>(6)+rmatrix.at<double>(7)+tvec.at<double>(2)
    };
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

    cout << "The Target Board: " << endl;
    cout << "origin point: " << endl
         << point_B_O << endl;
    cout << "normal vector: " << endl
         << NormalV_B << endl;
    cout << "one point on plane: " << endl
         << point_B << endl;
    cout << "(normal vector) * (vector on plane):         " << N_B[0] * P_B[0] + N_B[1] * P_B[1] + N_B[2] * P_B[2] <<endl<<endl;

    /* Locate laser plane*/
    ifstream rmatrixL("values/rmatrix_laser.txt");
    vector<double> rmatrix_laser_values;
    while (rmatrixL >> val)
    {
        rmatrix_laser_values.push_back(val);
    }
    ifstream tvecL("values/tvec_laser.txt");
    vector<double> tvec_laser_values;
    while (tvecL >> val)
    {
        tvec_laser_values.push_back(val);
    }
    Mat rmatrix_L = Mat(3, 3, CV_64FC1, rmatrix_laser_values.data());
    Mat tvec_L = Mat(3, 1, CV_64FC1, tvec_laser_values.data());

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
    //One point on the target board in camera frame
    vector<double> p_110_L {
        rmatrix_L.at<double>(0)+rmatrix_L.at<double>(1)+tvec_L.at<double>(0),
        rmatrix_L.at<double>(3)+rmatrix_L.at<double>(4)+tvec_L.at<double>(1),
        rmatrix_L.at<double>(6)+rmatrix_L.at<double>(7)+tvec_L.at<double>(2)
    };
    //Normal vector of the target board in camera frame
    vector<double> N_L = {
        p_001_L[0] - p_000_L[0],
        p_001_L[1] - p_000_L[1],
        p_001_L[2] - p_000_L[2]
    };   
    vector<double> P_L = {
        p_110_L[0] - p_000_L[0],
        p_110_L[1] - p_000_L[1],
        p_110_L[2] - p_000_L[2]
    }; 
    // Convert vector to Mat
    Mat NormalV_L = Mat(1, 3, CV_64FC1, N_L.data());
    Mat point_L = Mat(1, 3, CV_64FC1, p_110_L.data());
    Mat point_L_O = Mat(1, 3, CV_64FC1, p_000_L.data());
    cout << "The Laser Plane: " << endl;
    cout << "origin of laser plane: " << endl
         << point_L_O << endl;
    cout << "normal vector of the laser plane: " << endl
         << NormalV_L << endl;
    cout << "one point on the laser plane: " << endl
         << point_L << endl;
    cout << "(normal vector) * (vector on plane):          " << N_L[0] * P_L[0] + N_L[1] * P_L[1] + N_L[2] * P_L[2] << endl;

    imshow("Captured Image", image_dot); // Show the result
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
    double spacing = diagonal_spacing / sqrt(2);
    vector<Point3f> centered_board_points(board_shape.area());
    for (int n = 0; n < centered_board_points.size(); ++n)
    {
        int row_n = n / board_shape.width;
        int col_n = n % board_shape.width;
        centered_board_points[n].x = (float)(row_n - (board_shape.width - 1) / 2.) * spacing;
        centered_board_points[n].y = (float)(2 * col_n + row_n % 2 + 0.5 - board_shape.height) * spacing;
        centered_board_points[n].z = 0.0;
    }
    return centered_board_points;
}

vector<Point3f> createChessBoardCorners(Size2i board_shape, double squareSize)
{
    vector<Point3f> centered_board_corners;
    int count;
    for( int i = 0; i < board_shape.height; i++ )
    {
        for( int j = 0; j < board_shape.width; j++ )
        {
            centered_board_corners.push_back(Point3f((j*squareSize), (i*squareSize), 0.0));
        }
    }
    return centered_board_corners;
}