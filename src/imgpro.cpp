#include "imgpro.h"

int PixelCounter(Mat img)
{
  Mat img_nominal;
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

void CalculatedLine(cv::Mat img, cv::Point start, cv::Point end)
{
    int thickness = 4;
    int lineType = cv::LINE_AA;
    line( img, start, end, cv::Scalar( 0, 0, 255 ), thickness, lineType );
}

int NonZero(cv::Mat img)
    {
        cv::Mat canny_edge, canny_edge_blur;
        cv::Canny(img, canny_edge, 100, 200, 5, false);
        cv::GaussianBlur( canny_edge, canny_edge_blur, cv::Size(5, 5), 2, 2 );
        int count = cv::countNonZero(canny_edge_blur);
        return count;
    }
void HMI(cv::Mat img, int size, int min_size, int non_zero, int nom_distance, int center_distance)
    {
        string size_print = "No value";
        string min_size_print = "No value";
        string nom_distance_print = "No value";
        string center_distance_print = "No value";
        if(non_zero >20)
        {
            size_print = std::to_string(size);
            min_size_print = std::to_string(min_size);
            nom_distance_print = std::to_string(nom_distance);
            center_distance_print = std::to_string(center_distance);
        }
        string a = "Laser Beam Focus";
        putText(img, a, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,184,0),2);
        putText(img, "Dot Size: "+size_print + " pixel", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(205,134,0),2);
        putText(img, "Captured Min Size: "+min_size_print + " pixel", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(205,134,0),2);
        putText(img, "Laser Beam Location", cv::Point(500, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(110,254,255),2);
        putText(img, "Perpendicular Distance: "+nom_distance_print+ " pixel", cv::Point(500, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(60,204,205),2);
        putText(img, "Line Segment Length: "+center_distance_print+ " pixel", cv::Point(500, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(60,204,205),2);
    }

void GreenLight(cv::Mat img, int last, int current, int nom_distance, int center_distance)
{
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
dot_distance_return DotToLine(cv::Mat img, cv::Point start, cv::Point end, cv::Point center, cv::Point interPoint)
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
       dot_distance_return distance_result;
       distance_result.distance = min_distance;
       distance_result.segment_length = norm(point_list[num]-interPoint);
       distance_result.point = point_list[num];
       return distance_result;
    //    std::pair<double,double>dist(min_distance,norm(point_list[num]-interPoint)) ;
    //    return dist;
   }
vector<Point3f> createChessBoardCorners(Size2i patternsize, double squareSize)
   {
       vector<Point3f> centered_board_corners;
       for( int i = 0; i < patternsize.height; i++ )
       {
           for( int j = 0; j < patternsize.width; j++ )
           {
               centered_board_corners.push_back(Point3f((j*squareSize)-2*squareSize, squareSize-squareSize*i, 0.0));
           }
       }

       return centered_board_corners;
   }
solvePnP_result getRvecTvec(Mat image_captured, Size patternsize, double squareSize)
{
    Mat image_corners(image_captured.rows, image_captured.cols, IMREAD_GRAYSCALE);
    if (image_captured.empty())
    {
        cout << "Error opening image" << endl;
    }
    // Size patternsize(7, 4);
    vector<Point2f> corners_found;
    vector<Point2f> corners_found_refined;
    SimpleBlobDetector::Params params;
    params.maxArea = 10e4;
    Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
    bool patternfound = findChessboardCorners(image_captured, patternsize, corners_found, CALIB_CB_ASYMMETRIC_GRID);
    // cout <<endl<<endl<< "Corners found: " << endl << corners_found << endl;
    if(patternfound)
    {
        Size tWinSize  = Size ( 50, 50 );
        Size tZeroZone = Size ( -1, -1 );
        TermCriteria  tCriteria = TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
        cornerSubPix (image_captured, corners_found, tWinSize, tZeroZone, tCriteria );
        // cout <<endl<<endl<< "Corners found refined: " << endl << corners_found << endl;
    }

    drawChessboardCorners(image_corners, patternsize, Mat(corners_found), patternfound);

    // create chessboard pattern
    // double squareSize = 7;
    vector<Point3f> corners_created = createChessBoardCorners(patternsize, squareSize);
//    cout << "created pattern corners in mm: " << endl << corners_created << endl;

    // import camera matrix and distortion coefficients from txt file
    ifstream intrin("/home/lingbo/AG_GUI//values/intrinsic.txt");
    vector<double> cameraMatrix_values;
    double val;
    while (intrin >> val)
    {
        cameraMatrix_values.push_back(val);
    }
    ifstream dist("/home/lingbo/AG_GUI/values/distortion.txt");
    vector<double> distCoeffs_values;
    while (dist >> val)
    {
        distCoeffs_values.push_back(val);
    }
    Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
    Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());
    Mat rvec, tvec;
    solvePnP(corners_created, corners_found, cameraMatrix, distCoeffs, rvec, tvec);
    // cout<<endl<<"rvec: "<<rvec<<endl;
    // cout<<"tvec: "<<tvec<<endl;
    Mat rmatrix;
    Rodrigues(rvec, rmatrix);
    solvePnP_result result;
    result.rmatrix = rmatrix;
    result.rvec = rvec;
    result.tvec = tvec;
    result.corners_created = corners_created;
    result.corners_found = corners_found;
    return result;
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
        cross_P = crossProduct(N_B, N_L);
        // cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")"<<endl<<endl;
        double x0,y0,z0;
        x0 = point_L[0];
        y0 = point_L[1];
        z0 = point_L[2];

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
    double t = (N_B[0]*point_B[0] + N_B[1]*point_B[1] + N_B[2]*point_B[2] - N_B[0]*P0[0] - N_B[1]*P0[1] - N_B[2]*P0[2]) / (N_B[0]*C_L[0]+N_B[1]*C_L[1]+N_B[2]*C_L[2]);
    Point3f interPoint (P0[0]+C_L[0]*t, P0[1]+C_L[1]*t, P0[2]+C_L[2]*t);
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

double findSquareWidth(Mat tiles)
   {
       vector<vector<Point> > contours;
       vector<Vec4i> hierarchy;
       findContours( tiles, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
       vector<RotatedRect> minRect = findRectangle(contours,50);
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
               system("cd /home/lingbo/AG_GUI/images && mkdir -p saved_patches");
               //save each patches into file directory
               imwrite("/home/lingbo/AG_GUI/images/saved_patches/tile" + a + '_' + b + ".jpg", tiles);
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