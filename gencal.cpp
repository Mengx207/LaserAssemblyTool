#include "gencal.h"

Point3d locationCam2Target(Point2d imagePoint, solvePnP_result solvePnP_result)
{
    Point3f corner_created_max_x = solvePnP_result.corners_created[0];
    Point3f corner_created_max_y = solvePnP_result.corners_created[0];
    Point2f corner_found_max_x = solvePnP_result.corners_found[0];
    Point2f corner_found_max_y = solvePnP_result.corners_found[0];

    Point3f corner_created_min_x = solvePnP_result.corners_created[0];
    Point3f corner_created_min_y = solvePnP_result.corners_created[0];
    Point2f corner_found_min_x = solvePnP_result.corners_found[0];
    Point2f corner_found_min_y = solvePnP_result.corners_found[0];

    for(uint i = 1; i < solvePnP_result.corners_created.size(); i++)
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
    float magnifier = (xRange_found + yRange_found)/(xRange_created+yRange_created); //pattern is square
    // cout<<endl<<"magnifier: "<< magnifier<<endl;

    Point oneCorner = imagePoint;
    // cout<<endl<<"Dot coordinates on image plane (imageframe) in pixel: "<<endl<<oneCorner<<endl;

    // float corners_found_center_x = (corner_found_max_x.x + corner_found_min_x.x)/2;
    // float corners_found_center_y = (corner_found_max_y.y + corner_found_min_y.y)/2;
    float corners_found_center_x = solvePnP_result.corners_found[9].x;
    float corners_found_center_y = solvePnP_result.corners_found[9].y;
    // cout<<endl<<"center point of pattern (pixel) on image plane: "<<corners_found_center_x<<","<<corners_found_center_y<<endl;
    // Origin of the target frame is the center of the pattern, /magnifier transfer pixel to mm
    // Point3d cornerTargetFrame = Point3d((oneCorner.x-corners_found_center_x) / magnifier, (oneCorner.y-corners_found_center_y) / magnifier, 0);
    Point3d cornerTargetFrame = Point3d(-(oneCorner.y-corners_found_center_y)/magnifier,-(oneCorner.x-corners_found_center_x)/magnifier, 0);
    // cout <<"Dot coordinates on target board (targetframe) in mm: "<<endl<<cornerTargetFrame <<endl;

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
    Mat cornerCamFrame = transMatrix*cornerTF; //Same dot expressed in Cam Frame
    // cout<<"Dot coordinates on target board (cam frame)"<<endl<<cornerCamFrame<<endl;
    Point3d pointCamFrame;
    pointCamFrame.x = cornerCamFrame.at<double>(0);
    pointCamFrame.y = cornerCamFrame.at<double>(1);
    pointCamFrame.z = cornerCamFrame.at<double>(2);
    return pointCamFrame;

}

Point3d lineEquation(Point3d p1, Point3d p2, vector<double> tvec_laser_values)
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
    cout<<endl<<"Retrace along the line back to the laser origin: (" << p2.x + t*l<<","<<p2.y + t*m<<","<<p2.z + t*n<<")"<<endl;
    Point3d actual_origin = Point3d(p2.x + t*l,p2.y + t*m,p2.z + t*n);
    return actual_origin;
}

pair<Point2d, Point2d> extractLaserline2Points(Mat whiteline)
{
    Mat whiteline_blur, whiteline_color;
    cvtColor(whiteline, whiteline_color, COLOR_GRAY2BGR);
    blur(whiteline, whiteline_blur, Size(5,5));
    Canny(whiteline_blur, whiteline_blur, 200, 255, 5);

    vector<Vec2f> lines;
    HoughLines(whiteline_blur, lines, 1, CV_PI/180, 60, 0, 0 );
    Mat findline = Mat(whiteline.size().height, whiteline.size().width, CV_8UC3);

    float start_x_total = 0; float start_y_total = 0; float start_x, start_y;
    float end_x_total = 0; float end_y_total = 0; float end_x, end_y;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        start_x_total = start_x_total + pt1.x;
        start_y_total = start_y_total + pt1.y;
        end_x_total = end_x_total + pt2.x;
        end_y_total = end_y_total + pt2.y;
        // cout<<"ends of line: "<<pt1<<", "<<pt2<<endl;
        // line( findline, pt1, pt2, Scalar(0,0,255), 1, LINE_AA);
        // line( whiteline_color, pt1, pt2, Scalar(150,100,0), 1, LINE_AA);
        // circle( whiteline_color, pt1, 5, cv::Scalar(0,0,255), -1, 8, 0 );
        // circle( whiteline_color, pt2, 5, cv::Scalar(0,0,255), -1, 8, 0 );
    }
    start_x = start_x_total/lines.size();
    start_y = start_y_total/lines.size();
    end_x = end_x_total/lines.size();
    end_y = end_y_total/lines.size();
    Point2d start, end;
    start.x = cvRound(start_x);
    start.y = cvRound(start_y);
    end.x = cvRound(end_x);
    end.y = cvRound(end_y);
    // cout<<"start and end points of the line: ("<<start_x<<", "<<start_y<<") ("<<end_x<<", "<<end_y<<")"<<endl;
    pair<Point2d, Point2d> laserline2Points;
    laserline2Points.first = start;
    laserline2Points.second = end;
    cout<<"cvRounded start and end points of the line on image in image frame: "<<start<<", "<<end<<endl;
    line( whiteline_color, start, end, Scalar(0,0,255), 1, LINE_AA);
    circle(whiteline_color, start, 2, Scalar(0,0,255),3);
    circle(whiteline_color, end, 2, Scalar(0,0,255),3);
    return laserline2Points;
}

void laserlineGUI(RotatedRect rect, Point2d cal_center, int cal_angle, uniformity_data uniformity1, Mat drawing)
{
    std::string center_print_x, center_print_y, angle_print, width_print, cal_center_print_x, cal_center_print_y, cal_angle_print, width_avg_print, width_max_print, width_min_print, width_sd_print;
    center_print_x = std::to_string(int(rect.center.x));
    center_print_y = std::to_string(int(rect.center.y));
    angle_print = std::to_string(int(rect.angle));

    std::ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(2);

    streamObj << uniformity1.width_max;
    width_max_print = streamObj.str();
    streamObj.str("");
    streamObj << uniformity1.width_min;
    width_min_print = streamObj.str();
    streamObj.str("");
    streamObj << uniformity1.width_sd;
    width_sd_print = streamObj.str();
    streamObj.str("");
    streamObj << uniformity1.width_avg;
    width_avg_print = streamObj.str();
    streamObj.str("");

    if(rect.size.width < rect.size.height)
    {width_print = std::to_string(int(rect.size.width));}
    else
    {width_print = std::to_string(int(rect.size.height));}

    cal_center_print_x = std::to_string(int(cal_center.x));
    cal_center_print_y = std::to_string(int(cal_center.y));
    cal_angle_print = std::to_string(cal_angle);

    cv::putText(drawing, "Angle Designed: " + cal_angle_print, cv::Point(1000,600), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Angle Actual: " + angle_print, cv::Point(1000,630), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Width Average: " + width_avg_print + " mm", cv::Point(1000, 660), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Width Standard Deviation: " + width_sd_print, cv::Point(1000, 690), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Maximum Width: " + width_max_print + " mm", cv::Point(1000, 720), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Minimum Width: " + width_min_print + " mm", cv::Point(1000,740), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
    cv::putText(drawing, "Designed Center: [" + cal_center_print_x + "," + cal_center_print_y + "]", cv::Point(1000, 760), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
    // cv::putText(drawing, "Actual Center: [" + center_print_x + "," + center_print_y + "]", cv::Point(1000, 710), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),2);
}

laser_beam_return callLaserBeamAlign(Mat image_captured, string path, string path_rmatrix, string path_tvec, Mat src, int min_size, int last_min_size, int size_array[10], vector<Point> center_rect_list, double center_rect_count)
{
    vector<double> rmatrix_laser_values;
    vector<double> tvec_laser_values;
    Mat img_grey, dot_img;
    Point centerImage;
    int size_avg;

    ifstream intrin(path + "values/intrinsic.txt");
    vector<double> cameraMatrix_values;
    double val;
    while (intrin >> val)
    {
        cameraMatrix_values.push_back(val);
    }

    ifstream dist(path + "values/distortion.txt");
    vector<double> distCoeffs_values;
    while (dist >> val)
    {
        distCoeffs_values.push_back(val);
    }
    Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
    Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());

    Size patternSize (7,4);
    double squareSize = 7;
    solvePnP_result solvePnP_result = getRvecTvec(image_captured, patternSize, squareSize);

    ifstream rmatrixL(path_rmatrix);
    while (rmatrixL >> val)
    {
        rmatrix_laser_values.push_back(val);
    }
    ifstream tvecL(path_tvec);
    while (tvecL >> val)
    {
        tvec_laser_values.push_back(val*1000);
    }
    pair<vector<double>,vector<double>>target = targetBoardPlane(solvePnP_result.rmatrix, solvePnP_result.tvec);
    laser_plane laser_1;
    laser_1 = laserPlane(rmatrix_laser_values, tvec_laser_values);
    Point3f interPoint1;
    interPoint1 = intersectionPoint(laser_1.origin, laser_1.beam_dir, target.first, target.second);
    vector<Point3d> laserline_points_1;
    intersection line1;
    line1 = intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});
    for(int t=-150; t<150;)
    {
        t = t+10;
        Point3d points((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
//                    cout<<"point: "<<points<<endl;
        laserline_points_1.push_back(points);
    }
    vector<Point2d> projectedlaserline_1;
    projectPoints(laserline_points_1, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_1);

    for(int i=0; i < projectedlaserline_1.size()-1;)
    {
        if((projectedlaserline_1[i].x > 1440.0) || (projectedlaserline_1[i].y > 1080.0) || (projectedlaserline_1[i].x < 0.0) || (projectedlaserline_1[i].y < 0.0))
        {
            projectedlaserline_1.erase(projectedlaserline_1.begin()+i);
        }
        else
        {
            i++;
        }
    }
    //----------raw image to greyscale, threshold filter
    cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
    dot_img = src.clone();

    Mat img_grey_filtered_dot;
    threshold(img_grey,img_grey_filtered_dot,200,255,cv::THRESH_OTSU||cv::THRESH_TRIANGLE);
    line(dot_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2], Scalar(0,0,255), 4, LINE_AA);

    vector<Point3d> interPointArray;
    vector<Point2d> interPointsImage_vector;
    interPointArray.push_back(interPoint1);
    projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,interPointsImage_vector);
    cv::circle( dot_img, interPointsImage_vector[0], 5, cv::Scalar(0,0,255), -1, 8, 0 );
    int non_zero = NonZero(img_grey_filtered_dot);
    int count = PixelCounter(img_grey_filtered_dot);
    size_avg = SizeAverage(count,0,size_array, center_rect_count);
    if(size_avg < min_size && center_rect_count > 20)
    {
        if(abs(size_array[0]-size_array[9]) <=100) // changed
        {
            min_size = size_avg;
        }
    }
    double nom_distance,center_distance;
    Mat threshold_output;
    threshold(img_grey,threshold_output,200,255,cv::THRESH_BINARY);
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    if(non_zero > 50)
    {
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Point center_rect_avg;
        findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        if(contours.size() > 0)
        {
            vector<RotatedRect> minRect = findRectangle(contours,20);
            if(minRect.size() >= 1 && minRect.size()<10) // changed
            {
                center_rect_list.push_back(minRect[0].center);
                center_rect_count++;
                drawContourRectangle(drawing, contours, minRect);
                cv::circle( dot_img, minRect[0].center, 3, cv::Scalar(100,255,0), -1, 8, 0 );

            }
            if (center_rect_count > 30)
            {
                center_rect_list.erase(center_rect_list.begin());
                center_rect_count --;
            }
            if (center_rect_count >= 10)
            {
                Point sum = accumulate(center_rect_list.begin(), center_rect_list.end(), Point(0,0));
                center_rect_avg = sum*(1.0/center_rect_count);

                circle(dot_img, center_rect_avg, 3, Scalar(255,0,255), -1, 8, 0);
                std::pair<double,double>dist = DotToLine(dot_img, projectedlaserline_1[0], projectedlaserline_1[19], center_rect_avg, interPointsImage_vector[0]);
                nom_distance = dist.first;
                center_distance = dist.second;
                centerImage = center_rect_avg;
            }
        }
    }
    else
    {
        last_min_size = min_size;
        fill_n(size_array,10,0);
        center_rect_count = 0;
        center_rect_list.clear();
    }

    Point3d point_1 = locationCam2Target(interPointsImage_vector[0], solvePnP_result);

    HMI(dot_img, size_avg, min_size, non_zero, nom_distance, center_distance);
    GreenLight(dot_img, last_min_size, size_avg, nom_distance, center_distance);
//    imshow("img_grey_filtered_dot", img_grey_filtered_dot);
//    imshow("threshold output", threshold_output);
//    imshow("Contour and Rectangle", drawing);
    imshow("Laser Beam Alignment Window", dot_img);
    laser_beam_return laserBeamReturn;
    laserBeamReturn.center_rect_count = center_rect_count;
    laserBeamReturn.center_rect_list = center_rect_list;
    laserBeamReturn.last_min_size = last_min_size;
    laserBeamReturn.min_size = min_size;
    laserBeamReturn.dot_img = dot_img;
    for(int i=0; i<10;i++){
    laserBeamReturn.size_array[i] = size_array[i];
    }
    return laserBeamReturn;
}


laser_plane_return callLaserPlaneAlign(Mat image_captured, string path, string path_rmatrix, string path_tvec, Mat src)
{
    Mat line_img;
    ifstream intrin(path + "values/intrinsic.txt");
    vector<double> cameraMatrix_values;
    double val;
    while (intrin >> val)
    {
        cameraMatrix_values.push_back(val);
    }
    ifstream dist(path + "values/distortion.txt");
    vector<double> distCoeffs_values;
    while (dist >> val)
    {
        distCoeffs_values.push_back(val);
    }
    Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix_values.data());
    Mat distCoeffs = Mat(5, 1, CV_64FC1, distCoeffs_values.data());

    Size patternSize (7,4);
    double squareSize = 7;
    solvePnP_result solvePnP_result = getRvecTvec(image_captured,patternSize,squareSize);

    vector<double> rmatrix_laser_values;
    vector<double> tvec_laser_values;
    ifstream rmatrixL(path_rmatrix);
    while (rmatrixL >> val)
    {
        rmatrix_laser_values.push_back(val);
    }

    ifstream tvecL(path_tvec);
    while (tvecL >> val)
    {
        tvec_laser_values.push_back(val*1000);
    }

    // find target board plane in cam frame
    pair<vector<double>,vector<double>>target = targetBoardPlane(solvePnP_result.rmatrix, solvePnP_result.tvec);

    laser_plane laser_1;
    laser_1 = laserPlane(rmatrix_laser_values, tvec_laser_values);

    // find intersection between laser beam and target board
    Point3f interPoint1;
    interPoint1 = intersectionPoint(laser_1.origin, laser_1.beam_dir, target.first, target.second);
    vector<Point3d> interPointArray;
    vector<Point2d> projectedInterPoints;
    interPointArray.push_back(interPoint1);
    projectPoints(interPointArray, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs,projectedInterPoints);
    // cout<<endl<<"Intersection between laser beam and target board on camera image: "<<endl<<projectedInterPoints<<endl;

    // find intersection line between target board plane and laser plane in cam frame
    vector<cv::Point3d> laserline_points_1;
    intersection line1;
    line1 = intersectionLine(target.first, laser_1.normalvector, target.second, vector<double>{interPoint1.x, interPoint1.y, interPoint1.z});

    for(int t=-150; t<150;)
    {
        t = t+10;
        Point3d points((line1.x0+line1.a*t), (line1.y0+line1.b*t), (line1.z0+line1.c*t));
        laserline_points_1.push_back(points);
    }
    vector<Point2d> projectedlaserline_1;
    projectPoints(laserline_points_1, Mat::zeros(3,1,CV_64FC1), Mat::zeros(3,1,CV_64FC1),cameraMatrix,distCoeffs, projectedlaserline_1);

    for(int i=0; i < projectedlaserline_1.size()-1;)
    {
        if((projectedlaserline_1[i].x > 1440.0) || (projectedlaserline_1[i].y > 1080.0) || (projectedlaserline_1[i].x < 0.0) || (projectedlaserline_1[i].y < 0.0))
        {
            projectedlaserline_1.erase(projectedlaserline_1.begin()+i);
        }
        else
        {
            i++;
        }
    }
    // cout<<"two points on line: "<<projectedlaserline_1[0]<<projectedlaserline_1[projectedlaserline_1.size()-2]<<endl;

    // Calculated laser line angle
    double delta_y = (projectedlaserline_1[projectedlaserline_1.size()-2].y - projectedlaserline_1[1].y);
    double delta_x = (projectedlaserline_1[projectedlaserline_1.size()-2].x - projectedlaserline_1[1].x);
    double cal_angle = atan(delta_y/delta_x)*180/CV_PI;
    if (cal_angle < 0)
    {cal_angle = 90 + cal_angle;}

    Mat img_grey;
    cv::cvtColor(src, img_grey, cv::COLOR_BGR2GRAY);
    line_img = src.clone();
    line( line_img, projectedlaserline_1[0], projectedlaserline_1[projectedlaserline_1.size()-2],Scalar(0,0,255), 1, LINE_AA );

    cv::circle( line_img, projectedInterPoints[0], 5, cv::Scalar(0,0,255), -1, 8, 0 );
    // cout<<"one point: "<< projectedInterPoints[0]<<endl;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat threshold_output;
    threshold(img_grey,threshold_output,200,255,cv::THRESH_BINARY);

    findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<RotatedRect> minRect = findRectangle(contours,100);
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    Mat rotated_image = threshold_output.clone();

    if (minRect.size() >= 1)
    {
        drawContourRectangle(drawing, contours, minRect);
        circle(line_img, minRect[0].center, 5, Scalar(0,255,0), -1, 8, 0);
        double angle = minRect[0].angle;
        if (minRect[0].size.width < minRect[0].size.height)
        {
            angle = 90 + angle;
        }
        Mat rotation_matrix = getRotationMatrix2D(minRect[0].center, angle, 1.0);
        warpAffine(threshold_output, rotated_image, rotation_matrix, threshold_output.size());
        uniformity_data uniformity1;
        uniformity1 = cropImage(rotated_image);
        cout<<"2"<<endl;
        // From pixel number to actual diameter on target board
        uniformity1.width_avg = uniformity1.width_avg* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;
        uniformity1.width_max = uniformity1.width_max* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;
        uniformity1.width_min = uniformity1.width_min* 3.45 * (solvePnP_result.tvec.at<double>(0,2)/12)/1000;

        laserlineGUI(minRect[0], projectedInterPoints[0], cal_angle, uniformity1, line_img);
        // cv::imshow( "Rotated and Cropped laser line", uniformity1.image_BGR );
    }

    // cv::imshow( "Contour and Area", drawing );
    // cv::imshow("threshold",threshold_output);
    cv::imshow("Laser Plane Alignment GUI Window", line_img);
    laser_plane_return result;
    result.line_img = line_img;
    result.threshold_output = threshold_output;
    result.solvePnP_result = solvePnP_result;
    return result;
}

laser_verification_return callLaserVerification(string path, string path_rmatrix, string path_tvec, string path_beam_verify, string path_plane_verify)
{
    vector<double> rmatrix_laser_values;
    vector<double> tvec_laser_values;
    ifstream rmatrixL(path_rmatrix);
    double val;
    while (rmatrixL >> val)
    {
        rmatrix_laser_values.push_back(val);
    }

    ifstream tvecL(path_tvec);
    while (tvecL >> val)
    {
        tvec_laser_values.push_back(val*1000);
    }
    cout<<endl<<"beam verification-------------------------------------------"<<endl;
        vector<double> imgPoint_d1_vector, imgPoint_d2_vector, imgPoint_d3_vector;

        ifstream readPointd1(path_beam_verify+"_d1.txt");
        while (readPointd1 >> val)
        {
            imgPoint_d1_vector.push_back(val);
        }
        Point2d imgPoint_d1;
        imgPoint_d1.x = imgPoint_d1_vector[0];
        imgPoint_d1.y = imgPoint_d1_vector[1];

        ifstream readPointd2(path_beam_verify+"_d2.txt");
        while (readPointd2 >> val)
        {
            imgPoint_d2_vector.push_back(val);
        }
        Point2d imgPoint_d2;
        imgPoint_d2.x = imgPoint_d2_vector[0];
        imgPoint_d2.y = imgPoint_d2_vector[1];

        ifstream readPointd3(path_beam_verify+"_d3.txt");
        while (readPointd3 >> val)
        {
            imgPoint_d3_vector.push_back(val);
        }
        Point2d imgPoint_d3;
        imgPoint_d3.x = imgPoint_d3_vector[0];
        imgPoint_d3.y = imgPoint_d3_vector[1];

        solvePnP_result solvePnP_result_d1,solvePnP_result_d2,solvePnP_result_d3;
        Mat image_captured_d1, image_captured_d2, image_captured_d3;
        image_captured_d1 = imread(path + "images/pattern_d1.png", IMREAD_GRAYSCALE);
        image_captured_d2 = imread(path + "images/pattern_d2.png", IMREAD_GRAYSCALE);
        image_captured_d3 = imread(path + "images/pattern_d3.png", IMREAD_GRAYSCALE);

        Size patternSize (7,4);
        double squareSize = 7;
        solvePnP_result_d2 = getRvecTvec(image_captured_d2,patternSize,squareSize);
        solvePnP_result_d3 = getRvecTvec(image_captured_d3,patternSize,squareSize);
        solvePnP_result_d1 = getRvecTvec(image_captured_d1,patternSize,squareSize);

        Point3d p1 = locationCam2Target( imgPoint_d1, solvePnP_result_d1);
        Point3d p2 = locationCam2Target( imgPoint_d2, solvePnP_result_d2);
        Point3d p3 = locationCam2Target( imgPoint_d3, solvePnP_result_d3);
//        cout<<endl<<"3 points in camera frame:   " <<p1<<" "<<p2<<" "<<p3<<endl;
        Point3d actual_origin = lineEquation(p1,p3,tvec_laser_values);
        cout<<endl<<"ideal laser origin: "<<"("<<tvec_laser_values[0]<<", "<<tvec_laser_values[1]<<", "<<tvec_laser_values[2]<<")"<<endl;

        cout<<endl<<endl<<"plane verification-------------------------------------------"<<endl;
        vector<Point3d> start_vector, end_vector;
        ifstream start_d1, start_d2, start_d3, end_d1, end_d2, end_d3;

        std::ostringstream oss;
        std::string path_start, path_end;

        for(int i = 1; i<=3; i++)
        {
            path_start =path_plane_verify + "_d" + to_string(i) + ".txt";
            path_end = path_plane_verify + "_d" + to_string(i) + ".txt";
            if(i==1){start_d1.open(path_start); end_d1.open(path_end);}
            else if(i==2){start_d2.open(path_start); end_d2.open(path_end);}
            else if(i==3){start_d3.open(path_start); end_d3.open(path_end);}
        }

        double x, y, z;
        char comma;
        // Read Point3d in vector
        while (start_d1 >> x >> comma >> y >> comma >> z)
        {
            start_vector.push_back(Point3d(x,y,z));
        }
        while (start_d2 >> x >> comma >> y >> comma >> z)
        {
            start_vector.push_back(Point3d(x,y,z));
        }
        while (start_d3 >> x >> comma >> y >> comma >> z)
        {
            start_vector.push_back(Point3d(x,y,z));
        }
        while (end_d1 >> x >> comma >> y >> comma >> z)
        {
            end_vector.push_back(Point3d(x,y,z));
        }
        while (end_d2 >> x >> comma >> y >> comma >> z)
        {
            end_vector.push_back(Point3d(x,y,z));
        }
        while (end_d3 >> x >> comma >> y >> comma >> z)
        {
            end_vector.push_back(Point3d(x,y,z));
        }

        // cout<<endl<< "start_d1: "<< start_vector[0]<<endl;
        // cout<<endl<< "start_d2: "<< start_vector[1]<<endl;
        // cout<<endl<< "start_d3: "<< start_vector[2]<<endl;
        // cout<<endl<< "end_d1: "<< end_vector[0]<<endl;
        // cout<<endl<< "end_d2: "<< end_vector[1]<<endl;
        // cout<<endl<< "end_d3: "<< end_vector[2]<<endl;

        // Vector for vectors in 3D space from start to end points
        vector<vector<double>> vect3D_collection;

        for(int s=0; s<3; s++)
        {
            vector<double> v1;
            for(int e=0; e<3; e++)
            {
                vector<double> v1;
                v1.push_back(end_vector[e].x-start_vector[s].x);
                v1.push_back(end_vector[e].y-start_vector[s].y);
                v1.push_back(end_vector[e].z-start_vector[s].z);
                // cout<<endl<<v1[0]<<" "<<v1[1]<<" "<<v1[2]<<endl;
                vect3D_collection.push_back(v1);
            }
        }
        double min;
        vector<vector<double>> normalVector_collection;
        for(int i=0; i<3; i++)
        {
            vector<double> NV = crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+1]);
            double d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
            NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
            normalVector_collection.push_back(NV);

            NV = crossProduct(vect3D_collection[3*i],vect3D_collection[3*i+2]);
            d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
            NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
            normalVector_collection.push_back(NV);

            NV = crossProduct(vect3D_collection[3*i+1],vect3D_collection[3*i+2]);
            d = sqrt(NV[0]*NV[0] + NV[1]*NV[1] + NV[2]*NV[2]);
            NV[0] = NV[0]/d; NV[1] = NV[1]/d; NV[2] = NV[2]/d;
            normalVector_collection.push_back(NV);

            // normalVector_collection.push_back(laserline::crossProduct(vect3D_collection[3*i+1],vect3D_collection[3*i+2]));
            // min = *min_element(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end());
            // transform(normalVector_collection[3*i+2].begin(), normalVector_collection[3*i+2].end(), normalVector_collection[3*i+2].begin(), [min](double &c){ return c/min; });
        }

            double xSum = 0; double ySum = 0; double zSum = 0;
            Point3d norm_avg;
            for(int i=0; i<9; i++)
            {
                // cout << endl << "The actual normal vectors: " << normalVector_collection[i][0] << "," << normalVector_collection[i][1] << "," << normalVector_collection[i][2]<< endl;
                xSum = xSum + normalVector_collection[i][0];
                ySum = ySum + normalVector_collection[i][1];
                zSum = zSum + normalVector_collection[i][2];
            }
            norm_avg.x = xSum/9;
            norm_avg.y = ySum/9;
            norm_avg.z = zSum/9;
            cout<<endl<<"The actual normal vector average: "<< norm_avg<<endl;

            laser_plane laser_plane;
            laser_plane = laserPlane(rmatrix_laser_values, tvec_laser_values);
            cout<<endl<<"The ideal normal vector of ideal laser plane: "<<"["<< laser_plane.normalvector[0]<<","<<laser_plane.normalvector[1]<<","<<laser_plane.normalvector[2]<<"]"<<endl;
            laser_verification_return verification_return;
            verification_return.ideal_origin = Point3d(tvec_laser_values[0],tvec_laser_values[1],tvec_laser_values[2]);
            verification_return.actual_origin = actual_origin;
            verification_return.ideal_normalV = Point3d(laser_plane.normalvector[0],laser_plane.normalvector[1],laser_plane.normalvector[2]);
            verification_return.actual_normalV = norm_avg;

            verification_return.delta_origin.x = verification_return.actual_origin.x - verification_return.ideal_origin.x;
            verification_return.delta_origin.y = verification_return.actual_origin.y - verification_return.ideal_origin.y;
            verification_return.delta_origin.z = verification_return.actual_origin.z - verification_return.ideal_origin.z;
            verification_return.delta_normalV.x = verification_return.actual_normalV.x - verification_return.ideal_normalV.x;
            verification_return.delta_normalV.y = verification_return.actual_normalV.y - verification_return.ideal_normalV.y;
            verification_return.delta_normalV.z = verification_return.actual_normalV.z - verification_return.ideal_normalV.z;

//            cout<<endl<<verification_return.ideal_origin<<endl<<verification_return.actual_origin<<endl<<verification_return.ideal_normalV<<endl<<verification_return.actual_normalV<<endl;
//            cout<<verification_return.delta_origin<<endl<<verification_return.delta_normalV<<endl;
            return verification_return;
}
