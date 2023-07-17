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

void addTagPos(cv::Point3f position, double tagSize, std::vector<std::vector<cv::Point3f>>& objPoints) {
    std::vector<cv::Point3f> tagCorners;
    tagCorners.emplace_back(position);
    tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y, position.z));
    tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y - tagSize, position.z));
    tagCorners.emplace_back(cv::Point3f(position.x, position.y - tagSize, position.z));
    objPoints.emplace_back(tagCorners);
}

void flattenVector(std::vector<std::vector<cv::Point3f>> &objPoints, std::vector<cv::Point3f> &newObjPoints) {
    for (std::vector<cv::Point3f> objPoint : objPoints) {
        newObjPoints.insert(end(newObjPoints), begin(objPoint), end(objPoint));
    }
}

void flattenVector(std::vector<std::vector<cv::Point2f>> &objPoints, std::vector<cv::Point2f> &newObjPoints) {
    for (std::vector<cv::Point2f> objPoint : objPoints) {
        newObjPoints.insert(end(newObjPoints), begin(objPoint), end(objPoint));
    }
}

// void arucoLocate(Mat cvCamMat, Mat cvDistCoeffs)
// {
//     double tagSize = 0.02;

//     cv::Point3f tag0Pos(0, 0, 0);
//     cv::Point3f tag1Pos(0.18, 0, 0);
//     cv::Point3f tag2Pos(0, -0.12, 0);
//     cv::Point3f tag3Pos(0.18, -0.12, 0);

//     std::vector<std::vector<cv::Point3f>> objPoints;
//     addTagPos(tag0Pos, tagSize, objPoints);
//     addTagPos(tag1Pos, tagSize, objPoints);
//     addTagPos(tag2Pos, tagSize, objPoints);
//     addTagPos(tag3Pos, tagSize, objPoints);
//     cv::Ptr<cv::aruco::Dictionary> boardDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
//     Pylon::CGrabResultPtr grab_result;
//     Pylon::CPylonImage pylon_image;
//     Pylon::CImageFormatConverter converter;
//     converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;

//     camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

//     std::vector<cv::Point3f> flattenedObjPoints;

//     flattenVector(objPoints, flattenedObjPoints);

//     camera.RetrieveResult(1000, grab_result);
//     converter.Convert(pylon_image, grab_result);
//     Mat image = cv::Mat(grab_result->GetHeight(), grab_result->GetWidth(), CV_8UC3, (uint8_t *)pylon_image.GetBuffer());

//     // cap.read(image);
//     // // check if we succeeded
//     // if (image.empty())
//     // {
//     //     std::cerr << "ERROR! blank frame grabbed\n";
//     //     break;
//     // }
//     std::vector<int> ids;
//     std::vector<cv::Point2f> flattenedMarkerCorners;
//     std::vector<std::vector<cv::Point2f>> markerCorners;
//     // std::cout << "HERE" << std::endl;
//     cv::aruco::detectMarkers(image, boardDict, markerCorners, ids);
//     flattenVector(markerCorners, flattenedMarkerCorners);
//     if (ids.size() > 0)
//     {
//         cv::aruco::drawDetectedMarkers(image, markerCorners, ids);

//         cv::Vec3d rvec, tvec;
//         if (flattenedMarkerCorners.size() == flattenedObjPoints.size())
//         {
//             std::vector<cv::Point3f> fixedObjPoints;
//             for (auto id : ids)
//             {
//                 if (id < objPoints.size())
//                     fixedObjPoints.insert(end(fixedObjPoints), begin(objPoints.at(id)), end(objPoints.at(id)));
//             }
//             if (fixedObjPoints.size() == flattenedMarkerCorners.size())
//             {
//                 cv::solvePnP(fixedObjPoints, flattenedMarkerCorners, cvCamMat, cvDistCoeffs, rvec, tvec);
//                 cv::aruco::drawAxis(image, cvCamMat, cvDistCoeffs, rvec, tvec, 0.1);
//                 std::cout<<"rvec: "<<rvec<<std::endl<<"tvec: "<<tvec<<std::endl;

//             }
//         }
//         cv::imshow("Output Window", image);

//         // if (cv::waitKey(10) == 27)
//         // break;
//     }
// }