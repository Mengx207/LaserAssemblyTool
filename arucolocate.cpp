#include "utility.h"
using namespace GENAPI_NAMESPACE;

// void addTagPos(cv::Point3f position, double tagSize, std::vector<std::vector<cv::Point3f>>& objPoints) {
//     std::vector<cv::Point3f> tagCorners;
//     tagCorners.emplace_back(position);
//     tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y, position.z));
//     tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y - tagSize, position.z));
//     tagCorners.emplace_back(cv::Point3f(position.x, position.y - tagSize, position.z));
//     objPoints.emplace_back(tagCorners);
// }

void addTagPos(cv::Point3f position, double tagSize, std::vector<std::vector<cv::Point3f>>& objPoints) {
    std::vector<cv::Point3f> tagCorners;
    tagCorners.emplace_back(position);
    tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y, position.z));
    tagCorners.emplace_back(cv::Point3f(position.x + tagSize, position.y + tagSize, position.z));
    tagCorners.emplace_back(cv::Point3f(position.x, position.y + tagSize, position.z));
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

int main(int argc, char *argv[])
{
    system("cd values && mkdir -p aruco_result");
    Pylon::PylonInitialize();
    Pylon::CTlFactory &tlFactory = Pylon::CTlFactory::GetInstance();
    Pylon::CInstantCamera camera(tlFactory.CreateFirstDevice());
    cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
    cout << "SerialNumber : " << camera.GetDeviceInfo().GetSerialNumber() << endl;

    double camMat[3][3] = {3573.681082, 0, 733.98215, 0, 3572.916732, 576.44937, 0, 0, 1};
    double distCoeffs[5] = {-0.05969, -0.981368, -0.000368, -0.000843, 19.9579};

    cv::Mat cvCamMat(3, 3, 6, camMat);
    cv::Mat cvDistCoeffs(1, 5, 6, distCoeffs);

    std::vector<std::vector<cv::Point3f>> objPoints;
    double tagSize = 0.01;

    cv::Point3f tag0Pos(0, 0, 0);
    cv::Point3f tag1Pos(0.09, 0, 0);
    cv::Point3f tag2Pos(0.09, 0.06, 0);
    cv::Point3f tag3Pos(0, 0.06, 0);


    addTagPos(tag0Pos, tagSize, objPoints);
    addTagPos(tag1Pos, tagSize, objPoints);
    addTagPos(tag2Pos, tagSize, objPoints);
    addTagPos(tag3Pos, tagSize, objPoints);


    cv::Ptr<cv::aruco::Dictionary> boardDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    camera.RegisterConfiguration(new CSoftwareTriggerConfiguration1,RegistrationMode_ReplaceAll, Cleanup_Delete);

    camera.Open();

    Mat cam_frame, image;
    Pylon::CGrabResultPtr grab_result;
    Pylon::CPylonImage pylon_image;
    Pylon::CImageFormatConverter converter;
    converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;

    INodeMap& nodemap = camera.GetNodeMap();

    CEnumerationPtr(nodemap.GetNode("ExposureMode"))->FromString("Timed"); 
    CFloatPtr(nodemap.GetNode("ExposureTime"))->SetValue(3000.0);
    CEnumParameter(nodemap, "LineSelector").SetValue("Line3");
    CEnumParameter(nodemap, "LineMode").SetValue("Output");
    CEnumParameter(nodemap, "LineSource").SetValue("UserOutput2");

    CEnumParameter(nodemap, "LineSelector").SetValue("Line4");
    CEnumParameter(nodemap, "LineMode").SetValue("Output");
    CEnumParameter(nodemap, "LineSource").SetValue("UserOutput3");

    CEnumParameter(nodemap, "LineSelector").SetValue("Line3");
    CBooleanParameter(nodemap, "LineInverter").SetValue(false);

    CEnumParameter(nodemap, "LineSelector").SetValue("Line4");			
    CBooleanParameter(nodemap, "LineInverter").SetValue(false);

    CEnumParameter(nodemap, "LineSelector").SetValue("Line2");
    CEnumParameter(nodemap, "LineSource").SetValue("ExposureActive");

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    std::vector<cv::Point3f> flattenedObjPoints;

    flattenVector(objPoints, flattenedObjPoints);

    std::cout << flattenedObjPoints;

    
    while (true)
    {
        waitKey(100);
        while(camera.WaitForFrameTriggerReady(1000,TimeoutHandling_ThrowException)==0);			
        CCommandParameter(nodemap, "TriggerSoftware").Execute();
        bool test = camera.RetrieveResult(1000, grab_result, TimeoutHandling_ThrowException);
        converter.Convert(pylon_image, grab_result);
        cam_frame = cv::Mat(grab_result->GetHeight(), grab_result->GetWidth(), CV_8UC3, (uint8_t *) pylon_image.GetBuffer());
        image = cam_frame.clone();

        std::vector<int> ids;
        std::vector<cv::Point2f> flattenedMarkerCorners;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        // std::cout << "HERE" << std::endl;
        cv::aruco::detectMarkers(image, boardDict, markerCorners, ids);
        flattenVector(markerCorners, flattenedMarkerCorners);

        cv::Vec3d rvec, tvec;
        cv::Mat rmatrix;
        std::vector<cv::Point3f> fixedObjPoints;
        if (ids.size() > 0)
        {
            std::cout<<std::endl<<ids[0]<<ids[1]<<ids[2]<<ids[3]<<std::endl;

            cv::aruco::drawDetectedMarkers(image, markerCorners, ids);

            if (flattenedMarkerCorners.size() == flattenedObjPoints.size())
            {
                for (auto id : ids)
                {
                    if (id < objPoints.size())
                        fixedObjPoints.insert(end(fixedObjPoints), begin(objPoints.at(id)), end(objPoints.at(id)));
                }
                if (fixedObjPoints.size() == flattenedMarkerCorners.size())
                {
                    cv::solvePnP(fixedObjPoints, flattenedMarkerCorners, cvCamMat, cvDistCoeffs, rvec, tvec);
                    Rodrigues(rvec, rmatrix);

                    cv::aruco::drawAxis(image, cvCamMat, cvDistCoeffs, rvec, tvec, 0.05);
                    std::cout<<"rvec: "<<rvec<<std::endl<<"tvec: "<<tvec<<std::endl;
                    cout<<endl<<"fixedObjPoints: "<<endl;
                    for(int i=0; i<fixedObjPoints.size(); i++)
                    {cout<<fixedObjPoints[i]<<endl;}

                    cout<<endl<<"flattenedMarkerCorners: "<<endl;
                    for(int i=0; i<flattenedMarkerCorners.size(); i++)
                    {cout<<flattenedMarkerCorners[i]<<endl;}

                }
            }
        }
        cv::imshow("Output Window", image);

        if (cv::waitKey(10) == 27)
        {
			ofstream rvec_save("values/aruco_result/rvec_target2cam.txt");
			rvec_save << rvec[0]<<" "<<rvec[1]<<" "<<rvec[2];
            ofstream rmatrix_save("values/aruco_result/rmatrix_target2cam.txt");
			rmatrix_save << rmatrix.at<double>(0,0)<<" "<<rmatrix.at<double>(0,1)<<" "<<rmatrix.at<double>(0,2)<<" "<<rmatrix.at<double>(1,0)<<" "<<rmatrix.at<double>(1,1)<<" "<<rmatrix.at<double>(1,2)<<" "<<rmatrix.at<double>(2,0)<<" "<<rmatrix.at<double>(2,1)<<" "<<rmatrix.at<double>(2,2);

            ofstream tvec_save("values/aruco_result/tvec_target2cam.txt");
			tvec_save << tvec[0]<<" "<<tvec[1]<<" "<<tvec[2];

            ofstream obj("values/aruco_result/corners_obj.txt");
			for(int i=0; i<fixedObjPoints.size(); i++)
            {obj << fixedObjPoints[i].x<<" "<<fixedObjPoints[i].y<<" "<<fixedObjPoints[i].z<<" ";}

            ofstream found("values/aruco_result/corners_img.txt");
			for(int i=0; i<flattenedMarkerCorners.size(); i++)
            {found << flattenedMarkerCorners[i].x<<" "<< flattenedMarkerCorners[i].y<<" ";}

            break;
        }
            
    }
    // Pylon::PylonTerminate();
}
