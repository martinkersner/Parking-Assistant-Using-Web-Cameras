/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Collision Avoidance System manages all subsystems: 
 *   Undistortion,
 *   DisparityMap,
 *   Triangulation
 *   ObjectDetection
 *
 * m.kersner@gmail.com
 * 03/28/2015
 */

#include "CollisionAvoidanceSystem.h"

CollisionAvoidanceSystem::CollisionAvoidanceSystem( std::string _extrinsics,
                                                    std::string _distortions,
                                                    cv::Mat _background,
                                                    float _squareSize,
                                                    bool _demo )  
    : extrinsics(_extrinsics),
      distortions(_distortions),
      background(_background),
      squareSize(_squareSize),
      demo(_demo)
{
    Initialize();
}

void CollisionAvoidanceSystem::SetNumberOfDisparities( int disparityNumber )
{
    this->disparityMap.SetNumberOfDisparities(disparityNumber);
}

void CollisionAvoidanceSystem::Initialize()  
{
    this->undistortion = Undistortion(this->distortions);

    // disparity calculation settings
    this->disparityMap.SetPreFilterCap(63);
    this->disparityMap.SetSATWindowSize(21);
    this->disparityMap.SetNumberOfDisparities(176);

    this->objectDetection = ObjectDetection(this->background);

    this->triangulation = Triangulation(this->extrinsics, this->squareSize);
}

Object CollisionAvoidanceSystem::Detect( cv::Mat & left,
                                         cv::Mat & right )
{
    // undistortion & rectification
    cv::Mat leftCorrect  = this->undistortion.CorrectLeftImage(left);
    cv::Mat rightCorrect = this->undistortion.CorrectRightImage(right);

    // calculate disparity map
    cv::Mat disparity = this->disparityMap.CalculateDisparity( leftCorrect, 
                                                               rightCorrect );
    cv::Mat tmpDisparity = disparity.clone();
    cv::normalize(disparity, disparity, 0, 256, CV_MINMAX);
    disparity.convertTo(disparity, CV_8UC1);

    // object detection
    Object object = objectDetection.Detect(disparity);

    // triangulation
    if (object.found) {

        // only for purposes of demo
        if (this->demo) {
            cv::Mat mask, concatenation;
            cv::cvtColor(object.mask*255, mask, CV_GRAY2RGB);
            cv::hconcat(leftCorrect, mask, concatenation);
            cv::imshow("Undistorted Source Image and Mask of Object", concatenation);
            cv::waitKey();
        }

        // triangulation
        cv::Mat triangulatedMap = triangulation.DisparityToDepth(tmpDisparity);
        object.distance = cv::mean(triangulatedMap, object.mask)[0];
    }

    return object;
}

float CollisionAvoidanceSystem::Detect( cv::Mat & left,
                                        cv::Mat & right,
                                        cv::Mat & mask )
{
    float distance;

    // undistortion & rectification
    cv::Mat leftCorrect  = this->undistortion.CorrectLeftImage(left);
    cv::Mat rightCorrect = this->undistortion.CorrectRightImage(right);

    // calculate disparity map
    cv::Mat disparity = this->disparityMap.CalculateDisparity( leftCorrect, 
                                                               rightCorrect );
    cv::Mat tmpDisparity = disparity.clone();
    cv::normalize(disparity, disparity, 0, 256, CV_MINMAX);
    disparity.convertTo(disparity, CV_8UC1);

    // triangulation
    cv::Mat triangulatedMap = triangulation.DisparityToDepth(tmpDisparity);
    distance = cv::mean(triangulatedMap, mask)[0];

    return distance;
}

void CollisionAvoidanceSystem::ReinitializeCalibrationFiles()
{
    this->undistortion.ReloadDistortion();
    this->triangulation.ReloadExtrinsics();
}
