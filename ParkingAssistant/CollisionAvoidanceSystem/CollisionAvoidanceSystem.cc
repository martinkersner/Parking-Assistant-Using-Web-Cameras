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
                                                    cv::Mat _background )  
    : extrinsics(_extrinsics),
      distortions(_distortions),
      background(_background)
{
    Initialize();
}

void CollisionAvoidanceSystem::Initialize()  
{
    this->undistortion = Undistortion(this->distortions);

    // disparity calculation settings
    disparityMap.SetPreFilterCap(63);
    disparityMap.SetSATWindowSize(21);
    disparityMap.SetNumberOfDisparities(128);

    this->objectDetection = ObjectDetection(this->background);

    this->triangulation = Triangulation(this->extrinsics);
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

    // TODO 
    cv::Mat tmpDisparity = disparity.clone();

    cv::normalize(disparity, disparity, 0, 256, CV_MINMAX);
    disparity.convertTo(disparity, CV_8UC1);
    
    // object detection
    Object object = objectDetection.Detect(disparity);

    // triangulation
    if (object.found) {

        // TODO decide which method to use
        //      how to compute distance
        cv::Mat triangulatedMap = triangulation.DisparityToDepth(tmpDisparity);
        object.distance = cv::mean(triangulatedMap, object.mask)[0];

        //cv::normalize(triangulatedMap, triangulatedMap, 0, 256, CV_MINMAX);
        //triangulatedMap.convertTo(triangulatedMap, CV_8UC1);
        //cv::imshow("tria", triangulatedMap);
        //cv::waitKey();
    }

    return object;
}
