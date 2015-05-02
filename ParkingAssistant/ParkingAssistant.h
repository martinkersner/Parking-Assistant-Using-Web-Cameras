#ifndef PARKINGASSISTANT_H_
#define PARKINGASSISTANT_H_

#include "Properties.h"
#include "Miscellaneous/Fps.h"
#include "Miscellaneous/TimeMeasure.h"
#include "Miscellaneous/ImageAcquisition.h"
#include "Calibration/Calibration.h"
#include "CollisionAvoidanceSystem/CollisionAvoidanceSystem.h"
#include "CurbDetection/CurbDetection.h"

#define DEMO 1

enum State { STANDBY, CALIBRATION, RUNNING };

struct ObjectDetectionSubset {
    std::string name;
    int number;
};

struct DepthEstimationScene {
    int index;
    int distance;
};

int ParkingAssistantRealTime( std::string distortion, 
                              std::string extrinsics,
                              cv::Mat & background,
                              float squareSize );

void DisplayLeftAndRight( cv::Mat & left,
                          cv::Mat & right );

void ObjectDetectionDemo( std::string datasetPath );
void DepthEstimationDemo( std::string datasetPath );
void CurbDetectionDemo( std::string datasetPath );

#endif // PARKINGASSISTANT_H_
