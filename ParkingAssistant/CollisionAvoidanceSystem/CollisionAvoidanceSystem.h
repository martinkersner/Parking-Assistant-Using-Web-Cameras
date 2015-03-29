#ifndef COLLISIONAVOIDANCESYSTEM_COLLISIONAVOIDANCESYSTEM_H_
#define COLLISIONAVOIDANCESYSTEM_COLLISIONAVOIDANCESYSTEM_H_

#include "Undistortion.h"
#include "DisparityMap.h"
#include "Triangulation.h"
#include "ObjectDetection.h"

class CollisionAvoidanceSystem {
    std::string extrinsics,
                distortions;

    cv::Mat background;

    Undistortion undistortion;
    DisparityMap disparityMap;
    Triangulation triangulation;
    ObjectDetection objectDetection;

    public:
        CollisionAvoidanceSystem( std::string _extrinsics,
                                  std::string _distortions,
                                 cv::Mat _background );

        Object Detect( cv::Mat & left,
                       cv::Mat & right );
};

#endif // COLLISIONAVOIDANCESYSTEM_COLLISIONAVOIDANCESYSTEM_H_
