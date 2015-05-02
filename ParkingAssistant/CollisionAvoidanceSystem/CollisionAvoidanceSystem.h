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

    float squareSize;
    bool demo;

    Undistortion undistortion;
    DisparityMap disparityMap;
    Triangulation triangulation;
    ObjectDetection objectDetection;

    public:
        CollisionAvoidanceSystem( std::string _extrinsics,
                                  std::string _distortions,
                                  cv::Mat _background,
                                  float _squareSize,
                                  bool _demo=false );

        void Initialize();

        Object Detect( cv::Mat & left,
                       cv::Mat & right );

        float Detect( cv::Mat & left,
                      cv::Mat & right,
                      cv::Mat & mask );

        void SetNumberOfDisparities( int disparityNumber );

        void ReinitializeCalibrationFiles();
};

#endif // COLLISIONAVOIDANCESYSTEM_COLLISIONAVOIDANCESYSTEM_H_
