#ifndef COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_
#define COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_

#include <string>
#include <opencv2/opencv.hpp>

class Triangulation {
    cv::Mat Q;  // reprojection matrix
    double Q03, Q13, Q23, Q32, Q33;

    void LoadExtrinsics( std::string fileName );
    void PrepareProjectionMatrix();

    public:
        Triangulation( std::string fileName );
        cv::Mat DisparityToDepth( cv::Mat & disparity );
        cv::Mat DisparityToDepth2( cv::Mat & disparity );
};

#endif // COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_
