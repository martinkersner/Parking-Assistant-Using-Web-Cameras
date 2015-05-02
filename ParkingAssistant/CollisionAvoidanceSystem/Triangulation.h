#ifndef COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_
#define COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_

#include <string>
#include <opencv2/opencv.hpp>

class Triangulation {
    cv::Mat Q;  // reprojection matrix
    double Q03, Q13, Q23, Q32, Q33;
    float squareSize;
    std::string extrinsicsFile;

    void LoadExtrinsics( std::string fileName );
    void PrepareProjectionMatrix();

    public:
        Triangulation();
        Triangulation( std::string fileName,
                       float _sqaureSize );
        cv::Mat DisparityToDepth( cv::Mat & disparity );
        void ReloadExtrinsics();
};

#endif // COLLISIONAVOIDANCESYSTEM_TRIANGULATION_H_
