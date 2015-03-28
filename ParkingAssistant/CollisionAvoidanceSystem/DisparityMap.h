#ifndef COLLISIONAVOIDANCESYSTEM_DISPARITYMAP_H_
#define COLLISIONAVOIDANCESYSTEM_DISPARITYMAP_H_

#include <opencv2/opencv.hpp>

class DisparityMap {
    cv::StereoBM sbm;

    void InitializeStereoProperties();

    public:
        DisparityMap();
        void SetPreFilterSize( int value );
        void SetPreFilterCap( int value );
        void SetSATWindowSize( int value );
        void SetMinDisparity( int value );
        void SetNumberOfDisparities( int value );
        void SetTextureThreshold( int value );
        void SetUniquenessRatio( int value );
        void SetSpeckleWindowSize( int value );
        void SetSpeckleRange( int value );
        cv::Mat CalculateDisparity( cv::Mat & left,
                                    cv::Mat & right );
};

#endif // COLLISIONAVOIDANCESYSTEM_DISPARITYMAP_H_
