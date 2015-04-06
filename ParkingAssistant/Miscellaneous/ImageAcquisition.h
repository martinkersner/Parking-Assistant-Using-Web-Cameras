#ifndef MISCELLANEOUS_IMAGEACQUISITION_H_
#define MISCELLANEOUS_IMAGEACQUISITION_H_

#include <opencv2/opencv.hpp>

class ImageAcquisition {

    int indexLeft,
        indexRight;

    cv::VideoCapture capLeft,
                     capRight;

    cv::VideoCapture OpenCamera( int index );

    public:
        ImageAcquisition( int _indexLeft,
                          int _indexRight );

        int VerifyConnection();

        void SkipFirstFrames();

        void ReadLeftAndRight( cv::Mat & left, 
                               cv::Mat & right );
};

#endif // MISCELLANEOUS_IMAGEACQUISITION_H_
