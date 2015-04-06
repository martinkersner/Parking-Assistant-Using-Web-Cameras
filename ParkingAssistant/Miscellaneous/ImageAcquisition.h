#ifndef MISCELLANEOUS_IMAGEACQUISITION_H_
#define MISCELLANEOUS_IMAGEACQUISITION_H_

#include <opencv2/opencv.hpp>

class ImageAcquisition {

    int indexLeft,
        indexRight;

    int imageWidth,
        imageHeight;

    int fps;

    cv::VideoCapture capLeft,
                     capRight;

    cv::VideoCapture OpenCamera( int index );

    public:
        ImageAcquisition ( int _indexLeft, 
                           int _indexRight,
                           int _imageWidth,
                           int _imageHeight,
                           int _fps );

        ~ImageAcquisition();

        int VerifyConnection();

        void SkipFirstFrames();

        void ReadLeftAndRight( cv::Mat & left, 
                               cv::Mat & right );
};

#endif // MISCELLANEOUS_IMAGEACQUISITION_H_
