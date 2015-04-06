/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Acquires images from connected webcam. 
 *
 * m.kersner@gmail.com
 * 03/28/2015
 */

#include "ImageAcquisition.h"

ImageAcquisition::ImageAcquisition ( int _indexLeft, 
                                     int _indexRight ) 
    : indexLeft(_indexLeft),                                                             
      indexRight(_indexRight)

{
    this->capLeft = OpenCamera(this->indexLeft);
    this->capRight= OpenCamera(this->indexRight);
}

int ImageAcquisition::VerifyConnection()
{
    if (!this->capLeft.isOpened() || !this->capRight.isOpened()) {
        std::cerr  << "Cannot read from webcam!" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

cv::VideoCapture ImageAcquisition::OpenCamera( int index )
{
    cv::VideoCapture cap(index); 
    cap.set(CV_CAP_PROP_FRAME_WIDTH,  640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    // OpenCV seems to ignore fps setup
    cap.set(CV_CAP_PROP_FPS , 30);

    // TODO new OpenCV interface
    //cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //cap.set(cv::CAP_PROP_FPS , 30);
    
    return cap;
}

void ImageAcquisition::SkipFirstFrames()
{
    cv::Mat tmpLeft, tmpRight;

    this->capLeft  >> tmpLeft;
    this->capRight >> tmpRight;
}

void ImageAcquisition::ReadLeftAndRight( cv::Mat & left,
                                         cv::Mat & right )
{
    this->capLeft  >> left;
    this->capRight >> right;
}
