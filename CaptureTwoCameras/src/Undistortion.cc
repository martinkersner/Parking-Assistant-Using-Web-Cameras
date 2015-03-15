/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Correction of distorted images.
 *
 * m.kersner@gmail.com
 * 03/15/2015
 */

#include "Undistortion.h"

Undistortion::Undistortion( std::string fileName ) {
    LoadDistortion( fileName, 
                    this->rmap00, this->rmap01, this->rmap10, this->rmap11 );
}

void Undistortion::LoadDistortion( std::string fileName,
                                   cv::Mat & rmap00,
                                   cv::Mat & rmap01,
                                   cv::Mat & rmap10,
                                   cv::Mat & rmap11 )
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["MX1"] >> rmap00;
        fs["MY1"] >> rmap01;
        fs["MX2"] >> rmap10;
        fs["MY2"] >> rmap11;

        fs.release();
    }
}

cv::Mat Undistortion::CorrectLeftImage( cv::Mat & leftImage )
{
    cv::Mat leftImageCorrected;

    remap( leftImage, leftImageCorrected, 
           this->rmap00, this->rmap01, 
           cv::INTER_LINEAR );

    return leftImageCorrected;
}

cv::Mat Undistortion::CorrectRightImage( cv::Mat & rightImage )
{
    cv::Mat rightImageCorrected;

    remap( rightImage, rightImageCorrected, 
           this->rmap10, this->rmap11, 
           cv::INTER_LINEAR );

    return rightImageCorrected;
}
