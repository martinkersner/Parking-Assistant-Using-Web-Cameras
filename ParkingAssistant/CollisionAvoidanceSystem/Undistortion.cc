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

// this constructor is created only in order to posses class without initialization
Undistortion::Undistortion() 
{}

Undistortion::Undistortion( std::string fileName ) 
{
    LoadDistortion( fileName );
}

void Undistortion::LoadDistortion( std::string fileName )
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["MX1"] >> this->rmap00;
        fs["MY1"] >> this->rmap01;
        fs["MX2"] >> this->rmap10;
        fs["MY2"] >> this->rmap11;

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
