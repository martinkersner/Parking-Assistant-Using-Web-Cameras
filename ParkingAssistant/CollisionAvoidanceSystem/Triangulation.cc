/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Triangulation of disparity map.
 *
 * m.kersner@gmail.com
 * 03/28/2015
 */

#include "Triangulation.h"

// this constructor is created only in order to posses class without initialization
Triangulation::Triangulation()
{}

Triangulation::Triangulation( std::string fileName,
                              float _squareSize ) : squareSize(_squareSize)
{
    LoadExtrinsics(fileName);
    PrepareProjectionMatrix();
}

void Triangulation::LoadExtrinsics( std::string fileName ) 
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ); 

    if (fs.isOpened()) {
        fs["Q"]  >> this->Q;
    }
}

void Triangulation::PrepareProjectionMatrix()
{
    this->Q03 = this->Q.at<double>(0,3);
    this->Q13 = this->Q.at<double>(1,3);
    this->Q23 = this->Q.at<double>(2,3);
    this->Q32 = this->Q.at<double>(3,2);
    this->Q33 = this->Q.at<double>(3,3);
}

cv::Mat Triangulation::DisparityToDepth( cv::Mat & disparity )
{
    // zero values mean unknown disparities
    cv::Mat depth = cv::Mat::zeros(disparity.size(), CV_64FC1);
    double w;

    for (int i = 0; i < disparity.rows; ++i) {
        uchar* disp_ptr = disparity.ptr<uchar>(i);

        for (int j = 0; j < disparity.cols; ++j) {
            uchar d = disp_ptr[j];

            // discard bad pixels
            if (d == 0) 
                continue; 

            w = static_cast<double>(d) * this->Q32 + this->Q33; 
            depth.at<double>(i, j) = (this->Q23/w)/this->squareSize;
        }
    }

    return depth;
}
