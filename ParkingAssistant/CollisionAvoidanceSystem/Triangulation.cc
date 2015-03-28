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

Triangulation::Triangulation( std::string fileName )
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

            w = -1.0 * static_cast<double>(d) * this->Q32 + this->Q33; 
            depth.at<double>(i, j) = this->Q23/w;
        }
    }

    //depth.convertTo(depth, CV_8UC1);
    //cv::medianBlur(depth, depth, 15);

    return depth;
}

// TODO handle zero values of disparity map
cv::Mat Triangulation::DisparityToDepth2( cv::Mat & disparity )
{
    cv::Mat z = cv::Mat(disparity.size(), CV_64FC1, cv::Scalar(this->Q23));
    //cv::Mat mask = (disparity > 0) / 255.0;

    cv::Mat newDisp = cv::Mat(disparity.size(), CV_64FC1);
    disparity.convertTo(newDisp, CV_64FC1);
    cv::Mat w = (-1.0 * newDisp * this->Q32) + this->Q33;
    cv::Mat depth = z/w;

    //depth.copyTo(depth, mask);
    return depth;
}
