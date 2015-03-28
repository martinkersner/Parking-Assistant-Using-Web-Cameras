#ifndef UNDISTORTION_H_
#define UNDISTORTION_H_

#include <string>
#include <opencv2/opencv.hpp>

class Undistortion {
    cv::Mat rmap00,
            rmap01,
            rmap10,
            rmap11;

    void LoadDistortion( std::string fileName );

    public:
        Undistortion( std::string fileName );
        cv::Mat CorrectLeftImage( cv::Mat & leftImage );
        cv::Mat CorrectRightImage( cv::Mat & rightImage );
};

#endif // UNDISTORTION_H_