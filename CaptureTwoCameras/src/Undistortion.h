#include <string>
#include <opencv2/opencv.hpp>

class Undistortion {
    cv::Mat rmap00,
            rmap01,
            rmap10,
            rmap11;

    void LoadDistortion( std::string fileName,
                         cv::Mat & rmap00,
                         cv::Mat & rmap01,
                         cv::Mat & rmap10,
                         cv::Mat & rmap11 );

    public:
        Undistortion( std::string fileName );
        cv::Mat CorrectLeftImage( cv::Mat & leftImage );
        cv::Mat CorrectRightImage( cv::Mat & rightImage );
};
