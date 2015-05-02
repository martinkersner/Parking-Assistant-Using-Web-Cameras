#ifndef HUMANDETECTION_HUMANDETECTION_H_
#define HUMANDETECTION_HUMANDETECTION_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class HumanDetection {
    cv::Mat Normalize(cv::Mat & src);
    cv::Mat DistanceTransform(cv::Mat & src);
    std::vector<int> FindLineMaximum(cv::Mat & src);
    cv::Rect FindObject(cv::Mat & src);
    int FindCenter(cv::Mat & bwRow);

    int width  = 656;
    int height = 450;

    public:
        HumanDetection();
        ~HumanDetection();
        std::vector<int> ExtractFeatures(cv::Mat & src);
};

#endif // HUMANDETECTION_HUMANDETECTION_H_
