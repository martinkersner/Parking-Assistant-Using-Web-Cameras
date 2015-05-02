/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Feature extraction of detected objects for purpose of human detection.
 *
 * m.kersner@gmail.com
 * 05/01/2015
 */

#include "HumanDetection.h"

HumanDetection::HumanDetection()
{}

HumanDetection::~HumanDetection()
{}

std::vector<int> HumanDetection::ExtractFeatures(cv::Mat & src)
{
    std::vector<int> features;
    cv::Mat cloneSrc = src.clone();
    cv::Mat cropSrc;

    // normalization
    cv::Rect bb = FindObject(src);
    cropSrc = cloneSrc(bb);
    cv::Mat normSrc = Normalize(cropSrc);

    // distance transform
    cv::Mat skelSrc = DistanceTransform(normSrc);

    // feature extraction
    features = FindLineMaximum(skelSrc);

    return features;
}

cv::Rect HumanDetection::FindObject(cv::Mat & src)
{
    cv::Rect bb;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    double area, largestArea;
    int largestContourIdx;

   findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

   // finding largest area
   // assumes there is at least one object in input image
   for (int i = 0; i< contours.size(); ++i)
   {
       area = cv::contourArea(contours[i], false);

       if (area > largestArea) {
           largestArea = area;
           largestContourIdx = i;
           bb = cv::boundingRect(contours[i]);
       }
   }

   return bb;
}

cv::Mat HumanDetection::Normalize(cv::Mat & src)
{
    cv::Mat dst;
    float factor = float(this->height)/src.rows;

    // height normalization
    cv::resize(src, dst, cv::Size(0, 0), factor, factor);

    // empty block
    float blockWidth = this->width - dst.cols;

    // width normalization
    cv::Mat leftBlock  = cv::Mat::zeros(this->height, floor(blockWidth/2), CV_8UC1);
    cv::Mat rightBlock = cv::Mat::zeros(this->height, ceil(blockWidth/2), CV_8UC1);

    // concatenation 
    cv::hconcat(leftBlock, dst, dst);
    cv::hconcat(dst, rightBlock, dst);

    // create clear mask
    dst = dst > 0;

    return dst;
}

cv::Mat HumanDetection::DistanceTransform(cv::Mat & src)
{
    cv::Mat dst;

    cv::distanceTransform(src, dst, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::normalize(dst, dst, 0, 1., cv::NORM_MINMAX);

    return dst;
}

std::vector<int> HumanDetection::FindLineMaximum(cv::Mat & src)
{
    std::vector<int> lineMax;
    cv::Mat row, bwRow;
    double max;
    int index;

    // finding maximum values in each line
    for (int i = 0; i < src.rows; ++i) {
       row = src.row(i);
       cv::minMaxLoc(row, NULL, &max);
       bwRow = row >= max;
       index = FindCenter(bwRow);
       lineMax.push_back(index);
    }

    return lineMax;
}

int HumanDetection::FindCenter(cv::Mat & bwRow)
{
    int index, center, centerIdx;
    std::vector<int> indices;

    for (int i = 0; i < bwRow.cols; ++i) {
       if (int(bwRow.at<uchar>(0,i)) != 0) {
            indices.push_back(i);
        }
    }

    switch (indices.size()) {
        case 0:
            centerIdx = floor(bwRow.cols/2);
            break;

        case 1:
            centerIdx = indices.at(0);
            break;

        default:
            center = floor(float(indices.size())/2);
            centerIdx = floor(indices.at(center));
            break;
    }

    return centerIdx;
}
