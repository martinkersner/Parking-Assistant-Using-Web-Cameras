/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Sum of Squared Differences
 * Sum of Absolute Differences
 *
 * m.kersner@gmail.com
 * 01/28/2015
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <limits>

cv::Mat SAD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window );

cv::Mat SSD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window );

cv::Mat translateImg( cv::Mat &img, 
                      int offsetx, 
                      int offsety );

cv::Mat CreateLookupTable( int threshold );

cv::Mat ComputeDisparity( cv::Mat & left,
                          cv::Mat & right,
                          int windowSize,
                          int maxShift );

int main() {
    cv::Mat left = cv::imread("../img/bowling_left.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat right = cv::imread("../img/bowling_right.png", CV_LOAD_IMAGE_GRAYSCALE);

    //cv::Mat left = cv::imread("../img/tsukuba_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat right = cv::imread("../img/tsukuba_r.png", CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat disp;
    disp = ComputeDisparity (left, right, 6, 40);

    equalizeHist(disp, disp);
    cv::imwrite("disparity.png", disp);
}

cv::Mat SAD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window )
{
    cv::Mat diff, sad;

    cv::absdiff(left, right, diff);
    cv::filter2D(diff, sad, -1, window);

    return sad;
}

cv::Mat SSD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window )
{
    cv::Mat diff, square, ssd;

    cv::subtract(left, right, diff);
    cv::pow(diff, 2, square);
    cv::filter2D(square, ssd, -1, window);

    return ssd;
}

cv::Mat ComputeDisparity( cv::Mat & left,
                          cv::Mat & right,
                          int windowSize,
                          int maxShift )
{
    int defaultIntensity = std::numeric_limits<int>::max();
    cv::Mat sxd, sxdMask, sxdMask2, M;

    cv::Mat window = cv::Mat::ones(windowSize, windowSize, CV_8UC1);
    cv::Mat disparity = cv::Mat::zeros(left.rows, left.cols, CV_8UC1);
    cv::Mat minSXD    = cv::Mat(left.rows, left.cols, CV_8UC1, cv::Scalar(defaultIntensity));

    for (int i = 1; i <= maxShift; ++i) {
        translateImg(right, 1, 0);
        sxd = SAD(left, right, window);
        //sxd = SSD(left, right, window);

        sxdMask = (sxd < minSXD)/255;
        sxdMask2 = (sxd >= minSXD)/255;
        
        minSXD     = minSXD.mul(sxdMask2) + sxd.mul(sxdMask);
        disparity  = disparity.mul(sxdMask2) + sxdMask * i;
    }

    return disparity;
}

cv::Mat translateImg( cv::Mat &img, 
                      int offsetx, 
                      int offsety )
{
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    cv::warpAffine(img,img,trans_mat,img.size());
    return trans_mat;
}

// x <= threshold        0
// x >  threshold        1
cv::Mat CreateLookupTable ( int threshold )
{
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.data;

    for(int i = 0; i < 256; ++i)
        if (i <= threshold)
            p[i] = 0;
        else
            p[i] = 1;
}
