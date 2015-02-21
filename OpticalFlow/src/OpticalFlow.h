#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cmath> 

#include <iostream>
#include <string>
#include <iomanip>

# define M_PI   3.14159265358979323846

struct dataset_s {
    std::string name;
    std::vector<std::string> subdatasets;
};

cv::Mat CalculateFlow( const cv::Mat & first,
                       const cv::Mat & second );

void DrawOpticalFlow( const cv::Mat & flowX, 
                      const cv::Mat & flowY, 
                      cv::Mat & img, 
                      int step, 
                      const cv::Scalar & color );

void DrawOpticalFlow( const cv::Mat & flow, 
                      cv::Mat & img, 
                      int step, 
                      const cv::Scalar & color );

cv::Mat DrawOpticalFlowColor( const cv::Mat & flowX, 
                              const cv::Mat & flowY );

cv::Mat DrawOpticalFlowColor( const cv::Mat & flow );

cv::Mat AngularError( cv::Mat & flow,
                      cv::Mat & groundTruth );

float MeanAngularError( cv::Mat & flow,
                        cv::Mat & groundTruth );

cv::Mat L1EndpointError( cv::Mat & flow,
                         cv::Mat & groundTruth );

float MeanL1EndpointError( cv::Mat & flow,
                           cv::Mat & groundTruth );

cv::Mat L2EndpointError( cv::Mat & flow,
                         cv::Mat & groundTruth );

float MeanL2EndpointError( cv::Mat & flow,
                             cv::Mat & groundTruth );

double MultVec3x1( cv::Point p1,
                  cv::Point p2 );

double MagVec3x1( cv::Point p );
