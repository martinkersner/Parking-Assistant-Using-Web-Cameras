#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cmath> 

#include <iostream>
#include <string>
#include <iomanip>

#define M_PI                     3.14159265358979323846
#define CORRECT_FLOW_ESTIMATION  1
#define OCCLUDED                 99999

struct dataset_s {
    std::string name;
    std::vector<std::string> subdatasets;
};

void CalculateSparseFlow ( const cv::Mat & first,
                           const cv::Mat & second,
                           cv::Mat & flow,
                           cv::Mat & mask );

cv::Mat CalculateDenseFlow( const cv::Mat & first,
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

void SparseFlowToMat ( const std::vector<cv::Point2f> & firstFeatures,
                       const std::vector<cv::Point2f> & secondFeatures,
                       const std::vector<uchar> & status,
                       cv::Size & size,
                       cv::Mat & flow,
                       cv::Mat & mask );

inline bool CorrectFlowEstimation( uchar fe );

inline void CalculateParticularFlow( const cv::Point2f & firstFeature,
                                     const cv::Point2f & secondFeature,
                                     cv::Point2f & flow );

float SparseAngularError( const cv::Mat & flow,
                          const cv::Mat & groundTruth,
                          const cv::Mat & mask );

float SparseL1EndpointError( const cv::Mat & flow,
                             const cv::Mat & groundTruth, 
                             const cv::Mat & mask );

float SparseL2EndpointError( const cv::Mat & flow,
                             const cv::Mat & groundTruth, 
                             const cv::Mat & mask );
