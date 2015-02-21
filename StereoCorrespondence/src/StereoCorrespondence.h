#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <iostream>
#include <limits>
#include <string>
#include <cmath>
#include <iomanip>

enum bm_t {NCC_t, SSD_t, NSSD_t, SAD_t, RANK_t, CENSUS_t};

struct dataset_s {
    std::string name;
    std::string imageFormat;
    std::string dispaFormat;
    int factor;
    std::vector<std::string> subdatasets;
};

cv::Mat translateImg( cv::Mat &img, 
                      int offsetx, 
                      int offsety );

cv::Mat ComputeDisparity( cv::Mat & left,
                          cv::Mat & right,
                          int windowSize,
                          int maxDisparity,
                          int factor,
                          bm_t type );

cv::Mat Disparity( cv::Mat & left, 
                   cv::Mat & right, 
                   cv::Mat window, 
                   bm_t type );

// block matching methods
cv::Mat SAD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window );

cv::Mat SSD( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window );

cv::Mat NCC( cv::Mat & left, 
             cv::Mat & right,
             cv::Mat & window );

cv::Mat TransformToRank( cv::Mat & m );

cv::Mat RANK( cv::Mat & left, 
              cv::Mat & right,
              cv::Mat & window );

cv::Mat CENSUS( cv::Mat & left,
                cv::Mat & right );

// quality metrics 
void ControlConstraint( cv::Mat & m1,
                        cv::Mat & m2 );
float RMSE ( cv::Mat & disparity, 
             cv::Mat & disparityGt );

float RMSE ( cv::Mat & disparity, 
             cv::Mat & disparityGt,
             cv::Mat & colorMap );

float BM ( cv::Mat & disparity, 
           cv::Mat & disparityGt,
           float threshold );

float BM ( cv::Mat & disparity, 
           cv::Mat & disparityGt );

// bits
void SetBit( unsigned char & binaryValue,
             bool value, 
             int n );

int Hamming( const unsigned char & b1,
             const unsigned char & b2,
             int s );

cv::Mat CensusTransform( cv::Mat & img );

std::vector<uchar> GetNeighborhood( cv::Mat & img,
                                  int row,
                                  int col );

uchar CensusNeighborhood( std::vector<uchar> neighborhood,
                          uchar center );

// unit tests
bool UnitRankTransform();
bool UnitHamming();

// TODO delete /////////////////////////////////////////////////////////////////////////
cv::Mat CreateLookupTable( int threshold );
