#ifndef CURBDETECTION_CURBDETECTION_H_ 
#define CURBDETECTION_CURBDETECTION_H_

#include <opencv2/opencv.hpp>

typedef std::vector<std::vector<cv::Point> > VecVecPoint;
typedef std::vector<cv::Point> VecPoint;
typedef std::vector<int> VecInt;

struct Curb {
    bool found = false;
    cv::Mat input;
    cv::Mat output;
    VecVecPoint curb;
};

class CurbDetection {
    // median blur
    int medianKernelSize      = 11;
    int repeatBlur            = 3;

    // canny properties
    int cannyLowThreshold     = 20;
    int cannyRatio            = 3;
    int cannyKernelSize       = 3;

    // heuristics settings
    float maxAngleDiff        = 10;
    float maxMergeDist        = 12;

    // hough lines detection
    int houghThreshold        = 150;
    double houghMinLineLength = 50;
    double houghMaxLineGap    = 150;

    int resizeFactor          = 6;
    static const float NO_ANGLE;

    bool demo; // decides whether show result of detection

    void BlurImage( cv::Mat & src, cv::Mat & dst );
    void DetectEdges( cv::Mat & src, cv::Mat & dst );
    bool AllowedDistance( VecPoint l0, VecPoint l1 );
    VecPoint Merge( VecVecPoint lines );
    bool FindIntInVector( VecInt v, int i );
    VecPoint ControlMerge( VecVecPoint & lines, int index, VecInt & usedIndices );
    VecVecPoint MergeSimilarLines( VecVecPoint & lines );
    VecPoint Vec4iLinesToPointLines ( const cv::Vec4i & vl, const cv::Mat & src );
    VecPoint HighestLine( const VecVecPoint & lines);
    void ThreeLowestLines( const VecVecPoint & lines,
                           VecPoint & first,
                           VecPoint & second,
                           VecPoint & third );
    bool IsAscending( const VecPoint & line );
    float GetHypotenuse( const VecPoint & line );
    float GetAdjacentCathetus( const VecPoint & line );
    float GetLineAngle( VecPoint & line );
    bool EqualLines ( const VecPoint l0, const VecPoint l1 );
    bool AbsoluteComparison( float f0, float f1, float degreeOfFreedom );
    VecVecPoint FindCurb( const VecVecPoint & lines );
    VecVecPoint DetectCurb( const cv::Mat & src, cv::Mat & dst );
    void RepeatBlurImage( const cv::Mat & src, cv::Mat & dst, int repetition );
    cv::Mat DrawCurb( cv::Mat & src, VecVecPoint & curbing );
    
    // auxiliary method
    cv::Mat DownSample(const cv::Mat src, double factor);

    public:
        CurbDetection( bool _demo );
        ~CurbDetection();
        Curb Detect( cv::Mat & src );
};

#endif // CURBDETECTION_CURBDETECTION_H_
