#ifndef COLLISIONAVOIDANCESYSTEM_OBJECTDETECTION_H_
#define COLLISIONAVOIDANCESYSTEM_OBJECTDETECTION_H_

#include <opencv2/opencv.hpp>

struct Object {
    bool found = false;
    cv::Mat mask;
    int distance = 0;
};

class ObjectDetection {
    cv::Mat background;

    // threshold addresses to minimal allowed difference after background subtraction 
    int threshold = 20; 

    // offset of histogram
    int latestOffset = 0;

    // histogram properties
    int numberValues = 256;
    int binWidth = 5;
    int histSize = numberValues/binWidth; 

    // histogram window properties
    int histWidth  = 512; 
    int histHeight = 400;

    // detection properties
    int minBase = 3100;

    cv::Mat ComputeHistogram( cv::Mat & src );

    cv::Mat HistogramPostprocessing ( cv::Mat & hist );

    Object ClosestObject ( cv::Mat & hist,
                           cv::Mat & img );

    int BinId2Intensity( int binId );

    void ClearHistogram( cv::Mat & hist,
                         int offset );

    float ObjectDistance( cv::Mat & hist,
                          int lowerBin,
                          int upperBin );

    cv::Mat ModifyHistogram( cv::Mat & hist );

    cv::Mat GetObject( cv::Mat & src, 
                       float lower,
                       float upper );

    // auxiliary methods for debugging
    void DisplayHistogram( cv::Mat & hist );

    public:
        ObjectDetection();
        ObjectDetection( cv::Mat & _background );
        Object Detect( cv::Mat & scene );
};

#endif // COLLISIONAVOIDANCESYSTEM_OBJECTDETECTION_H_
