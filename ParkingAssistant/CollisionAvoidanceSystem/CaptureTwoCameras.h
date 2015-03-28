#include <opencv2/opencv.hpp>
#include <string>

enum CaptureState {INITIALIZED, RUNNING, PAUSE};

class CaptureTwoCameras {
    cv::Mat RemapLeftImage( cv::Mat leftImage );
    cv::Mat RemapRightImage( cv::Mat rightImage ); 
    cv::VideoCapture OpenCamera( int index );
    void LoadDistortionCameraModels( std::string fileName );
    inline void SkipFirstFrames();

    int indexLeft, indexRight;
    cv::VideoCapture capLeft, capRight;
    cv::Mat rmap[2][2];  // parameters for undistortion and rectification
    std::string outDir;

public:
    CaptureTwoCameras( int indexLeft, 
                       int indexRight,
                       std::string fileName,
                       std::string outDir );
    cv::Mat ReadLeftAndRepair();
    cv::Mat ReadRightAndRepair();
    cv::Mat ReadLeftRepairSave( int index );
    cv::Mat ReadRightRepairSave( int index );
};
