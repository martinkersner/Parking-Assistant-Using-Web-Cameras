/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Stereo Calibrating of Horizontally aligned camera
 *
 * m.kersner@gmail.com
 * 01/25/2015
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

typedef std::vector<std::string> StringPair; // left image, right image

class CalibrateAndRectify {

    // camera intrinsic properties
    cv::Mat cameraMatrix[2];
    cv::Mat distCoeffs[2];

    // camera extrinsic properties
    cv::Mat R, T;
    cv::Mat R1, R2, P1, P2, Q;

    // parameters for undistortion and rectification
    cv::Mat rmap[2][2];

    // TODO reimplement!
    //std::vector<std::string > GetFileNames(std::string & path);
    std::vector<StringPair> 
    GetFileNames( std::string & path,
                  int numberFrames );

    void 
    StereoCalibrate( StringPair & sp,
                     cv::Size chessboardSize,
                     std::vector<std::vector<cv::Point2f>> * framePoints );

    cv::Size 
    GetFrameSize( std::vector<StringPair> & vecStringPairs );

    double 
    ComputePartlyError( std::vector<std::vector<cv::Point2f>> * framePoints, 
                        std::vector<cv::Vec3f> * lines, 
                        int npt,
                        int i );

    std::vector<std::vector<cv::Point3f>>
    PrepareObjectPoints( int numberFrames,
                         cv::Size chessboardSize );

    void 
    SaveIntrinsics();

    void
    SaveExtrinsics();

    void 
    SaveDistortionCameraModels();

    bool 
    FindStereoChessboardCorners( cv::Mat frameLeft, 
                                 cv::Mat frameRight, 
                                 cv::Size chessboardSize,
                                 std::vector<std::vector<cv::Point2f>> * framePoints );

    void 
    CalibrateStereoCamera( cv::Size chessboardSize, 
                           cv::Size frameSize,
                           int numberFrames,
                           std::vector<std::vector<cv::Point2f>> * framePoints );

    public:
        // Calibration utility
        // compute intrinsic and extrinsic camera parameters
        // TODO reimplement
        //      find out number of images from given directory containing images
        //CalibrateStereoCamera(std::string pathToImages);
        CalibrateAndRectify(std::string pathToImages, 
                            int numberImages, 
                            cv::Size chessboardSize );

        // Rectification utility
        // load given intrinsic and extrinsic parameters
        //CalibrateAndRectify(std::string intrinsics, std::string extrinsics);
        
        void
        RemapImage( cv::Mat leftImage, 
                    cv::Mat rightImage );
};
