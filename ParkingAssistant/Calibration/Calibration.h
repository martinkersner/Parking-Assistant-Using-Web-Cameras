#ifndef CALIBRATION_CALIBRATION_H_
#define CALIBRATION_CALIBRATION_H_

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

class Calibration {

    // calibration properties
    cv::Size chessboardSize;
    int numberImagesForCalibration;
    std::string chessboardPath;
    cv::Size frameSize;
    int numberCalibrated = 0;

    // 2D points in image plane
    std::vector<std::vector<cv::Point2f>> imagePoints[2]; 

    // calibration output files
    std::string intrinsicsFile;
    std::string extrinsicsFile;
    std::string distortionFile;

    // camera intrinsic properties
    cv::Mat cameraMatrix[2];
    cv::Mat distCoeffs[2];

    // camera extrinsic properties
    cv::Mat R, T;
    cv::Mat R1, R2, P1, P2, Q;

    // parameters for undistortion and rectification
    cv::Mat rmap[2][2];

    std::vector<std::vector<cv::Point3f>> PrepareObjectPoints();

    void SaveIntrinsics();

    void SaveExtrinsics();

    void SaveDistortionCameraModels();

    bool FindStereoChessboardCorners( cv::Mat imageLeft, 
                                      cv::Mat imageRight );

    void CalibrateStereoCamera();

    void SaveImages( cv::Mat leftImage,
                     cv::Mat rightImage );

    void ClearImagePoints();

    public:
        Calibration( int rows,
                     int cols,
                     int _numberImagesForCalibration,
                     std::string _chessboardPath,
                     cv::Size _frameSize,
                     std::string _intrinsicsFile,
                     std::string _extrinsicsFile,
                     std::string _distortionFile );

        int Calibrate( cv::Mat & left,
                       cv::Mat & right );
};

#endif // CALIBRATION_CALIBRATION_H_
