/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Stereo Calibrating of horizontally aligned cameras
 *
 * m.kersner@gmail.com
 * 01/24/2015
 */

#include "Calibration.h"

Calibration::Calibration( int rows,
                          int cols,
                          int _numberImagesForCalibration,
                          std::string _chessboardPath,
                          cv::Size _frameSize,
                          std::string _intrinsicsFile,
                          std::string _extrinsicsFile,
                          std::string _distortionFile ) 
    : numberImagesForCalibration(_numberImagesForCalibration),
      chessboardPath(_chessboardPath),
      frameSize(_frameSize),
      intrinsicsFile(_intrinsicsFile),
      extrinsicsFile(_extrinsicsFile),
      distortionFile(_distortionFile)
{
    this->chessboardSize = cv::Size(rows, cols);
}

int Calibration::Calibrate( cv::Mat & left,
                            cv::Mat & right )
{
    bool valid = FindStereoChessboardCorners( left, right );

    if (valid) {
        SaveImages(left, right);

        std::cout << this->numberCalibrated << std::endl;

        this->numberCalibrated++;

        if (this->numberCalibrated >= this->numberImagesForCalibration) {
            CalibrateStereoCamera();

            ClearImagePoints();
            this->numberCalibrated = 0;
            return EXIT_SUCCESS; // calibration finished
        }
    }

    // calibration is still ongoing
    return EXIT_FAILURE;
}

bool Calibration::FindStereoChessboardCorners( cv::Mat imageLeft, 
                                               cv::Mat imageRight )
{
    bool foundLeft  = false;
    bool foundRight = false;

    cv::cvtColor(imageLeft,  imageLeft,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(imageRight, imageRight, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> cornersLeft;
    std::vector<cv::Point2f> cornersRight;

    int flags = cv::CALIB_CB_ADAPTIVE_THRESH +
                cv::CALIB_CB_NORMALIZE_IMAGE;
                //cv::CALIB_CB_FAST_CHECK;

    foundLeft = cv::findChessboardCorners( imageLeft, 
                                           this->chessboardSize, 
                                           cornersLeft, 
                                           flags );

    foundRight = cv::findChessboardCorners( imageRight, 
                                            this->chessboardSize, 
                                            cornersRight, 
                                            flags );
    // compute corners more accurately
    if (foundLeft)
        cornerSubPix( imageLeft,
                      cornersLeft, 
                      cv::Size(11,11), cv::Size(-1,-1), 
                      cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 
                                        30, 
                                        0.1 ));

    if (foundRight)
        cornerSubPix( imageRight,
                      cornersRight, 
                      cv::Size(11,11), cv::Size(-1,-1), 
                      cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 
                                        30, 
                                        0.1 ));

    if (foundLeft && foundRight) {
        // store found corners only if it has been found at both of examined images
        this->imagePoints[0].push_back(cornersLeft);
        this->imagePoints[1].push_back(cornersRight);

        return true;
    }
    else
        return false;
}

void Calibration::SaveIntrinsics() 
{
    cv::FileStorage fs(this->intrinsicsFile, cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "M1" << this->cameraMatrix[0]
           << "D1" << this->distCoeffs[0]
           << "M2" << this->cameraMatrix[1]
           << "D2" << this->distCoeffs[1];

        fs.release();
    }
}

void Calibration::SaveExtrinsics()
{
    cv::FileStorage fs(this->extrinsicsFile, cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "R"  << this->R 
           << "T"  << this->T 
           << "R1" << this->R1 
           << "R2" << this->R2 
           << "P1" << this->P1 
           << "P2" << this->P2 
           << "Q"  << this->Q;

        fs.release();
    }
}

void Calibration::SaveDistortionCameraModels()
{
    cv::FileStorage fs(this->distortionFile, cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "MX1"  << this->rmap[0][0]
           << "MY1"  << this->rmap[0][1]
           << "MX2"  << this->rmap[1][0]
           << "MY2"  << this->rmap[1][1];

        fs.release();
    }
}

std::vector<std::vector<cv::Point3f>> Calibration::PrepareObjectPoints()
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    objectPoints.resize(this->numberImagesForCalibration);

    // prepare object points
    // like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    for (int i = 0; i < this->numberImagesForCalibration; i++) {
        for (int j = 0; j < this->chessboardSize.height; j++) {
            for (int k = 0; k < this->chessboardSize.width; k++) {

                // multiply by square size?
                objectPoints[i].push_back(cv::Point3f( float(j), 
                                                       float(k),
                                                       0.0       ));

            }
        }
    }

    return objectPoints;
}

void Calibration::CalibrateStereoCamera()
{
    // 3D points in real world space
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // we assume that corners were found in each pair of images
    objectPoints = PrepareObjectPoints();

    //cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat E, F;
    double reprErr;

    reprErr = cv::stereoCalibrate( objectPoints, 
                                   this->imagePoints[0], this->imagePoints[1],
                                   this->cameraMatrix[0], this->distCoeffs[0],
                                   this->cameraMatrix[1], this->distCoeffs[1],
                                   this->frameSize, 
                                   this->R, this->T, 
                                   E, F,
                                   cv::TermCriteria( cv::TermCriteria::COUNT + 
                                                     cv::TermCriteria::EPS, 
                                                     100, 
                                                     1e-5 ),
                                   cv::CALIB_FIX_ASPECT_RATIO +
                                   cv::CALIB_ZERO_TANGENT_DIST +
                                   cv::CALIB_SAME_FOCAL_LENGTH +
                                   cv::CALIB_RATIONAL_MODEL +
                                   cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);

    std::vector<cv::Vec3f> lines[2];

    for (int i = 0; i < this->numberImagesForCalibration; i++) {

        cv::Mat imgpt[2];

        for (int k = 0; k < 2; k++) {
            imgpt[k] = cv::Mat(this->imagePoints[k][i]);

            undistortPoints( imgpt[k], imgpt[k], 
                             this->cameraMatrix[k], 
                             this->distCoeffs[k], 
                             cv::Mat(), 
                             this->cameraMatrix[k]);

            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
    }

    SaveIntrinsics();
    
    // EXTRINSICS
    cv::Rect validROI[2];
    stereoRectify( this->cameraMatrix[0], this->distCoeffs[0], 
                   this->cameraMatrix[1], this->distCoeffs[1], 
                   this->frameSize, 
                   this->R, this->T, 
                   this->R1, this->R2, 
                   this->P1, this->P2, 
                   this->Q, 
                   cv::CALIB_ZERO_DISPARITY, 
                   1, 
                   this->frameSize, 
                   &validROI[0], &validROI[1]);

    SaveExtrinsics();

    // lower is better
    std::cout << "Reprojection Error: " << reprErr << std::endl;

    // DISTORTION
    initUndistortRectifyMap( this->cameraMatrix[0], 
                             this->distCoeffs[0], 
                             this->R1, this->P1, 
                             this->frameSize, 
                             CV_16SC2, 
                             this->rmap[0][0], this->rmap[0][1] );

    initUndistortRectifyMap( this->cameraMatrix[1], 
                             this->distCoeffs[1], 
                             this->R2, this->P2, 
                             this->frameSize, 
                             CV_16SC2,
                             this->rmap[1][0], this->rmap[1][1] );

     SaveDistortionCameraModels();
}

/**
 * Backup of captured chessboards.
 */
void Calibration::SaveImages( cv::Mat leftImage,
                              cv::Mat rightImage )
{
    std::string leftName  = this->chessboardPath 
                            + std::to_string(this->numberCalibrated) + "-left.png";
    std::string rightName = this->chessboardPath 
                            + std::to_string(this->numberCalibrated) + "-right.png";

    cv::imwrite(leftName,  leftImage);
    cv::imwrite(rightName, rightImage);
}

/**
 * Clear image points after calibration is done.
 * This step is necessary preceding process preparing for another calibration
 */
void Calibration::ClearImagePoints()
{
    for (int i = 0; i < this->imagePoints[0].size(); ++i)    
        this->imagePoints[0].at(i).clear();

    for (int i = 0; i < this->imagePoints[1].size(); ++i)    
        this->imagePoints[1].at(i).clear();

    this->imagePoints[0].clear();
    this->imagePoints[1].clear();
}
