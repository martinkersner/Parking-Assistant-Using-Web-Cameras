/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Stereo Calibrating of Horizontally aligned camera
 *
 * m.kersner@gmail.com
 * 01/24/2015
 */

#include "CalibrateAndRectify.hh"

// TODO application arguments
//      frames -> images

int main(int argc, char ** argv) {

    // TODO replace with application arguments
    std::string path          = "chessboard2/";
    const int numberImages    = 11; // temporary solution

    const float square_size   = 2.5; 
    const int rows            = 9;
    const int cols            = 6;
    cv::Size chessboardSize = cv::Size(rows, cols);

    CalibrateAndRectify cr(path, numberImages, chessboardSize);

    //cv::Mat left = cv::imread("chessboard2/left0.ppm");
    //cr.RemapImage(left, left);

    return EXIT_SUCCESS;
}

CalibrateAndRectify::
CalibrateAndRectify(std::string pathToImages, int numberImages, cv::Size chessboardSize)
{
    // 2D points in image plane
    std::vector<std::vector<cv::Point2f>> imagePoints[2]; 

    std::vector<StringPair> vecStringPairs = GetFileNames(pathToImages, numberImages);

    cv::Size imageSize = GetFrameSize(vecStringPairs);

    int i = 0;
    for (StringPair sp : vecStringPairs) {
        StereoCalibrate(sp, chessboardSize, imagePoints);
        std::cout << ++i << std::endl;
    }

    CalibrateStereoCamera( chessboardSize, 
                           imageSize,
                           numberImages,
                           imagePoints );
}

std::vector<StringPair> CalibrateAndRectify::
GetFileNames( std::string & path, 
              int numberFrames ) 
{
    std::vector<StringPair> vecStringPairs;

    for (int i = 0; i < numberFrames; ++i) {
        StringPair sp; // {left_image, right_image}
        //sp.push_back(path + std::to_string(i) + "_left.png");
        //sp.push_back(path + std::to_string(i) + "_right.png");

        sp.push_back(path + "left"  + std::to_string(i) + ".ppm");
        sp.push_back(path + "right" + std::to_string(i) + ".ppm");
        vecStringPairs.push_back(sp);
    }

    return vecStringPairs;
}

void CalibrateAndRectify::
StereoCalibrate( StringPair & sp,
                 cv::Size chessboardSize,
                 std::vector<std::vector<cv::Point2f>> * framePoints )
{
    cv::Mat frameLeft = cv::imread(sp.at(0));
    cv::Mat frameRight = cv::imread(sp.at(1));

    FindStereoChessboardCorners(frameLeft, frameRight, chessboardSize, framePoints);
}

/**
 * Expects the same size of all images, therefore only the size of the first image 
 * is checked.
 */
cv::Size CalibrateAndRectify::
GetFrameSize( std::vector<StringPair> & vecStringPairs ) 
{
    cv::Mat tmpFrame = cv::imread(vecStringPairs.at(0).at(0));
    return tmpFrame.size();
}

bool CalibrateAndRectify::
FindStereoChessboardCorners( cv::Mat frameLeft, 
                             cv::Mat frameRight, 
                             cv::Size chessboardSize,
                             std::vector<std::vector<cv::Point2f>> * framePoints )
{
    //cv::Mat frameLeftOriginal  = frameLeft;
    //cv::Mat frameRightOriginal = frameRight;

    bool foundLeft  = false;
    bool foundRight = false;

    cv::cvtColor(frameLeft,  frameLeft,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(frameRight, frameRight, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> cornersLeft;
    std::vector<cv::Point2f> cornersRight;

    int flags = cv::CALIB_CB_ADAPTIVE_THRESH +
                cv::CALIB_CB_NORMALIZE_IMAGE;
                //cv::CALIB_CB_FAST_CHECK;

    foundLeft = cv::findChessboardCorners( frameLeft, 
                                           chessboardSize, 
                                           cornersLeft, 
                                           flags );

    foundRight = cv::findChessboardCorners( frameRight, 
                                            chessboardSize, 
                                            cornersRight, 
                                            flags );

    // Compute corners more accurately
    if (foundLeft)
        cornerSubPix( frameLeft,
                      cornersLeft, 
                      cv::Size(11,11), cv::Size(-1,-1), 
                      cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 
                                        30, 
                                        0.1 ));

    if (foundRight)
        cornerSubPix( frameRight,
                      cornersRight, 
                      cv::Size(11,11), cv::Size(-1,-1), 
                      cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 
                                        30, 
                                        0.1 ));

    // TODO delete?
    //drawChessboardCorners(frameLeftOriginal, boardSize, cornersLeft, foundLeft);
    //drawChessboardCorners(frameRightOriginal, boardSize, cornersRight, foundRight);
    
    framePoints[0].push_back(cornersLeft);
    framePoints[1].push_back(cornersRight);

    // TODO necessary?
    if (foundLeft && foundRight)
        return true;
    else 
        return false;
}

void CalibrateAndRectify::
SaveIntrinsics() 
{
    cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "M1" << this->cameraMatrix[0]
           << "D1" << this->distCoeffs[0]
           << "M2" << this->cameraMatrix[1]
           << "D2" << this->distCoeffs[1];

        fs.release();
    }
}

void CalibrateAndRectify::
SaveExtrinsics()
{
    cv::FileStorage fs("extrinsics.xml", cv::FileStorage::WRITE);

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

void CalibrateAndRectify::
SaveDistortionCameraModels()
{
    cv::FileStorage fs("distortion_camera_models.xml", cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "MX1"  << this->rmap[0][0]
           << "MY1"  << this->rmap[0][1]
           << "MX2"  << this->rmap[1][0]
           << "MY2"  << this->rmap[1][1];

        fs.release();
    }
}

double CalibrateAndRectify::
ComputePartlyError( std::vector<std::vector<cv::Point2f>> * framePoints, 
                    std::vector<cv::Vec3f> * lines, 
                    int npt,
                    int i )
{
    double err   = 0;
    double errij = 0;

    for(int j = 0; j < npt; j++ ) {
        errij = fabs( framePoints[0][i][j].x*lines[1][j][0] +
                      framePoints[0][i][j].y*lines[1][j][1] + 
                      lines[1][j][2] ) 
                +
                fabs( framePoints[1][i][j].x*lines[0][j][0] +
                      framePoints[1][i][j].y*lines[0][j][1] + 
                      lines[0][j][2] );

        err += errij;
    }

    return err;
}

std::vector<std::vector<cv::Point3f>> CalibrateAndRectify::
PrepareObjectPoints( int numberFrames,
                     cv::Size chessboardSize ) 
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    objectPoints.resize(numberFrames);

    // prepare object points
    // like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    for (int i = 0; i < numberFrames; i++) {
        for (int j = 0; j < chessboardSize.height; j++) {
            for (int k = 0; k < chessboardSize.width; k++) {

                // multiply by square size?
                objectPoints[i].push_back(cv::Point3f( float(j), 
                                                       float(k),
                                                       0.0       ));

            }
        }
    }

    return objectPoints;
}

void CalibrateAndRectify::
CalibrateStereoCamera( cv::Size chessboardSize, 
                       cv::Size frameSize,
                       int numberFrames,
                       std::vector<std::vector<cv::Point2f>> * framePoints )
{
    // 3D points in real world space
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // we expect that corners were found in each pair of images
    objectPoints = PrepareObjectPoints(numberFrames, chessboardSize);

    //cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat E, F;
    double rms;

    rms = cv::stereoCalibrate( objectPoints, 
                               framePoints[0], framePoints[1],
                               this->cameraMatrix[0], this->distCoeffs[0],
                               this->cameraMatrix[1], this->distCoeffs[1],
                               frameSize, 
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

    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];

    for (int i = 0; i < numberFrames; i++) {

        cv::Mat imgpt[2];

        // unnecessary part of algorithm
        // it is part related to average re-projection error
        int npt = (int)framePoints[0][i].size(); // size or rows? what is size?

        for (int k = 0; k < 2; k++) {
            imgpt[k] = cv::Mat(framePoints[k][i]);

            undistortPoints( imgpt[k], imgpt[k], 
                             this->cameraMatrix[k], 
                             this->distCoeffs[k], 
                             cv::Mat(), 
                             this->cameraMatrix[k]);

            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        // unnecessary part of algorithm
        // it is part related to average reprojection error
        err += ComputePartlyError( framePoints, lines, npt, i );
        npoints += npt;
    }

    SaveIntrinsics();
    
    // Compute the rest of extrinsic params
    cv::Rect validROI[2];
    stereoRectify( this->cameraMatrix[0], this->distCoeffs[0], 
                   this->cameraMatrix[1], this->distCoeffs[1], 
                   frameSize, 
                   this->R, this->T, 
                   this->R1, this->R2, 
                   this->P1, this->P2, 
                   this->Q, 
                   cv::CALIB_ZERO_DISPARITY, 
                   1, 
                   frameSize, 
                   &validROI[0], &validROI[1]);

    SaveExtrinsics();

    // lower is better
    std::cout << "Average Reprojection Error: " <<  err/npoints << std::endl;
    std::cout << "RMS Error: " << rms << std::endl;

    initUndistortRectifyMap( this->cameraMatrix[0], 
                             this->distCoeffs[0], 
                             this->R1, this->P1, 
                             frameSize, 
                             CV_16SC2, 
                             this->rmap[0][0], this->rmap[0][1] );

    initUndistortRectifyMap( this->cameraMatrix[1], 
                             this->distCoeffs[1], 
                             this->R2, this->P2, 
                             frameSize, 
                             CV_16SC2,
                             this->rmap[1][0], this->rmap[1][1] );

     SaveDistortionCameraModels();
}

// TODO find out the way of distribution of repired images
void CalibrateAndRectify::
RemapImage ( cv::Mat leftImage, 
             cv::Mat rightImage ) 
{
    cv::Mat leftImageRepaired, rightImageRepaired;

    remap( leftImage, leftImageRepaired, 
           this->rmap[0][0], this->rmap[0][1], 
           cv::INTER_LINEAR );

    remap( rightImage, rightImageRepaired, 
           this->rmap[1][0], this->rmap[1][1], 
           cv::INTER_LINEAR );
}
