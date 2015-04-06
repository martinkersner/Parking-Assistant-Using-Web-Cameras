/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Stereo Calibrating of Horizontally aligned cameras
 *
 * m.kersner@gmail.com
 * 01/24/2015
 */

#include "CalibrateAndRectify.h"
#include "Fps.h"

// TODO application arguments
//      refine realtime calibrating
//      loading/saving images from/to two directories (left, right)
//      get a number of images from given directory (boost?)

int main(int argc, char ** argv) {

    // TODO replace with application arguments
    std::string path          = "chessboard2/";
    const int numberImages    = 21; // temporary solution

    const float square_size   = 2.5; 
    const int rows            = 7; //9
    const int cols            = 7; //6
    cv::Size chessboardSize = cv::Size(rows, cols);

    // Real-time calibration
    CalibrateAndRectify cr(chessboardSize);

    //CalibrateAndRectify cr(path, numberImages, chessboardSize);

    //cv::Mat left = cv::imread("chessboard2/left9.ppm");
    //cv::Mat right = cv::imread("chessboard2/right9.ppm");

    //cv::Mat leftRepaired = cr.RemapLeftImage(left);
    //cv::Mat rightRepaired = cr.RemapRightImage(right);

    return EXIT_SUCCESS;
}

CalibrateAndRectify::
CalibrateAndRectify( cv::Size chessboardSize )
{
    FPS fps;
    cv::VideoCapture capLeft(1); 
    cv::VideoCapture capRight(2);

    assert(capLeft.isOpened() && capRight.isOpened());

    cv::namedWindow("Left",1);
    cv::namedWindow("Right",1);

    char k;
    cv::Mat frameLeft, frameRight;

    // 2D points in image plane
    std::vector<std::vector<cv::Point2f>> imagePoints[2]; 
    int numberCalibrated = 0;
    bool valid = false;

    // TODO Create an argument from it
    //      or Can we get a number of images for calibration from imagePoints?
    int numberImagesForCalibration = 2;

    // retrive size of frame
    // used to compute an error of rectification
    capLeft >> frameLeft;
    cv::Size frameSize = frameLeft.size();

    // TODO Find a better way to create a timespan between capturing chessboards
    //      for calibration.
    int i = 0;
    while (true) {   
        fps.before();

        capLeft >> frameLeft;
        capRight >> frameRight;

        if (i++ > 100) {
            i = 0;
            valid = FindStereoChessboardCorners( frameLeft, 
                                                 frameRight, 
                                                 chessboardSize, 
                                                 imagePoints );

            if (valid) {
                valid = false;

                std::cout << numberCalibrated << std::endl;

                if (++numberCalibrated >= numberImagesForCalibration) {
                    CalibrateStereoCamera( chessboardSize,
                                           frameSize,
                                           numberImagesForCalibration, 
                                           imagePoints );

                    // TODO better way of terminating app
                    exit(1);
                }
            }
        } 

        fps.after();

        // adding info about FPS to image
        std::string text = "FPS: " + std::to_string((int)fps.getFps());
        PutTextToImage(frameLeft, text);

        cv::imshow("Left", frameLeft);
        cv::imshow("Right", frameRight);

        k = cv::waitKey(30);

        if (k == 'q')
            break;
    }
}

CalibrateAndRectify::
CalibrateAndRectify( std::string pathToImages, 
                     int numberImages, 
                     cv::Size chessboardSize )
{
    // 2D points in image plane
    std::vector<std::vector<cv::Point2f>> imagePoints[2]; 

    std::vector<StringPair> vecStringPairs = GetFileNames(pathToImages, numberImages);

    cv::Size imageSize = GetImageSize(vecStringPairs);

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

CalibrateAndRectify::
CalibrateAndRectify( std::string intrinsics,
                     std::string extrinsic )
{
    LoadIntrinsics();
    LoadExtrinsics();
    LoadDistortionCameraModels();
}

void CalibrateAndRectify::
PutTextToImage( cv::Mat & image, 
                std::string text )
{
    cv::putText( image, 
                 text, 
                 cv::Point(40,40),
                 cv::FONT_HERSHEY_SIMPLEX,
                 0.5,
                 cv::Scalar(255,0,0) );
}

void CalibrateAndRectify::
LoadIntrinsics() 
{
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ); 

    if (fs.isOpened()) {
        fs["M1"] >> this->cameraMatrix[0];
        fs["D1"] >> this->distCoeffs[0];
        fs["M2"] >> this->cameraMatrix[1];
        fs["D2"] >> this->distCoeffs[1];

        fs.release();
    }
}

void CalibrateAndRectify::
LoadExtrinsics() 
{
    cv::FileStorage fs("extrinsics.yml", cv::FileStorage::READ); 

    if (fs.isOpened()) {
        fs["R"]  >> this->R;
        fs["T"]  >> this->T;
        fs["R1"] >> this->R1;
        fs["R2"] >> this->R2;
        fs["P1"] >> this->P1;
        fs["P2"] >> this->P2;
        fs["Q"]  >> this->Q;
    }
}

void CalibrateAndRectify::
LoadDistortionCameraModels()
{
    cv::FileStorage fs("distortion_camera_models.yml", cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["MX1"] >> this->rmap[0][0];
        fs["MY1"] >> this->rmap[0][1];
        fs["MX2"] >> this->rmap[1][0];
        fs["MY2"] >> this->rmap[1][1];

        fs.release();
    }
}

// TODO find better way of storing left and right images
std::vector<StringPair> CalibrateAndRectify::
GetFileNames( std::string & path, 
              int numberImages ) 
{
    std::vector<StringPair> vecStringPairs;

    for (int i = 0; i < numberImages; ++i) {
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
                 std::vector<std::vector<cv::Point2f>> * imagePoints )
{
    cv::Mat imageLeft = cv::imread(sp.at(0));
    cv::Mat imageRight = cv::imread(sp.at(1));

    FindStereoChessboardCorners(imageLeft, imageRight, chessboardSize, imagePoints);
}

/**
 * Expects the same size of all images, therefore only the size of the first image 
 * is checked.
 */
cv::Size CalibrateAndRectify::
GetImageSize( std::vector<StringPair> & vecStringPairs ) 
{
    cv::Mat tmpImage = cv::imread(vecStringPairs.at(0).at(0));
    return tmpImage.size();
}

bool CalibrateAndRectify::
FindStereoChessboardCorners( cv::Mat imageLeft, 
                             cv::Mat imageRight, 
                             cv::Size chessboardSize,
                             std::vector<std::vector<cv::Point2f>> * imagePoints )
{
    //cv::Mat imageLeftOriginal  = imageLeft;
    //cv::Mat imageRightOriginal = imageRight;

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
                                           chessboardSize, 
                                           cornersLeft, 
                                           flags );

    foundRight = cv::findChessboardCorners( imageRight, 
                                            chessboardSize, 
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

    // TODO delete?
    //drawChessboardCorners(imageLeftOriginal, boardSize, cornersLeft, foundLeft);
    //drawChessboardCorners(imageRightOriginal, boardSize, cornersRight, foundRight);
    
    if (foundLeft && foundRight) {
        // store found corners only if it has been found at both of examined images
        imagePoints[0].push_back(cornersLeft);
        imagePoints[1].push_back(cornersRight);


        return true;
    }
    else
        return false;
}

void CalibrateAndRectify::
SaveIntrinsics() 
{
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);

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
    cv::FileStorage fs("extrinsics.yml", cv::FileStorage::WRITE);

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
    cv::FileStorage fs("distortion_camera_models.yml", cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "MX1"  << this->rmap[0][0]
           << "MY1"  << this->rmap[0][1]
           << "MX2"  << this->rmap[1][0]
           << "MY2"  << this->rmap[1][1];

        fs.release();
    }
}

double CalibrateAndRectify::
ComputePartlyError( std::vector<std::vector<cv::Point2f>> * imagePoints, 
                    std::vector<cv::Vec3f> * lines, 
                    int npt,
                    int i )
{
    double err   = 0;
    double errij = 0;

    for(int j = 0; j < npt; j++ ) {
        errij = fabs( imagePoints[0][i][j].x*lines[1][j][0] +
                      imagePoints[0][i][j].y*lines[1][j][1] + 
                      lines[1][j][2] ) 
                +
                fabs( imagePoints[1][i][j].x*lines[0][j][0] +
                      imagePoints[1][i][j].y*lines[0][j][1] + 
                      lines[0][j][2] );

        err += errij;
    }

    return err;
}

std::vector<std::vector<cv::Point3f>> CalibrateAndRectify::
PrepareObjectPoints( int numberImages,
                     cv::Size chessboardSize ) 
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    objectPoints.resize(numberImages);

    // prepare object points
    // like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    for (int i = 0; i < numberImages; i++) {
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
                       cv::Size imageSize,
                       int numberImages,
                       std::vector<std::vector<cv::Point2f>> * imagePoints )
{
    // 3D points in real world space
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // we expect that corners were found in each pair of images
    objectPoints = PrepareObjectPoints(numberImages, chessboardSize);

    //cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat E, F;
    double rms;

    rms = cv::stereoCalibrate( objectPoints, 
                               imagePoints[0], imagePoints[1],
                               this->cameraMatrix[0], this->distCoeffs[0],
                               this->cameraMatrix[1], this->distCoeffs[1],
                               imageSize, 
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

    for (int i = 0; i < numberImages; i++) {

        cv::Mat imgpt[2];

        // unnecessary part of algorithm
        // it is part related to average re-projection error
        int npt = (int)imagePoints[0][i].size(); // size or rows? what is size?

        for (int k = 0; k < 2; k++) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);

            undistortPoints( imgpt[k], imgpt[k], 
                             this->cameraMatrix[k], 
                             this->distCoeffs[k], 
                             cv::Mat(), 
                             this->cameraMatrix[k]);

            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        // unnecessary part of algorithm
        // it is part related to average reprojection error
        err += ComputePartlyError( imagePoints, lines, npt, i );
        npoints += npt;
    }

    SaveIntrinsics();
    
    // Compute the rest of extrinsic params
    cv::Rect validROI[2];
    stereoRectify( this->cameraMatrix[0], this->distCoeffs[0], 
                   this->cameraMatrix[1], this->distCoeffs[1], 
                   imageSize, 
                   this->R, this->T, 
                   this->R1, this->R2, 
                   this->P1, this->P2, 
                   this->Q, 
                   cv::CALIB_ZERO_DISPARITY, 
                   1, 
                   imageSize, 
                   &validROI[0], &validROI[1]);

    SaveExtrinsics();

    // lower is better
    std::cout << "Average Reprojection Error: " <<  err/npoints << std::endl;
    std::cout << "RMS Error: " << rms << std::endl;

    initUndistortRectifyMap( this->cameraMatrix[0], 
                             this->distCoeffs[0], 
                             this->R1, this->P1, 
                             imageSize, 
                             CV_16SC2, 
                             this->rmap[0][0], this->rmap[0][1] );

    initUndistortRectifyMap( this->cameraMatrix[1], 
                             this->distCoeffs[1], 
                             this->R2, this->P2, 
                             imageSize, 
                             CV_16SC2,
                             this->rmap[1][0], this->rmap[1][1] );

     SaveDistortionCameraModels();
}

cv::Mat CalibrateAndRectify::
RemapLeftImage ( cv::Mat leftImage ) 
{
    cv::Mat leftImageRepaired;

    remap( leftImage, leftImageRepaired, 
           this->rmap[0][0], this->rmap[0][1], 
           cv::INTER_LINEAR );

    return leftImageRepaired;
}

cv::Mat CalibrateAndRectify::
RemapRightImage( cv::Mat rightImage ) 
{
    cv::Mat rightImageRepaired;

    remap( rightImage, rightImageRepaired, 
           this->rmap[1][0], this->rmap[1][1], 
           cv::INTER_LINEAR );

    return rightImageRepaired;
}

/**
 * Horizontally concatenate given images and draw lines through them.
 * Function expect the same size of both images.
 */
cv::Mat CalibrateAndRectify::
DrawComparingLines ( cv::Mat leftImage,
                     cv::Mat rightImage )
{
    assert(leftImage.size() == rightImage.size());

    cv::Size sz = leftImage.size();

    cv::Mat compLines(sz.height, 2*sz.width, CV_8UC3);
    cv::Mat left(compLines, cv::Rect(0, 0, sz.width, sz.height));
    cv::Mat right(compLines, cv::Rect(sz.width, 0, sz.width, sz.height));

    leftImage.copyTo(left);
    rightImage.copyTo(right);

    for (int j = 0; j < sz.height; j += 16) {
        cv::line( compLines, 
                  cv::Point(0, j),
                  cv::Point(sz.width*2, j),
                  CV_RGB(255,0,0) );
    }

    return compLines;
}
