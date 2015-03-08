/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Capturing and rectification of frames from tow webcams.
 *
 * m.kersner@gmail.com
 * 03/08/2015
 */

#include "CaptureTwoCameras.h"

int main()
{
    std::string outDir = "output/";
    std::string fileName = "distortion_camera_models.yml";
    CaptureTwoCameras ctc(1, 2, fileName, outDir);
     
    cv::Mat left, right, concatenation;
    char key;
    int i = 0;
    CaptureState cs = INITIALIZED;

    namedWindow("TwoCameras", cv::WINDOW_AUTOSIZE);

    while (true) {   
        key = cv::waitKey(1);

        if (key == 'c' || cs == RUNNING) {
            left  = ctc.ReadLeftRepairSave(i);
            right = ctc.ReadRightRepairSave(i);

            hconcat(left, right, concatenation);
            cv::imshow("TwoCameras", concatenation);

            cs = RUNNING;
            i++;
        }
        else if (key == 'p') {
            cs = PAUSE;
        }
        else if (key == 'q') {
            break;
        }
    }

    return EXIT_SUCCESS;
}

CaptureTwoCameras::CaptureTwoCameras( int indexLeft, 
                                      int indexRight,
                                      std::string fileName,
                                      std::string outDir ) : indexLeft(indexLeft), 
                                                             indexRight(indexRight),
                                                             outDir(outDir)

{
    capLeft = OpenCamera(this->indexLeft);
    capRight= OpenCamera(this->indexRight);
    LoadDistortionCameraModels(fileName);
    SkipFirstFrames();
}

cv::VideoCapture CaptureTwoCameras::OpenCamera( int index )
{
    cv::VideoCapture cap(index); 
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //cap.set(cv::CAP_PROP_FPS , 30);
    
    return cap;
}

void CaptureTwoCameras::LoadDistortionCameraModels( std::string fileName )
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["MX1"] >> this->rmap[0][0];
        fs["MY1"] >> this->rmap[0][1];
        fs["MX2"] >> this->rmap[1][0];
        fs["MY2"] >> this->rmap[1][1];

        fs.release();
    }
}

cv::Mat CaptureTwoCameras::RemapLeftImage( cv::Mat leftImage ) 
{
    cv::Mat leftImageRepaired;

    remap( leftImage, leftImageRepaired, 
           this->rmap[0][0], this->rmap[0][1], 
           cv::INTER_LINEAR );

    return leftImageRepaired;
}

cv::Mat CaptureTwoCameras::RemapRightImage( cv::Mat rightImage ) 
{
    cv::Mat rightImageRepaired;

    remap( rightImage, rightImageRepaired, 
           this->rmap[1][0], this->rmap[1][1], 
           cv::INTER_LINEAR );

    return rightImageRepaired;
}

cv::Mat CaptureTwoCameras::ReadLeftAndRepair()
{
    cv::Mat tmpLeft;

    this->capLeft >> tmpLeft;
    
    return RemapLeftImage(tmpLeft);
}

cv::Mat CaptureTwoCameras::ReadLeftRepairSave( int index )
{
    cv::Mat left = ReadLeftAndRepair();
    std::string path = this->outDir + "left-" + std::to_string(index) + ".png";
    cv::imwrite(path, left);

    return left;
}

cv::Mat CaptureTwoCameras::ReadRightAndRepair()
{
    cv::Mat tmpRight;

    this->capRight >> tmpRight;
    
    return RemapRightImage(tmpRight);
}

cv::Mat CaptureTwoCameras::ReadRightRepairSave( int index )
{
    cv::Mat right = ReadRightAndRepair();
    std::string path = this->outDir + "right-" + std::to_string(index) + ".png";
    cv::imwrite(path, right);

    return right;
}

inline void CaptureTwoCameras::SkipFirstFrames()
{
    cv::Mat tmpLeft, tmpRight;

    this->capLeft >> tmpLeft;
    this->capRight >> tmpRight;
}
