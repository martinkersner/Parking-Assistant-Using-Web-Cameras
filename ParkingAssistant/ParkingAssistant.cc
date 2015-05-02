/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * m.kersner@gmail.com
 * 03/28/2015
 */

#include "ParkingAssistant.h"

int main( int argc, char **argv ) 
{
#if DEMO == 0  // REAL
    distortion = "distortion_camera_models.yml";
    extrinsics = "extrinsics.yml";
    cv::Mat background = cv::imread("background.pgm", cv::COLOR_BGR2GRAY);

    int code = ParkingAssistantRealTime( distortion, 
                                         extrinsics, 
                                         background,
                                         SQUARE_SIZE );
    return code;

#elif DEMO == 1 
    std::cout << "DEMO" << std::endl;

    // OBJECT DETECTION
    std::cout << "OBJECT DETECTION" << std::endl;
    std::string objectDetectionDatasetPath = "../Datasets/Object_Detection/";
    ObjectDetectionDemo( objectDetectionDatasetPath );

    // DEPTH ESTIMATION ACCURACY
    std::cout << "DEPTH ESTIMATION ACCURACY" << std::endl;
    std::string depthEstimationDatasetPath = "../Datasets/Depth_Estimation_Accuracy/";
    DepthEstimationDemo ( depthEstimationDatasetPath );

    // CURB DETECTION
    std::cout << "CURB DETECTION" << std::endl;
    std::string curbDatasetPath = "../Datasets/Curbs/";
    CurbDetectionDemo( curbDatasetPath );

    return EXIT_SUCCESS;
#endif
}

int ParkingAssistantRealTime( std::string distortion, 
                              std::string extrinsics,
                              cv::Mat & background,
                              float squareSize )
{
    cv::Mat left, 
            right;
    char key;
    State state = STANDBY;

    // We assume that we connect additional webcams.
    // Linux assigns numbers to webcams.
    // Number 0 is ussualy used for built-int webcam, however if the computer 
    // lacks of webcam, this option should be changed.
    // Number one stands for left camera, number 2 is for right one.
    ImageAcquisition ia( LEFT_CAM_IDX, RIGHT_CAM_IDX, 
                         IMG_WIDTH, IMG_HEIGHT, CAM_FPS ); 

    if (ia.VerifyConnection() == EXIT_FAILURE)
        return EXIT_FAILURE;

    ia.SkipFirstFrames();

    // Calibration
    int calTimeSpan = CALIB_TIMESPAN;
    Calibration c( CHESS_ROWS, CHESS_COLS, 
                   CALIB_IMAGES, CALIB_DIR, 
                   cv::Size(IMG_WIDTH, IMG_HEIGHT),
                   CAM_PROP_INT, CAM_PROP_EXT, CAM_PROP_DIS );

    // Collision Avoidance System
    // employs default calibration files that need to be recalibrated
    CollisionAvoidanceSystem cas( extrinsics, 
                                  distortion, 
                                  background,
                                  SQUARE_SIZE );

    // Curb Detection
    CurbDetection cd;

    int i = 0;
    while (true) {
        ia.ReadLeftAndRight(left, right);

        // for debugging purposes
        DisplayLeftAndRight(left, right);

        if (state == RUNNING) {
            cas.Detect(left, right);
            cd.Detect(left);
        }
        
        if (state == CALIBRATION) {
            i++;
            if (i >= CALIB_TIMESPAN) {
                i = 0;

                if (c.Calibrate(left, right) == EXIT_SUCCESS) {
                    state = STANDBY;

                    // reupload calibration configuration
                    cas.ReinitializeCalibrationFiles();
                }
            }
        }

        // keys interruption
        key = cv::waitKey(1);
        switch (key) {
            case 'q':
                cv::destroyAllWindows();
                return EXIT_SUCCESS;

            case 'c':
                state = CALIBRATION;
                break;

            case 'r':
                state = RUNNING;
                break;

            case 's':
                state = STANDBY;
                break;
        }
    }

    return EXIT_SUCCESS;
}

void DisplayLeftAndRight( cv::Mat & left,
                          cv::Mat & right )
{
    cv::Mat concatenation;
    cv::hconcat(left, right, concatenation);
    cv::imshow("View", concatenation);
}

// Runs object detection on whole dataset.
void ObjectDetectionDemo( std::string datasetPath )
{
    int disparityNumber = 176;

    cv::Mat background = cv::imread( datasetPath + "background/disparity-" + 
                                     std::to_string(disparityNumber) + ".pgm",
                                     cv::COLOR_BGR2GRAY );

    CollisionAvoidanceSystem cas( datasetPath + "extrinsics.yml", 
                                  datasetPath + "distortion_camera_models.yml", 
                                  background,
                                  SQUARE_SIZE,
                                  DEMO );

    cas.SetNumberOfDisparities(disparityNumber);

    std::string objectImagePath;
    cv::Mat objectImageLeft, objectImageRight;

    std::vector<ObjectDetectionSubset> subsets;
    ObjectDetectionSubset scene0 = { "scene0/", 1 };
    ObjectDetectionSubset scene1 = { "scene1/", 1 };
    ObjectDetectionSubset scene2 = { "scene2/", 1 };
    ObjectDetectionSubset scene3 = { "scene3/", 1 };
    ObjectDetectionSubset scene4 = { "scene4/", 1 };
    ObjectDetectionSubset scene5 = { "scene5/", 1 };
    ObjectDetectionSubset carboy = { "carboy/", 12 };
    ObjectDetectionSubset teddy  = { "teddy/",  12 };
    ObjectDetectionSubset box    = { "box/",    12 };

    subsets.push_back(scene0);
    subsets.push_back(scene1);
    subsets.push_back(scene2);
    subsets.push_back(scene3);
    subsets.push_back(scene4);
    subsets.push_back(scene5);
    subsets.push_back(carboy);
    subsets.push_back(teddy);
    subsets.push_back(box);

    for (ObjectDetectionSubset ods : subsets) {
        for (int i = 0; i < ods.number; i++) {
            objectImagePath = datasetPath + ods.name + std::to_string(i) + "-original-";

            objectImageLeft  = cv::imread(objectImagePath + "left.png");
            objectImageRight = cv::imread(objectImagePath + "right.png");

            cas.Detect(objectImageLeft, objectImageRight);
        }
    }

    cv::destroyAllWindows();
}

void DepthEstimationDemo ( std::string datasetPath )
{
    CollisionAvoidanceSystem cas( datasetPath + "extrinsics.yml", 
                                  datasetPath + "distortion_camera_models.yml", 
                                  cv::Mat(),
                                  SQUARE_SIZE,
                                  DEMO );

    cas.SetNumberOfDisparities(160);

    cv::Mat mask,
            objectImageLeft, objectImageRight;
    std::string objectImagePath;
    
    std::vector<DepthEstimationScene> scenes;
    DepthEstimationScene scene0 = { 0, 37 };
    DepthEstimationScene scene1 = { 1, 44 };
    DepthEstimationScene scene2 = { 2, 54 };
    DepthEstimationScene scene3 = { 3, 72 };
    DepthEstimationScene scene4 = { 4, 84 };
    DepthEstimationScene scene5 = { 5, 93 };
    DepthEstimationScene scene6 = { 6, 104 };
    DepthEstimationScene scene7 = { 7, 114 };
    DepthEstimationScene scene8 = { 8, 125 };
    DepthEstimationScene scene9 = { 9, 144 };

    scenes.push_back(scene0);
    scenes.push_back(scene1);
    scenes.push_back(scene2);
    scenes.push_back(scene3);
    scenes.push_back(scene4);
    scenes.push_back(scene5);
    scenes.push_back(scene6);
    scenes.push_back(scene7);
    scenes.push_back(scene8);
    scenes.push_back(scene9);

    float distance, errDistance;
    for (DepthEstimationScene des : scenes) {
        objectImagePath = datasetPath + "scene" + std::to_string(des.index) + "/";

        mask = cv::imread(objectImagePath + "mask.png", cv::COLOR_BGR2GRAY);
        objectImageLeft  = cv::imread(objectImagePath + "0-original-left.png");
        objectImageRight = cv::imread(objectImagePath + "0-original-right.png");

        // distance estimation
        distance = cas.Detect(objectImageLeft, objectImageRight, mask);
        errDistance = std::abs(distance-des.distance);

        cv::putText( objectImageLeft, "est.: " + std::to_string(distance) + " cm", 
                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                     1, cv::Scalar(255, 255, 255), 2);

        cv::putText( objectImageLeft, "err.: " + std::to_string(errDistance) + " cm", 
                     cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 
                     1, cv::Scalar(255, 255, 255), 2);

        cv::imshow("Source image", objectImageLeft);
        cv::waitKey();
    }

    cv::destroyAllWindows();
}

// Runs curb detection on whole curb dataset.
void CurbDetectionDemo( std::string datasetPath )
{
    CurbDetection cd(DEMO);

    std::string curbImagePath;
    cv::Mat     curbImage;

    for (int i = 0; i < 19; ++i) { // iterating through subsets
        for (int j = 0; j < 10; ++j) { // iterating through images in subset

            curbImagePath = datasetPath + 
                            "subset" + std::to_string(i) + "/" + 
                            std::to_string(j) + ".jpg";

            curbImage = cv::imread(curbImagePath);
            cd.Detect(curbImage);

        } 
    }

    cv::destroyAllWindows();
}
