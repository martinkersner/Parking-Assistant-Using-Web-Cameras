#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class SFM {
    cv::Mat intrinsics;
    cv::Mat distortion;

    cv::Mat fundamental;
    cv::Mat essential;

    cv::Mat imgLeft;
    cv::Mat imgRight;

    std::vector<cv::Point2f> leftPoints;
    std::vector<cv::Point2f> rightPoints;
    std::vector<cv::Vec3b> colorPoints;

    const cv::Mat projectionLeft = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat projectionRight;

    cv::Mat flow;
    cv::Mat model;

    // Features and Descriptors
	std::vector<cv::Mat> imgs; 
	std::vector<std::vector<cv::KeyPoint> > imgpts;
	std::vector<cv::Mat> descriptors;

    std::vector<cv::KeyPoint> imgpts1_good, imgpts2_good;
    std::vector<cv::DMatch> good_matches_, very_good_matches_;
    std::vector<cv::KeyPoint> imgpts2_very_good, imgpts1_very_good;

    void loadIntrinsics( std::string intrinsicsFile );
    void loadDistortion( std::string distortionFile );
    void calcOpticalFlow();
    void createPointsFlow();
    void createPointsFeatures();
    void undistortPoints();
    void createFundamentalEssential();
    void createProjection();
    void TriangulatePointsOpenCV();

    void calcFeaturesDescriptors();
    void MatchFeatures( int idx_i, 
                        int idx_j );

    double TriangulatePoints();

    cv::Mat_<double> LinearLSTriangulation( cv::Point3d u,
                                            cv::Matx34d P, 
                                            cv::Point3d u1,
                                            cv::Matx34d P1 );

    std::vector<cv::DMatch> EliminateMatches( const std::vector<cv::KeyPoint> & kpts1,
                                              const std::vector<cv::KeyPoint> & kpts2,
                                              std::vector<cv::KeyPoint> & kpts1_good,
                                              std::vector<cv::KeyPoint> & kpts2_good,
                                              std::vector<cv::DMatch> & matches );

    std::vector<uchar> GetUsedMatches( std::vector<cv::KeyPoint> kpts1,
                                       std::vector<cv::KeyPoint> kpts2 );

    void AlignPoints( const std::vector<cv::KeyPoint> & kpts1,
                      const std::vector<cv::KeyPoint> & kpts2,
                      const std::vector<cv::DMatch>   & matches,
                      std::vector<cv::KeyPoint>       & pts1,
                      std::vector<cv::KeyPoint>       & pts2 );

    bool CheckCoherentRotation(cv::Mat_<double> & R);
    void SaveCamera( cv::Mat_<double> R, 
                     cv::Mat_<double> t );

    int GetSign( double value );
    cv::Mat drawOpticalFlow( cv::Mat & image,
            cv::Mat & flowX,
            cv::Mat & flowY );

    void pokus( cv::Mat & resultDenseOpticalFlow,
            cv::Mat & flowX,
            cv::Mat & flowY
            );

    public:
        SFM( cv::Mat imgLeft,
             cv::Mat imgright,
             std::string intrinsicsFile,
             std::string distortionFile );

        cv::Mat getModel();
        void printModel();
};
