/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 * 
 * Structure from Motion
 *
 * Inspired by Roy Shilkrot and Saburo Okita: 
 * Mastering OpenCV with Practical Computer Vision Projects
 * http://subokita.com/2014/03/26/structure-from-motion-using-farnebacks-optical-flow-part-2/
 *
 * m.kersner@gmail.com
 * 02/07/2015
 */

#include "StructureFromMotion.h"

SFM::SFM( cv::Mat imgLeft,
          cv::Mat imgRight,
          std::string intrinsicsFile,
          std::string distortionFile )
{
    this->imgLeft = imgLeft;
    this->imgRight = imgRight;

    loadIntrinsics(intrinsicsFile);
    loadDistortion(distortionFile);

    // OPTICAL FLOW
    calcOpticalFlow();

    createPointsFlow();

    // FEATURES
    //calcFeaturesDescriptors();
    //MatchFeatures(0,1);
    //createPointsFeatures();
    
    undistortPoints();
    createFundamentalEssential();
    createProjection();

    TriangulatePointsOpenCV();
    //TriangulatePoints();
}

cv::Mat SFM::drawOpticalFlow( cv::Mat & image,
                      cv::Mat & flowX,
                      cv::Mat & flowY )
{
  cv::Mat pile = cv::Mat::ones(11, 11, CV_8UC1);
  assert(pile.rows%2 != 0 && pile.cols%2 != 0);

  int startRow = pile.rows/2;
  int startCol = pile.cols/2;

  cv::filter2D(flowX, flowX, -1, pile);
  cv::filter2D(flowY, flowY, -1, pile);
 
  cv::Point p, q;

  //cv::normalize(flowX, flowX, -startRow, startRow, cv::NORM_MINMAX);
  //cv::normalize(flowY, flowY, -startRow, startRow, cv::NORM_MINMAX);

  //double min;
  //double max;
  //cv::minMaxLoc(flowY, &min, &max);

  //std::cout << min << std::endl;
  //std::cout << max << std::endl;
  /*
     cv::Point2f flowPoint = this->flow.at<cv::Point2f>(y, x);
     this->leftPoints.push_back(cv::Point2f(x, y));
     this->rightPoints.push_back(cv::Point2f(x + flowPoint.x, y + flowPoint.y));
  */

  double min, max;
  double min2, max2;
  cv::minMaxLoc(flowX, &min, &max);
  cv::minMaxLoc(flowY, &min2, &max2);
  std::cout << min << " " << max << std::endl;
  std::cout << min2 << " " << max2 << std::endl;

  for (int y = startRow; y < image.rows; y+=pile.rows) {
    for (int x = startCol; x < image.cols; x+=pile.cols) {
      p.x = x;
      p.y = y;

      //std::cout << flowX.at<double>(y, x) << " " 
      //          << flowY.at<double>(y, x) << std::endl;

      //q.x = x + flowX.at<double>(y, x);
      //q.y = y + flowY.at<double>(y, x);
      
      double mag = sqrt( (flowX.at<double>(y, x) * flowX.at<double>(y, x)) +
             flowY.at<double>(y, x) + flowY.at<double>(y, x) ) / 100;

     // q.x = x + 2 * startRow * GetSign(flowX.at<double>(y, x));
     // q.y = y + 2 * startCol * GetSign(flowY.at<double>(y, x));

      q.x = x + mag * GetSign(flowX.at<double>(y, x));
      q.y = y + mag * GetSign(flowY.at<double>(y, x));

      cv::arrowedLine( image, 
                       p, 
                       q, 
                       cv::Scalar(255,0,0),
                       1,
                       8,
                       0,
                       0.1 );
    }
  }

  return image;
}

int SFM::GetSign( double value )
{
  if (value >= 0)
    return 1;
  else
    return -1;
}

void SFM::loadIntrinsics( std::string fileName )
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["i"] >> this->intrinsics;
        fs.release();
    }
}

void SFM::loadDistortion( std::string fileName )
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    if (fs.isOpened()) {
        fs["d"] >> this->distortion;
        fs.release();
    }
}

void SFM::SaveCamera( cv::Mat_<double> R, 
                      cv::Mat_<double> t )
{
    cv::FileStorage fs("camera.yml", cv::FileStorage::WRITE);
    fs << "R" << R;
    fs << "t" << t;
    fs.release();
}

void SFM::calcOpticalFlow()
{
    cv::Mat grayLeft, grayRight;

    cv::cvtColor(this->imgLeft,  grayLeft,  CV_BGR2GRAY);
    cv::cvtColor(this->imgRight, grayRight, CV_BGR2GRAY);

    cv::calcOpticalFlowFarneback( grayLeft, grayRight, this->flow, 
                                  0.5,   // pyramide scale
                                  10,     // levels 
                                  400,    // winsize
                                  30,    // iteration
                                  7,     // poly_n
                                  1.5,   // poly_sigma
                                  0 );   // flags
}

void SFM::calcFeaturesDescriptors()
{
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;

	detector = cv::FeatureDetector::create("PyramidFAST");
	extractor = cv::DescriptorExtractor::create("ORB");

    this->imgs.push_back(this->imgLeft);
    this->imgs.push_back(this->imgRight);
	
	detector->detect(this->imgs, this->imgpts);
	extractor->compute(this->imgs, this->imgpts, this->descriptors);
}

void SFM::MatchFeatures( int idx_i, 
                         int idx_j )
{
    const std::vector<cv::KeyPoint> & kpts1 = this->imgpts[idx_i];
    const std::vector<cv::KeyPoint> & kpts2 = this->imgpts[idx_j];

    const cv::Mat & descriptors_1 = this->descriptors[idx_i];
    const cv::Mat & descriptors_2 = this->descriptors[idx_j];

    std::set<int> existing_trainIdx;
    std::vector<cv::DMatch> matches;
    
    if (descriptors_1.empty())
        CV_Error(0, "descriptors_1 is empty");

    if (descriptors_2.empty())
        CV_Error(0, "descriptors_2 is empty");
    
    //matching descriptor vectors using Brute Force matcher
    //allow cross-check. use Hamming distance for binary descriptor (ORB)
    cv::BFMatcher matcher(cv::NORM_HAMMING, true); 

    matcher.match(descriptors_1, descriptors_2, matches);

    // TODO delete
    std::cout << matches.size() << " matches" << std::endl;
    
	assert(matches.size() > 0);
    
    // Eliminate any re-matching of training points (multiple queries to one training)
    for(unsigned int i = 0; i < matches.size(); i++ )
    { 
        //"normalize" matching: sometimes imgIdx is the one holding the trainIdx
        if (matches[i].trainIdx <= 0) {
            matches[i].trainIdx = matches[i].imgIdx;
        }
        
        if(existing_trainIdx.find(matches[i].trainIdx) == existing_trainIdx.end() && 
           matches[i].trainIdx >= 0 && 
           matches[i].trainIdx < (int)(kpts2.size()) )
        {
            this->good_matches_.push_back( matches[i]);
            this->imgpts1_good.push_back(kpts1[matches[i].queryIdx]);
            this->imgpts2_good.push_back(kpts2[matches[i].trainIdx]);
            existing_trainIdx.insert(matches[i].trainIdx);
        }
    }
    
    //DrawDescriptorMatches(this->imgs[idx_i], this->imgs[idx_j], std::string("img.png"));
    
    assert(this->imgpts1_good.size()  > 0);
	assert(this->imgpts2_good.size()  > 0);
	assert(this->good_matches_.size() > 0);
	assert(this->imgpts1_good.size() == this->imgpts2_good.size() && 
           this->imgpts1_good.size() == this->good_matches_.size());
	
    //Select features that make epipolar sense
    EliminateMatches( kpts1, kpts2, 
                      this->imgpts1_very_good, this->imgpts2_very_good,
                      this->good_matches_ );

    //DrawDescriptorMatches( this->imgs[idx_i], this->imgs[idx_j], 
    //                       std::string("good.png") );
}

void SFM::createPointsFeatures()
{
    std::cout << imgpts2_good.size() << std::endl;

    //for (auto i = 0; i < imgpts1_good.size(); ++i) {
    for (auto i = 0; i < imgpts1_very_good.size(); ++i) {
        cv::Vec3b color = this->imgLeft.at<cv::Vec3b>(imgpts1_good.at(i).pt);

        this->leftPoints.push_back(imgpts1_very_good.at(i).pt);
        this->rightPoints.push_back(imgpts2_very_good.at(i).pt);

        //this->leftPoints.push_back(imgpts1_good.at(i).pt);
        //this->rightPoints.push_back(imgpts2_good.at(i).pt);

        this->colorPoints.push_back(color);
    }
}

void SFM::createPointsFlow()
{
    int delta = 6;

    for (int y = 0; y < this->flow.rows; y+=delta) {
        for (int x = 0; x < this->flow.cols; x+=delta) {
            cv::Point2f flowPoint = this->flow.at<cv::Point2f>(y, x);
            cv::Vec3b color = this->imgLeft.at<cv::Vec3b>(y, x);
            
            if (fabs(flowPoint.x) < 0.1 && fabs(flowPoint.y) < 0.1)
                continue;
            
            this->leftPoints.push_back(cv::Point2f(x, y));
            this->rightPoints.push_back(cv::Point2f(x + flowPoint.x, y + flowPoint.y));
            this->colorPoints.push_back(color);
        }
    }
}

void SFM::undistortPoints()
{
    cv::undistortPoints(this->leftPoints,  this->leftPoints,  this->intrinsics, this->distortion);
    cv::undistortPoints(this->rightPoints, this->rightPoints, this->intrinsics, this->distortion);
}

void SFM::createFundamentalEssential()
{
    this->fundamental = cv::findFundamentalMat(leftPoints, rightPoints, cv::FM_RANSAC, 3.0, 0.99 );
    this->essential   = this->intrinsics.t() * this->fundamental * this->intrinsics;
}

void SFM::createProjection()
{
    cv::SVD svd(this->essential);
    static const cv::Mat W = ( cv::Mat_<double>(3, 3) <<
                               0, -1, 0,
                               1, 0, 0,
                               0, 0, 1 );

    static const cv::Mat Wt = ( cv::Mat_<double>(3, 3) <<
                                 0, 1, 0,
                                -1, 0, 0,
                                 0, 0, 1 );
    
    cv::Mat_<double> R1 = svd.u * W * svd.vt;
    cv::Mat_<double> R2 = svd.u * Wt * svd.vt;
    cv::Mat_<double> T1 = svd.u.col(2);
    cv::Mat_<double> T2 = -svd.u.col(2);
    
//    // R1 T1
//    this->projectionRight =( cv::Mat_<double>(3, 4) <<
//                             R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
//                             R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
//                             R1(2, 0), R1(2, 1), R1(2, 2), T1(2) );

    // R1 T2
    this->projectionRight =( cv::Mat_<double>(3, 4) <<
                             R1(0, 0), R1(0, 1), R1(0, 2), T2(0),
                             R1(1, 0), R1(1, 1), R1(1, 2), T2(1),
                             R1(2, 0), R1(2, 1), R1(2, 2), T2(2) );

//    // R2 T1
//    this->projectionRight =( cv::Mat_<double>(3, 4) <<
//                             R2(0, 0), R2(0, 1), R2(0, 2), T1(0),
//                             R2(1, 0), R2(1, 1), R2(1, 2), T1(1),
//                             R2(2, 0), R2(2, 1), R2(2, 2), T1(2) );

//    // R2 T2
//    this->projectionRight =( cv::Mat_<double>(3, 4) <<
//                             R2(0, 0), R2(0, 1), R2(0, 2), T2(0),
//                             R2(1, 0), R2(1, 1), R2(1, 2), T2(1),
//                             R2(2, 0), R2(2, 1), R2(2, 2), T2(2) );

    CheckCoherentRotation(R2);
    SaveCamera(R1, T2);
}

void SFM::TriangulatePointsOpenCV()
{
    cv::triangulatePoints( this->projectionLeft, 
                           this->projectionRight, 
                           this->leftPoints, 
                           this->rightPoints, 
                           this->model );
    
    std::vector<cv::Mat> splitted = { this->model.row(0) / this->model.row(3),
                                      this->model.row(1) / this->model.row(3),
                                      this->model.row(2) / this->model.row(3) };
    
    cv::merge(splitted, this->model);
}

cv::Mat SFM::getModel()
{
    return this->model;
}

void SFM::printModel()
{
    std::cout << "ply" << std::endl;
    std::cout << "format ascii 1.0" << std::endl;
    std::cout << "element vertex  " << this->model.cols << std::endl;
    std::cout << "property float x" << std::endl;
    std::cout << "property float y" << std::endl;
    std::cout << "property float z" << std::endl;
    std::cout << "property uchar diffuse_blue" << std::endl;
    std::cout << "property uchar diffuse_green" << std::endl;
    std::cout << "property uchar diffuse_red" << std::endl;
    std::cout << "end_header" << std::endl;

    //int mul = 10000;
    int mul = 1;

    for (int i = 0; i < this->model.cols; i++) {
        cv::Mat m = this->model.col(i);

       // std::cout << int(mul*m.at<float>(0)) << " "
       //           << int(mul*m.at<float>(1)) << " " 
       //           << int(mul*m.at<float>(2)) << " ";

        std::cout << (mul*m.at<float>(0)) << " "
                  << (mul*m.at<float>(1)) << " " 
                  << (mul*m.at<float>(2)) << " ";

        std::cout << (int)this->colorPoints.at(i)[0] << " " <<
                     (int)this->colorPoints.at(i)[1] << " " <<
                     (int)this->colorPoints.at(i)[2] << std::endl;
    }
}

cv::Mat_<double> SFM::
LinearLSTriangulation( cv::Point3d u,  //homogenous image point (u,v,1)
                       cv::Matx34d P,  //camera 1 matrix
                       cv::Point3d u1, //homogenous image point in 2nd camera
                       cv::Matx34d P1  //camera 2 matrix
                     )
{
    //build A matrix
    cv::Matx43d A( u.x*P(2,0)-P(0,0), u.x*P(2,1)-P(0,1), u.x*P(2,2)-P(0,2),
                   u.y*P(2,0)-P(1,0), u.y*P(2,1)-P(1,1), u.y*P(2,2)-P(1,2),
                   u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1), u1.x*P1(2,2)-P1(0,2),
                   u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1), u1.y*P1(2,2)-P1(1,2) );

    //build B vector
    cv::Matx41d B( -(u.x*P(2,3)   - P(0,3)),
                   -(u.y*P(2,3)   - P(1,3)),
                   -(u1.x*P1(2,3) - P1(0,3)),
                   -(u1.y*P1(2,3) - P1(1,3)) );

    //solve for X
    cv::Mat_<double> X;
    solve(A, B, X, cv::DECOMP_SVD);

    return X;
}

double SFM::TriangulatePoints()
{
    //std::vector<double> reproj_error;
    cv::Mat Kinv = this->intrinsics.inv(cv::DECOMP_SVD);

    std::vector<cv::Point3d> pointcloud;

    int pts_size = leftPoints.size();
    std::cout << pts_size << std::endl;

    this->model = cv::Mat(cv::Size(1, pts_size), CV_64FC3);

    for (int i=0; i < pts_size; ++i) {
        //convert to normalized homogeneous coordinates
        cv::Point2f kp = this->leftPoints[i];
        //cv::Point2f kp = pt_set1[i].pt;
        cv::Point3d u(kp.x, kp.y,1.0);
        cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
        u = um.at<cv::Point3d>(0);

        cv::Point2f kp1 = this->rightPoints[i];
        //cv::Point2f kp1 = pt_set2[i].pt;
        cv::Point3d u1(kp1.x,kp1.y,1.0);
        cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
        u1 = um1.at<cv::Point3d>(0);

        //triangulate
        cv::Mat_<double> X = LinearLSTriangulation( u, this->projectionLeft, 
                                                   u1, this->projectionRight);
        //cv::Mat_<double> X =LinearLSTriangulation(u, P, u1, P1);

        //calculate reprojection error
        //Mat_<double> xPt_img = K * Mat(P1) * X;
        //Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
        //reproj_error.push_back(norm(xPt_img_-kp1));

        //store 3D point
        //pointcloud.push_back(cv::Point3d(X(0), X(1), X(2)));

        cv::Mat m = cv::Mat(cv::Size(1,1), CV_64FC3);
        if (m.cols == 3) {
            m.at<double>(0) = X(0);
            m.at<double>(1) = X(1);
            m.at<double>(2) = X(2);
            this->model.push_back(m);
        }
    }

    cv::transpose(this->model, this->model);
    //std::cout << this->model.rows << " " << this->model.cols << std::endl;
    //std::cout << this->model.channels() << std::endl;
    //exit(1);
    
    //return mean reprojection error
    //cv::Scalar me = cv::mean(reproj_error);
    return 1.1;
    //return me[0];
}

void SFM::AlignPoints( const std::vector<cv::KeyPoint> & kpts1,
                       const std::vector<cv::KeyPoint> & kpts2,
                       const std::vector<cv::DMatch>   & matches,
                       std::vector<cv::KeyPoint>       & kpts1_tmp,
                       std::vector<cv::KeyPoint>       & kpts2_tmp ) 
{
	if (matches.empty()) { //points already aligned
		kpts1_tmp = kpts1;
		kpts2_tmp = kpts2;
	} 
    else {
        for (int i=0; i<matches.size(); ++i) {
            assert(matches[i].queryIdx < kpts1.size());
            kpts1_tmp.push_back(kpts1[matches[i].queryIdx]);

            assert(matches[i].trainIdx < kpts2.size());
            kpts2_tmp.push_back(kpts2[matches[i].trainIdx]);
        }   
    }   
}

/*
 * Convert vector of OpenCV types KeyPoint to Point2f.
 */
void KeyPointsToPoints( const std::vector<cv::KeyPoint> & kps, 
                        std::vector<cv::Point2f> & ps ) 
{
    ps.clear();

    for (int i=0; i<kps.size(); ++i) 
        ps.push_back(kps[i].pt);
}

std::vector<cv::DMatch> SFM::
EliminateMatches( const std::vector<cv::KeyPoint> & kpts1,
                  const std::vector<cv::KeyPoint> & kpts2,
                  std::vector<cv::KeyPoint>       & kpts1_good,
                  std::vector<cv::KeyPoint>       & kpts2_good,
                  std::vector<cv::DMatch>         & matches ) 
{
    // TODO delete
	//Try to eliminate keypoints based on the fundamental matrix
	//(although this is not the proper way to do this)
	
    std::vector<cv::KeyPoint> kpts1_tmp;
    std::vector<cv::KeyPoint> kpts2_tmp;
    std::vector<cv::DMatch> new_matches;

	kpts1_good.clear(); 
    kpts2_good.clear();

    AlignPoints( kpts1, kpts2, matches, kpts1_tmp, kpts2_tmp );
	
    std::vector<uchar> status = GetUsedMatches( kpts1_tmp, kpts2_tmp );

    std::cout << "F keeping " << cv::countNonZero(status) 
              << " / " << status.size() << std::endl;	

	for (int i=0; i < status.size(); i++) {
		if (status[i]) {
			kpts1_good.push_back(kpts1_tmp[i]);
			kpts2_good.push_back(kpts2_tmp[i]);

			if (matches.empty()) //points already aligned
				new_matches.push_back(cv::DMatch( matches[i].queryIdx,
                                                  matches[i].trainIdx,
                                                  matches[i].distance) );
			else
				new_matches.push_back(matches[i]);
		}
	}	
	
    std::cout << matches.size() << " matches before, " 
              << new_matches.size() << " new matches after Fundamental Matrix\n";

    // TODO rewrite?
    //keep only those points who survived the fundamental matrix
	//matches = new_matches; 
	
	return new_matches;
}

/**
 * Retrieve matches which were used to compute fundamental matrix.
 */
std::vector<uchar> SFM::
GetUsedMatches( std::vector<cv::KeyPoint> kpts1,
                std::vector<cv::KeyPoint> kpts2 )
{
    std::vector<uchar> status(kpts1.size());
    std::vector<cv::Point2f> pts1,pts2;
    double minVal, 
           maxVal;

    KeyPointsToPoints(kpts1, pts1);
    KeyPointsToPoints(kpts2, pts2);

    cv::minMaxIdx(pts1, &minVal, &maxVal);

    //threshold from [Snavely07 4.1]
    cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.006 * maxVal, 0.99, status); 

    return status;
}

bool SFM::CheckCoherentRotation(cv::Mat_<double> & R)
{
    if (fabsf(cv::determinant(R)) - 1.0 > 1e-07) {
        std::cerr << "Rotation matrix is not coherent!" << std::endl;
        return false;
    }

    return true;
}

int main(int argc, const char * argv[]) {
    // soldier
    //Mat img1 = imread( "soldier/visualize/02.jpg" );
    //Mat img2 = imread( "soldier/visualize/03.jpg" );
    //Mat cam_matrix = (Mat_<double>(3, 3) <<
    //                  6704.926882, 0,           738.251932,
    //                  0,           6705.241311, 757.560286,
    //                  0,           0,           1);
    //
    //Mat dist_coeff = (Mat_<double>(1, 5) << 
    //                  -0.125368, -0.097388, -0.003711, -0.000161, 0.000000);

    // DINO
    cv::Mat imgLeft = cv::imread("dino/visualize/00.jpg");
    cv::Mat imgRight = cv::imread("dino/visualize/01.jpg");
    std::string intrinsics = "dino-intrinsics.yml";
    std::string distortion = "dino-distortion.yml";

    // TEMPLE
    //cv::Mat imgLeft = cv::imread("temple/templeSR0001.png");
    //cv::Mat imgRight = cv::imread("temple/templeSR0002.png");
    //std::string intrinsics = "temple-intrinsics.yml";
    //std::string distortion = "temple-distortion.yml";

    // FOUNTAIN
    //cv::Mat imgLeft  = cv::imread("fountain_dense/urd/0001.png");
    //cv::Mat imgRight = cv::imread("fountain_dense/urd/0002.png");
    //std::string intrinsics = "fountain-intrinsics.yml";
    //std::string distortion = "fountain-distortion.yml";

    SFM sfm = SFM(imgLeft, imgRight, intrinsics, distortion);
    sfm.printModel();
    
    return 0;
}
