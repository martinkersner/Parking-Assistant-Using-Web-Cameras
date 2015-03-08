/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Optical Flow
 *
 * m.kersner@gmail.com
 * 02/21/2015
 */

#include "OpticalFlow.h"

int main() {
    cv::Mat first, second, 
            grayFirst, graySecond,
            flow, groundTruth, mask;

    std::string firstPath, secondPath,
                gtPath;
    
    std::vector<dataset_s> dataset;
    dataset_s datasetMiddlebury = { "../img/Middlebury/",
                                    { 
                                     "Dimetrodon",
                                     "Grove2",
                                     "Grove3",
                                     "Hydrangea",
                                     "RubberWhale",
                                     "Urban2",
                                     "Urban3",
                                     "Venus" 
                                    } 
                                  };

    dataset_s datasetUCL = { "../img/UCL/",
                                   { "009_Crates1",
                                     "010_Crates2",
                                     "013_Mayan1",
                                     "014_Mayan2",
                                     "015_YoesmiteSun",
                                     "016_GroveSun",
                                     "017_Robot",
                                     "018_Sponza1",
                                     "019_Sponza2",
                                     "022_Crates1Htxtr2",
                                     "024_Crates2Htxtr1",
                                     "026_Brickbox1t1",
                                     "029_Brickbox2t2",
                                     "030_GrassSky0",
                                     "039_GrassSky9",
                                     "049_TxtRMovement",
                                     "050_TxtLMovement",
                                     "051_blow1Txtr1",
                                     "088_blow19Txtr2",
                                     "089_drop1Txtr1",
                                     "106_drop9Txtr2",
                                     "107_roll1Txtr1",
                                     "124_roll9Txtr2",
                                     "125_street1Txtr1" } };

    dataset.push_back(datasetMiddlebury);
    //dataset.push_back(datasetUCL);

    for (dataset_s d : dataset) {
        std::cout << d.name << std::endl;

        for (std::string s : d.subdatasets) {
            std::cout << s << std::endl;
            firstPath    = d.name + s + std::string("/1.png");
            secondPath   = d.name + s + std::string("/2.png");
            gtPath       = d.name + s + std::string("/1_2.flo");

            first  = cv::imread(firstPath);
            second = cv::imread(secondPath);
            cv::cvtColor(first,  grayFirst,  cv::COLOR_BGR2GRAY);
            cv::cvtColor(second, graySecond, cv::COLOR_BGR2GRAY);

            groundTruth = cv::optflow::readOpticalFlow(gtPath);

            // TODO find better way
            /*
            // DENSE FLOW
            flow = CalculateDenseFlow(grayFirst, graySecond);
            std::cout << "AE" << std::setw(17) 
                      << MeanAngularError(flow, groundTruth) << std::endl;
            std::cout << "L1" << std::setw(17) 
                      << MeanL1EndpointError(flow, groundTruth) << std::endl;
            std::cout << "L2" << std::setw(17) 
                      << MeanL2EndpointError(flow, groundTruth) << std::endl << std::endl;
            */

            ///*
            // SPARSE FLOW
            CalculateSparseFlow(grayFirst, graySecond, flow, mask);
            std::cout << "AE" << std::setw(17) 
                      << SparseAngularError(flow, groundTruth, mask) << std::endl;
            std::cout << "L1" << std::setw(17) 
                      << SparseL1EndpointError(flow, groundTruth, mask) << std::endl;
            std::cout << "L2" << std::setw(17) 
                      << SparseL2EndpointError(flow, groundTruth, mask) << std::endl << std::endl;
            //*/

            /*
            // for drawing purposes, white canvas
            cv::Mat white = cv::Mat(first.rows, first.cols, CV_8UC3, cv::Scalar(255, 255, 255));
            DrawOpticalFlow(flow, white, 10, cv::Scalar(0, 0, 0));
            cv::Mat color = DrawOpticalFlowColor(flow);
            cv::imshow("colors", color);
            cv::imshow("lines", white);
            cv::waitKey(0);
            */
        }
    }

    // write optical flow
    //cv::optflow::writeOpticalFlow("out.flo", flow);
}

void DrawOpticalFlow( const cv::Mat & flowX, 
                      const cv::Mat & flowY, 
                      cv::Mat & img, 
                      int step, 
                      const cv::Scalar & color ) 
{
    for(int y = 0; y < img.rows; y += step)
        for(int x = 0; x < img.cols; x += step)
        {
            cv::Point2f fxy; 
            fxy.x = std::round( flowX.at<float>(y, x) + x );
            fxy.y = std::round( flowY.at<float>(y, x) + y );

            cv::line(img, cv::Point(x,y), cv::Point(fxy.x, fxy.y), color);
            cv::circle(img, cv::Point(fxy.x, fxy.y), 1, color, -1);
        }
}

void DrawOpticalFlow( const cv::Mat & flow, 
                      cv::Mat & img, 
                      int step, 
                      const cv::Scalar & color ) 
{
    cv::Mat flowX, flowY;
    std::vector<cv::Mat> flowXY(2);
    cv::split(flow, flowXY);
    flowX = flowXY[0];
    flowY = flowXY[1];

    DrawOpticalFlow(flowX, flowY, img, step, color);
}

cv::Mat DrawOpticalFlowColor( const cv::Mat & flowX, 
                              const cv::Mat & flowY )
{
    cv::Mat hsv, rgb,
            h, s, v,
            magnitude, angle;

    std::vector<cv::Mat> hsvChannels;

    h   = cv::Mat(flowX.rows, flowX.cols, CV_32FC1);
    s   = cv::Mat(flowX.rows, flowX.cols, CV_32FC1, 255);
    v   = cv::Mat(flowX.rows, flowX.cols, CV_32FC1);
    hsv = cv::Mat(flowX.rows, flowX.cols, CV_32FC3);

    cv::cartToPolar(flowX, flowY, magnitude, angle);

    // H
    h = angle*90/M_PI;
    h.convertTo(h, CV_32F);

    // V
    cv::normalize(magnitude, v, 0, 255, cv::NORM_MINMAX, CV_32FC1);

    // HSV
    hsvChannels.push_back(h);
    hsvChannels.push_back(s);
    hsvChannels.push_back(v);
    merge(hsvChannels, hsv);

    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

    // additional normalization
    rgb.convertTo(rgb, CV_8UC1);

    return rgb;
}

cv::Mat DrawOpticalFlowColor( const cv::Mat & flow )
{
    cv::Mat flowX, flowY;
    std::vector<cv::Mat> flowXY(2);
    cv::split(flow, flowXY);
    flowX = flowXY[0];
    flowY = flowXY[1];

    return DrawOpticalFlowColor(flowX, flowY);
}

cv::Mat AngularError( cv::Mat & flow,
                      cv::Mat & groundTruth )
{
    cv::Mat_<float> angErr = cv::Mat_<float>::zeros(flow.rows, flow.cols);
    cv::Point2f p1, p2;
    float tmp;

    for (int i = 0; i < flow.rows; ++i) {
        for (int j = 0; j < flow.cols; ++j) {

            p1 = flow.at<cv::Point2f>(i,j);
            p2 = groundTruth.at<cv::Point2f>(i,j);
            tmp = MultVec3x1(p1, p2) / (MagVec3x1(p1) * MagVec3x1(p2));
            
            angErr.at<float>(i,j) = acos(tmp);
        }
    }

    return angErr;
}

float SparseAngularError( const cv::Mat & flow,
                          const cv::Mat & groundTruth,
                          const cv::Mat & mask )
{
    cv::Mat maskedGroundTruth;
    cv::Mat maskedFlow;
    int number = cv::sum(mask)[0];

    groundTruth.copyTo(maskedGroundTruth, mask);
    flow.copyTo(maskedFlow, mask);

    cv::Mat ae = AngularError(maskedFlow, maskedGroundTruth);

    return float(cv::sum(ae)[0])/number;
}


float MeanAngularError( cv::Mat & flow,
                        cv::Mat & groundTruth )
{
   return cv::mean(AngularError(flow, groundTruth))[0];
}

cv::Mat L1EndpointError( cv::Mat & flow,
                         cv::Mat & groundTruth )
{
    cv::Mat_<double> endErr = cv::Mat_<double>(flow.rows, flow.cols);
    //cv::Point_<double> p1, p2;
    cv::Point2f p1, p2;
    double tmp,
           tmpX, tmpY,
           px1, px2,
           py1, py2;

    for (int i = 0; i < flow.rows; ++i) {
        for (int j = 0; j < flow.cols; ++j) {

            p1 = flow.at<cv::Point2f>(i,j);
            p2 = groundTruth.at<cv::Point2f>(i,j);
            
            px1 = p1.x;
            px2 = p2.x;

            py1 = p1.y;
            py2 = p2.y;

            tmpX = std::abs(px1-px2);
            tmpY = std::abs(py1-py2);
            tmp = tmpX + tmpY;
            
            if (p2.x < OCCLUDED || p2.y < OCCLUDED)
                endErr.at<double>(i,j) = tmp;
            else
                endErr.at<double>(i,j) = 0;
        }
    }

    return endErr;
}

float SparseL1EndpointError( const cv::Mat & flow,
                             const cv::Mat & groundTruth, 
                             const cv::Mat & mask )
{
    cv::Mat maskedGroundTruth;
    cv::Mat maskedFlow;
    int number = cv::sum(mask)[0];

    groundTruth.copyTo(maskedGroundTruth, mask);
    flow.copyTo(maskedFlow, mask);

    cv::Mat ae = L1EndpointError(maskedFlow, maskedGroundTruth);

    return float(cv::sum(ae)[0])/number;
}

float MeanL1EndpointError( cv::Mat & flow,
                           cv::Mat & groundTruth )
{
    return cv::mean(L1EndpointError(flow, groundTruth))[0];
}

cv::Mat L2EndpointError( cv::Mat & flow,
                         cv::Mat & groundTruth )
{
    cv::Mat_<double> endErr = cv::Mat_<double>(flow.rows, flow.cols);
    cv::Point_<double> p1, p2;
    double tmp, 
           tmpX, tmpXX,
           tmpY, tmpYY;

    for (int i = 0; i < flow.rows; ++i) {
        for (int j = 0; j < flow.cols; ++j) {

            p1 = flow.at<cv::Point2f>(i,j);
            p2 = groundTruth.at<cv::Point2f>(i,j);

            tmpX = abs(p1.x-p2.x);
            tmpXX = tmpX*tmpX;

            tmpY = abs(p1.y-p2.y);
            tmpYY = tmpY*tmpY;

            tmp = tmpXX + tmpYY;
            
            if (p2.x < OCCLUDED || p2.y < OCCLUDED)
                endErr.at<double>(i,j) = sqrt(tmp);
            else
                endErr.at<double>(i,j) = 0;
        }
    }

    return endErr;
}

float SparseL2EndpointError( const cv::Mat & flow,
                             const cv::Mat & groundTruth, 
                             const cv::Mat & mask )
{
    cv::Mat maskedGroundTruth;
    cv::Mat maskedFlow;
    int number = cv::sum(mask)[0];

    groundTruth.copyTo(maskedGroundTruth, mask);
    flow.copyTo(maskedFlow, mask);

    cv::Mat ae = L2EndpointError(maskedFlow, maskedGroundTruth);

    return float(cv::sum(ae)[0])/number;
}

float MeanL2EndpointError( cv::Mat & flow,
                             cv::Mat & groundTruth )
{
    return cv::mean(L2EndpointError(flow,groundTruth))[0];
}

double MultVec3x1( cv::Point p1,
                   cv::Point p2 ) 
{
    double px1, py1,
           px2, py2;

    px1 = p1.x;
    py1 = p1.y;

    px2 = p2.x;
    py2 = p2.y;

    return px1*px2 + py1*py2 + 1.0;
}

double MagVec3x1( cv::Point p )
{
    double px, py;
    px = p.x;
    py = p.y;

    return sqrt(px*px + py*py + 1.0);
}

void CalculateSparseFlow ( const cv::Mat & first,
                           const cv::Mat & second,
                           cv::Mat & flow,
                           cv::Mat & mask )
{
    cv::Size size = first.size();

    std::vector<cv::Point2f> featuresLeft;
    std::vector<cv::Point2f> featuresRight;

    // FEATURE DETECTION
    goodFeaturesToTrack( first, featuresLeft, 
                         5000, // maxCorners
                         0.01, // qualityLevel
                         3 );  // minDistance, 
                         //InputArray mask=noArray(), 
                         //int blockSize=3, 
                         //bool useHarrisDetector=false, 
                         //double k=0.04 );

    // OPTICAL FLOW
    std::vector<uchar> status;
    cv::Mat err;
    int winSize = 40;

    calcOpticalFlowPyrLK( first, second, 
                          featuresLeft, featuresRight, 
                          status, err, 
                          cv::Size(winSize, winSize), 
                          3,      // max level
                          cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
                                            30, 0.01), 
                          0,      // flags 
                          1e-4 ); // minEigThreshold

    SparseFlowToMat( featuresLeft, featuresRight, status,
                     size,
                     flow, mask );
}

inline bool CorrectFlowEstimation( uchar fe )
{
    if (fe == CORRECT_FLOW_ESTIMATION)
        return true;
    else
        return false;
}

inline void CalculateParticularFlow( const cv::Point2f & firstFeature,
                                     const cv::Point2f & secondFeature,
                                     cv::Point2f & flow )
{
    flow.x = secondFeature.x - firstFeature.x;
    flow.y = secondFeature.y - firstFeature.y;
}

void SparseFlowToMat( const std::vector<cv::Point2f> & firstFeatures,
                      const std::vector<cv::Point2f> & secondFeatures,
                      const std::vector<uchar> & status,
                      cv::Size & size,
                      cv::Mat & flow,
                      cv::Mat & mask )
{
    int x, y;
    cv::Point2f pixelFlow;
    cv::Mat tmpFlow = cv::Mat::zeros(size, CV_32FC2);
    cv::Mat tmpMask = cv::Mat::zeros(size, CV_8UC1);

    for (int i = 0; i < status.size(); ++i) {
        if (CorrectFlowEstimation(status.at(i))) {// && err.at<float>(y,x) < 100) {
            x = secondFeatures.at(i).x;
            y = secondFeatures.at(i).y;

            // mask
            tmpMask.at<uchar>(y, x) = 1;

            // flow
            CalculateParticularFlow(firstFeatures.at(i), secondFeatures.at(i), pixelFlow);
            
            tmpFlow.at<cv::Vec2f>(y, x)[0] = pixelFlow.x;
            tmpFlow.at<cv::Vec2f>(y, x)[1] = pixelFlow.y;
        }
    }

    flow = tmpFlow; 
    mask = tmpMask;
}

cv::Mat CalculateDenseFlow( const cv::Mat & first,
                            const cv::Mat & second )
{
    cv::Mat flow;

    cv::calcOpticalFlowFarneback( first, second, flow,
                                  0.5,   // pyramide scale
                                  4,     // levels 
                                  20,    // winsize
                                  30,    // iteration
                                  7,     // poly_n
                                  1.5,   // poly_sigma
                                  0 );   // flags

    return flow;
}
