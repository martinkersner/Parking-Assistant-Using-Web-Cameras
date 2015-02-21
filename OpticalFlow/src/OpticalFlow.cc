#include "OpticalFlow.h"

int main() {
    cv::Mat first, second, 
            grayFirst, graySecond,
            flow, groundTruth;

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
            flow = CalculateFlow(grayFirst, graySecond);

            std::cout << "AE" << std::setw(17) 
                      << MeanAngularError(flow, groundTruth) << std::endl;
            std::cout << "L1" << std::setw(17) 
                      << MeanL1EndpointError(flow, groundTruth) << std::endl;
            std::cout << "L2" << std::setw(17) 
                      << MeanL2EndpointError(flow, groundTruth) << std::endl << std::endl;

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
    cv::Mat_<float> angErr = cv::Mat_<float>(flow.rows, flow.cols);
    cv::Point p1, p2;
    float tmp;

    for (int i = 0; i < flow.rows; ++i) {
        for (int j = 0; j < flow.cols; ++j) {

            p1 = flow.at<cv::Point>(i,j);
            p2 = groundTruth.at<cv::Point>(i,j);
            tmp = MultVec3x1(p1, p2) / (MagVec3x1(p1) * MagVec3x1(p2));
            
            angErr.at<float>(i,j) = acos (tmp);
        }
    }

    return angErr;
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
    cv::Point_<double> p1, p2;
    double tmp,
           tmpX, tmpY,
           px1, px2,
           py1, py2;

    for (int i = 0; i < flow.rows; ++i) {
        for (int j = 0; j < flow.cols; ++j) {

            p1 = flow.at<cv::Point>(i,j);
            p2 = groundTruth.at<cv::Point>(i,j);

            px1 = p1.x;
            px2 = p2.x;

            py1 = p1.y;
            py2 = p2.y;

            tmpX = std::abs(px1-px2);
            tmpY = std::abs(py1-py2);
            tmp = tmpX + tmpY;
            
            endErr.at<double>(i,j) = tmp;
        }
    }

    return endErr;
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

            p1 = flow.at<cv::Point>(i,j);
            p2 = groundTruth.at<cv::Point>(i,j);

            tmpX = abs(p1.x-p2.x);
            tmpXX = tmpX*tmpX;

            tmpY = abs(p1.y-p2.y);
            tmpYY = tmpY*tmpY;

            tmp = tmpXX + tmpYY;
            
            endErr.at<double>(i,j) = sqrt(tmp);
        }
    }

    return endErr;
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

cv::Mat CalculateFlow( const cv::Mat & first,
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
