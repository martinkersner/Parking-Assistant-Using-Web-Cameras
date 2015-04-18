/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Detection of the closest objects in disparity map.
 *
 * m.kersner@gmail.com
 * 03/27/2015
 */

#include "ObjectDetection.h"

// this constructor is created only in order to posses class without initialization
ObjectDetection::ObjectDetection()
{}

ObjectDetection::ObjectDetection( cv::Mat & _background ) : background(_background)
{}

Object ObjectDetection::Detect( cv::Mat & scene )
{
    Object object;
    cv::Mat mask = ((scene - this->background) > this->threshold) / 255.0;

    cv::Mat obstacles = mask.mul(scene);
    cv::Mat hist = ComputeHistogram(obstacles);
    cv::Mat modHist = ModifyHistogram(hist);
    object = ClosestObject(modHist, obstacles);

    return object;
}

cv::Mat ObjectDetection::ComputeHistogram( cv::Mat & src )
{
    cv::Mat hist;
    float range[] = {0, float(this->numberValues)};
    const float* histRange = { range };
    bool uniform    = true; 
    bool accumulate = false;

    cv::calcHist( &src, 1, 0, cv::Mat(), hist, 1, &(this->histSize), &histRange, 
                  uniform, accumulate );

    hist = HistogramPostprocessing(hist);

    return hist;
}

cv::Mat ObjectDetection::HistogramPostprocessing ( cv::Mat & hist ) 
{
    hist.convertTo(hist, CV_16UC1);
    ClearHistogram(hist, 1);

    return hist;
}

cv::Mat ObjectDetection::ModifyHistogram( cv::Mat & hist ) 
{
    cv::Mat modHist = hist.clone();
    cv::Mat posMask, tmpMask;

    modHist -= this->minBase;
    posMask = (modHist > 0) / 255.0;
    posMask.convertTo(posMask, CV_16UC1);

    tmpMask = hist.mul(posMask);

    return tmpMask;
}

int ObjectDetection::BinId2Intensity( int binId )
{
    return binId * this->binWidth;
}

void ObjectDetection::ClearHistogram( cv::Mat & hist,
                                      int offset )
{
    for (int i = this->latestOffset; i < offset; ++i ) // exclusive right boundary
        hist.at<unsigned short>(i) = 0;

    this->latestOffset = offset;
}

Object ObjectDetection::ClosestObject ( cv::Mat & hist,
                                        cv::Mat & obstacles ) // debugging purposes
{
    int upperBin, lowerBin;
    Object object;

    // searching for upper boundary
    int i;
    for (i = hist.rows-1; i >= 0; --i) {
        if (hist.at<unsigned short>(i) > 0) {
            upperBin = i+1;
            break;
        }
    }

    // not found
    if (i == 0)
        return object;

    // searching for lower boundary
    for (int j = i; j >= 0; --j) {
        if (hist.at<unsigned short>(j) <= 0) {
            lowerBin = j+1;
            break;
        }
    }

    object.found = true;
    object.mask = GetObject( obstacles, 
                             BinId2Intensity(lowerBin), BinId2Intensity(upperBin) );

    return object;
}

// AUXILIARY METHODS ///////////////////////////////////////////////////////////
cv::Mat ObjectDetection::GetObject( cv::Mat & src, 
                                 float lower,
                                 float upper )
{
    cv::Mat maskObject;
    cv::Mat maskLower = (src > lower) / 255.0;
    cv::Mat maskUpper = (src < upper) / 255.0;

    cv::Mat mask   = maskLower.mul(maskUpper);
    cv::Mat object = src.mul(mask);

    cv::erode(object, object, cv::Mat(), cv::Point(-1, -1), 4, 1, 1);
    maskObject = (object > 0) / 255.0;

    return maskObject;
}

void ObjectDetection::DisplayHistogram ( cv::Mat & hist )
{
    cv::Mat tmpHist = hist.clone();
    cv::normalize(tmpHist, tmpHist, 0, this->histHeight, cv::NORM_MINMAX, -1, cv::Mat());

    int binWidth = cvRound((double) this->histWidth/this->histSize);
    cv::Mat histImage(this->histHeight, this->histWidth, CV_8UC3, cv::Scalar(0,0,0));

    for (int i = 1; i < this->histSize; i++) {
        line( histImage, 
              cv::Point( binWidth*(i-1), this->histHeight - cvRound(tmpHist.at<float>(i-1)) ),
              cv::Point( binWidth*(i),   this->histHeight - cvRound(tmpHist.at<float>(i)) ),
              cv::Scalar(255, 0, 0), 
              2, 8, 0 );
    }

    cv::imshow("Histogram", histImage);
    cv::waitKey(0);
}
