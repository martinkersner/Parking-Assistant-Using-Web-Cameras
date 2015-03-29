/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Calculation of disparity map.
 *
 * m.kersner@gmail.com
 * 03/15/2015
 */

#include "DisparityMap.h"

DisparityMap::DisparityMap()
{
    InitializeStereoProperties();
}

// assume we exploit cvFindStereoCorrespondence
void DisparityMap::InitializeStereoProperties()
{
	this->sbm.state->preFilterSize 		 = 5;
	this->sbm.state->preFilterCap 		 = 1;
	this->sbm.state->SADWindowSize 		 = 5;
	this->sbm.state->minDisparity 		 = 0;
	this->sbm.state->numberOfDisparities = 64;
	this->sbm.state->textureThreshold 	 = 0;
	this->sbm.state->uniquenessRatio 	 = 0;
	this->sbm.state->speckleWindowSize 	 = 0;
	this->sbm.state->speckleRange		 = 0;
}

void DisparityMap::SetPreFilterSize( int value )
{
	this->sbm.state->preFilterSize = value;
}

void DisparityMap::SetPreFilterCap( int value )
{
	this->sbm.state->preFilterCap = value;
}

void DisparityMap::SetSATWindowSize( int value )
{
	this->sbm.state->SADWindowSize = value;
}

void DisparityMap::SetMinDisparity( int value )
{
	this->sbm.state->minDisparity = value;
}

void DisparityMap::SetNumberOfDisparities( int value )
{
	this->sbm.state->numberOfDisparities = value;
}

void DisparityMap::SetTextureThreshold( int value )
{
	this->sbm.state->textureThreshold = value;
}

void DisparityMap::SetUniquenessRatio( int value )
{
	this->sbm.state->uniquenessRatio = value;
}

void DisparityMap::SetSpeckleWindowSize( int value )
{
	this->sbm.state->speckleWindowSize = value;
}

void DisparityMap::SetSpeckleRange( int value )
{
	this->sbm.state->speckleRange = value;
}

cv::Mat DisparityMap::CalculateDisparity( cv::Mat & left,
                                          cv::Mat & right )
{
    cv::Mat disparity,
            left_8UC1, right_8UC1;

    // channels reduction
    if (left.channels() > 1) {
        cvtColor(left, left_8UC1, CV_BGR2GRAY);
        cvtColor(right, right_8UC1, CV_BGR2GRAY);
    }
    else {
        left_8UC1  = left;
        right_8UC1 = right;
    }

    // calculation disparity 
    this->sbm(left_8UC1, right_8UC1, disparity);

    return disparity;
}
