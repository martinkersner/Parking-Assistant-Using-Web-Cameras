/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Detection of curbs in RGB images.
 *
 * m.kersner@gmail.com
 * 04/11/2015
 */

#include "CurbDetection.h"

const float CurbDetection::NO_ANGLE = 999;

CurbDetection::CurbDetection( bool _demo ) : demo(_demo)
{}

CurbDetection::~CurbDetection()
{}

Curb CurbDetection::Detect( cv::Mat & src ) {
    Curb detResult;
    VecVecPoint curb;
    cv::Mat dst, edges, gray;

    if (this->demo)
        src = DownSample(src, this->resizeFactor); 

    // preprocessing
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    cv::equalizeHist(gray, dst);
    RepeatBlurImage(dst, dst, this->repeatBlur);
    DetectEdges(dst, edges);

    // processing
    curb = DetectCurb(edges, dst);

    if (this->demo) {
        cv::Mat detectedCurb = src.clone();
        detectedCurb = DrawCurb(detectedCurb, curb);
        cv::imshow("Curb Detection", detectedCurb);
        cv::waitKey();
    }

    if (curb.size() > 0)
        detResult.found = true;

    detResult.input  = edges;
    detResult.output = dst;
    detResult.curb   = curb;

    return detResult;
}

cv::Mat CurbDetection::DrawCurb( cv::Mat & src, VecVecPoint & curb ) 
{
    cv::Mat dst = src.clone();

    for (int i = 0; i < curb.size(); ++i) {
        line( dst, 
              curb.at(i).at(0), curb.at(i).at(1), 
              cv::Scalar(255, 255, 255), 2, CV_AA );
    }

    return dst;
}

cv::Mat CurbDetection::DownSample( const cv::Mat src, double factor )
{
    cv::Mat dst;

    factor = 1/factor;
    resize(src, dst, cv::Size(0, 0), factor, factor, cv::INTER_LINEAR);

    return dst;
}

// assumes that the first point in vector lays in the left side, thus the second
// point lays in the right side
bool CurbDetection::AllowedDistance( VecPoint l0, VecPoint l1 )
{
    // retrieve end points from lines
    cv::Point lPt0 = l0.at(0);
    cv::Point rPt0 = l0.at(1);

    cv::Point lPt1 = l1.at(0);
    cv::Point rPt1 = l1.at(1);

    // measure distances
    float leftPointDistance = std::sqrt( std::pow(std::abs(lPt0.x-lPt1.x), 2) + 
                                         std::pow(std::abs(lPt0.y-lPt1.y), 2) );

    float rightPointDistance = std::sqrt( std::pow(std::abs(rPt0.x-rPt1.x), 2) + 
                                          std::pow(std::abs(rPt0.y-rPt1.y), 2) );

    if (leftPointDistance < this->maxMergeDist && rightPointDistance < this->maxMergeDist)
        return true;
    else
        return false;
}

VecPoint CurbDetection::Merge( VecVecPoint lines )
{
    VecPoint mergedLine,
             leftPoints, rightPoints;

    float leftX  = 0,  
          leftY  = 0,
          rightX = 0, 
          rightY = 0;
    int numberOfLines = lines.size();

    // divide to left and right points
    for (VecPoint l : lines) {
        if (l.at(0).x < l.at(1).x) {
            leftPoints.push_back(l.at(0));
            rightPoints.push_back(l.at(1));
        }
        else {
            leftPoints.push_back(l.at(1));
            rightPoints.push_back(l.at(0));
        }
    }

    // calculate average positions
    for (int i = 0; i < numberOfLines; ++i) {
        leftX += leftPoints.at(i).x;
        leftY += leftPoints.at(i).y;
        
        rightX += rightPoints.at(i).x;
        rightY += rightPoints.at(i).y;
    }

    leftX  /= numberOfLines;
    leftY  /= numberOfLines;
    rightX /= numberOfLines;
    rightY /= numberOfLines;

    // merged line
    mergedLine.push_back(cv::Point(leftX, leftY));
    mergedLine.push_back(cv::Point(rightX, rightY));

    return mergedLine;
}

bool CurbDetection::FindIntInVector( VecInt v, int i )
{
    if (std::find(v.begin(), v.end(), i) != v.end())
        return true;
    else
        return false;
}

VecPoint CurbDetection::ControlMerge( VecVecPoint & lines,
                                      int index,
                                      VecInt & usedIndices )
{
    VecVecPoint toMerge;

    bool found  = false;
    VecPoint l0 = lines.at(index);
    VecPoint l1;

    toMerge.push_back(l0);

    for (int i = index+1; i < lines.size(); ++i) {
        if (!FindIntInVector(usedIndices, i)) {
            l1 = lines.at(i);

            if (AllowedDistance(l0, l1)) {
                usedIndices.push_back(i);
                toMerge.push_back(l1);
            }
        }
    }

    // MERGE
    if (toMerge.size() == 1) // line is not supposed to merge
        return l0;
    else 
        return Merge(toMerge);
}

// method of stored lines changes in this method
// lines are not stored in Vec2f anymore
VecVecPoint CurbDetection::MergeSimilarLines( VecVecPoint & lines )
{
    VecVecPoint mergedLines;
    VecPoint line;
    std::vector<int> usedIndices;

    for(int i = 0; i < lines.size(); ++i) {
        if (!FindIntInVector(usedIndices, i)) {
            usedIndices.push_back(i);
            line = ControlMerge(lines, i, usedIndices);
            mergedLines.push_back(line);
        }
    }

    return mergedLines;
}

VecPoint CurbDetection::Vec4iLinesToPointLines ( const cv::Vec4i & vl,
                                                 const cv::Mat & src )
{
    cv::Vec4i v;
    cv::Point leftPoint, rightPoint;
    VecPoint line;

    v[0] = 0;
    v[1] = ((float)vl[1] - vl[3]) / (vl[0] - vl[2]) * -vl[0] + vl[1]; 
    v[2] = src.cols; 
    v[3] = ((float)vl[1] - vl[3]) / (vl[0] - vl[2]) * (src.cols - vl[2]) + vl[3];

    leftPoint  = cv::Point(v[0], v[1]);
    rightPoint = cv::Point(v[2], v[3]);

    line.push_back(leftPoint);
    line.push_back(rightPoint);

    return line;
}

VecPoint CurbDetection::HighestLine( const VecVecPoint & lines )
{
    VecPoint highestLine;

    if (lines.size() > 0) {
        highestLine = lines.at(0);
        for (VecPoint l : lines) {
            if (highestLine.at(0).y > l.at(0).y)
                highestLine = l;
        }
    }

    return highestLine;
}

void CurbDetection::ThreeLowestLines( const VecVecPoint & lines,
                                      VecPoint & first,
                                      VecPoint & second,
                                      VecPoint & third )
{
    float tmpY;
    VecPoint highestLine = HighestLine(lines);

    first  = highestLine;
    second = highestLine;
    third  = highestLine;

    for (VecPoint l : lines) {
        tmpY = l.at(0).y;

        if (tmpY > first.at(0).y) {
            third  = second;
            second = first;
            first  = l;
        }
        else if (tmpY > second.at(0).y) {
            third = second;
            second = l;
        }
        else if (tmpY > third.at(0).y) {
            third = l;
        }
    }
}

bool CurbDetection::IsAscending( const VecPoint & line )
{
    if (line.at(0).y < line.at(1).y)
        return true;
    else
        return false;
}

float CurbDetection::GetHypotenuse( const VecPoint & line )
{
    return std::sqrt( std::pow(std::abs(line.at(0).x-line.at(1).x), 2) + 
                      std::pow(std::abs(line.at(0).y-line.at(1).y), 2) );
}

float CurbDetection::GetAdjacentCathetus( const VecPoint & line )
{
    return std::abs(line.at(0).x - line.at(1).x);
}

float CurbDetection::GetLineAngle( VecPoint & line )
{
    float adjacentCathetus = GetAdjacentCathetus(line);
    float hypotenuse = GetHypotenuse(line);
    float angle = std::acos(adjacentCathetus/hypotenuse) * 180.0 / CV_PI;;
    bool ascending = IsAscending(line);
    
    if (ascending)
        return angle;
    else
        return -angle;
}

bool CurbDetection::EqualLines ( const VecPoint l0,
                                 const VecPoint l1 )
{
    if ( l0.at(0).x == l1.at(0).x && l0.at(0).y == l1.at(0).y && // left points
         l0.at(1).x == l1.at(1).x && l0.at(1).y == l1.at(1).y )  // right points
        return true;
    else
        return false;
}

bool CurbDetection::AbsoluteComparison( float f0,
                                        float f1,
                                        float degreeOfFreedom )
{
    if (std::abs(f0-f1) <= degreeOfFreedom)
        return true;
    else
        return false;
}

VecVecPoint CurbDetection::FindCurb( const VecVecPoint & lines )
{
    VecPoint firstLine, secondLine, thirdLine;
    VecVecPoint curb;

    int tmpCurbSize;
    int tmpLinesSize;
    float angleFirstLine  = this->NO_ANGLE, 
          angleSecondLine = this->NO_ANGLE, 
          angleThirdLine  = this->NO_ANGLE,
          tmpAngle        = this->NO_ANGLE;

    ThreeLowestLines(lines, firstLine, secondLine, thirdLine);

    tmpLinesSize = lines.size();

    if (tmpLinesSize > 2) { 
    // potential fully found curb

        // process the first and second line from bottom
        if (!EqualLines(firstLine, secondLine)) {
           angleFirstLine = GetLineAngle(firstLine);
           angleSecondLine = GetLineAngle(secondLine);

           if (AbsoluteComparison(angleFirstLine, angleSecondLine, this->maxAngleDiff))
           {
               curb.push_back(firstLine);
               curb.push_back(secondLine);
           }
        }
        else {
            curb.push_back(firstLine);
        }

        // process the second and third line from bottom
        int tmpCurbSize = curb.size();
        switch (tmpCurbSize) {
            case 1: 
                if (!EqualLines(thirdLine, curb.at(0))) {
                    tmpAngle = GetLineAngle(curb.at(0));

                    if (AbsoluteComparison(angleThirdLine, tmpAngle, this->maxAngleDiff))
                        curb.push_back(thirdLine);
                }
                break;

            case 2: 
                if (!EqualLines(thirdLine, secondLine)) {
                    angleThirdLine = GetLineAngle(thirdLine);

                    if (AbsoluteComparison(angleThirdLine, angleSecondLine, this->maxAngleDiff))
                        curb.push_back(thirdLine);
                }
                break;
        }
    }
    else if (tmpLinesSize == 2) {
        if (!EqualLines(firstLine, secondLine)) {
            angleFirstLine = GetLineAngle(firstLine);
            angleSecondLine = GetLineAngle(secondLine);

            if (AbsoluteComparison(angleFirstLine, angleSecondLine, this->maxAngleDiff)){
                curb.push_back(firstLine);
                curb.push_back(secondLine);
            }
        }
        else {
                curb.push_back(firstLine);
        }
    }
    else if (tmpLinesSize == 1) {
        curb.push_back(firstLine);
    }

    return curb;
}

VecVecPoint CurbDetection::DetectCurb( const cv::Mat & src, 
                                       cv::Mat & dst )
{
    VecVecPoint mergedLines, allLinesFromPoints, curb;
    std::vector<cv::Vec4i> lines;
    cv::Point pt1,    pt2,
              newPt1, newPt2;

    // find hough lines
    HoughLinesP(src, lines, 1, CV_PI/180, 150, 50, 150);

    // convert Hough lines to vector of points
    for (int i = 0; i < lines.size(); i++)
        allLinesFromPoints.push_back( Vec4iLinesToPointLines(lines[i], src) );

    // merge lines
    if (allLinesFromPoints.size() > 0) {
        mergedLines = MergeSimilarLines(allLinesFromPoints);
        curb = FindCurb(mergedLines);
    }

    return curb;
}

void CurbDetection::BlurImage( cv::Mat & src, cv::Mat & dst ) 
{
    medianBlur(src, dst, this->medianKernelSize);
}

void CurbDetection::DetectEdges( cv::Mat & src, cv::Mat & dst ) {
    cv::Canny( src, dst, 
               this->cannyLowThreshold, 
               this->cannyLowThreshold*this->cannyRatio, 
               this->cannyKernelSize );
}

void CurbDetection::RepeatBlurImage( const cv::Mat & src, 
                                     cv::Mat & dst, 
                                     int repetition )
{
    dst = src.clone();

    for (int i = 0; i < repetition; ++i) {
        BlurImage(dst, dst);
    }
}
