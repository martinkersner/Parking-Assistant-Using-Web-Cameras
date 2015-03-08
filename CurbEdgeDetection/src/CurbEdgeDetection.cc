#include "CurbEdgeDetection.h"

std::vector<std::string> CreateImageNames(std::string path, int numberOfImages)
{
    std::vector<std::string> pathToImages;

    for (int i = 0; i < numberOfImages; ++i) {
        pathToImages.push_back(path+std::to_string(i)+std::string(".jpg"));
    }

    return pathToImages;
}


// clear freckles from detected edges
cv::Mat ClearFreckles(const cv::Mat & src) {
    cv::Mat srcTmp = src;
    cv::Mat dst = cv::Mat::zeros(srcTmp.rows, srcTmp.cols, srcTmp.type());;

    int area;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    //int chainType = CV_CHAIN_APPROX_SIMPLE;
    int chainType = CV_CHAIN_APPROX_NONE;
    findContours(srcTmp, contours, hierarchy, CV_RETR_TREE, chainType, cv::Point(0, 0));

    // information about contours
    int numberContours = contours.size();
    int remindedContours = 0;

    cv::Rect rect;
    int w, h;
    float bbFactorWidth  = 0.3*srcTmp.cols;
    float bbFactorHeight = 0.3*srcTmp.rows;

    // SOLIDITY
    int hullArea;
    float solidity;
    std::vector<cv::Point> hull;

    // find contours which represents paintings
    if(!contours.empty() && !hierarchy.empty()) {
        for (int i = 0; i < contours.size(); ++i) {

            /*
            rect = boundingRect(contours.at(i));
            w = rect.width;
            h = rect.height;

            std::cout << w << " " << h << std::endl;
            if (w < bbFactorWidth || h < bbFactorHeight)
                continue;
            */

            // AREA
            area = contourArea(contours.at(i));
            //std::cout << area << std::endl;
            //if (area < 5) // 700
            //    continue;

            // SOLIDITY
            /*
            convexHull(contours.at(i), hull, false);
            hullArea = contourArea(hull);
            solidity = float(area)/hullArea;
            if (solidity > 0.3)
                continue;
            */

            //std::cout << area << std::endl;

            drawContours(dst, contours, i, cv::Scalar::all(255), 1, 8);

            remindedContours++;
        }
    }

    std::cout << remindedContours << "/" << numberContours << std::endl;

    return dst;
}

cv::Mat DetectEdges(cv::Mat & src) {
  cv::Mat dst;

  // convert to gray
  cv::cvtColor(src, dst, CV_BGR2GRAY);

  // histogram equalization
  cv::equalizeHist(dst, dst);

  //blurimage(dst, dst, GAUSSIAN, 3);
  //BlurImage(dst, dst, BILATERAL, 11);
  int med = 11;
  Blur typeBlur = MEDIAN;
  BlurImage(dst, dst, typeBlur, med);
  BlurImage(dst, dst, typeBlur, med);
  BlurImage(dst, dst, typeBlur, med);

  //EdgeDetect(dst, dst, SOBEL);
  EdgeDetect(dst, dst, CANNY);

  //MorphologicalClosing(dst, dst);
  //dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), 1);

  return dst;
}

cv::Mat DownSample(const cv::Mat src, double factor)
{
    cv::Mat dst;

    factor = 1/factor;
    resize(src, dst, cv::Size(0, 0), factor, factor, cv::INTER_LINEAR);

    return dst;
}

void GetEdgePoints ( cv::Point & pt0,
                     cv::Point & pt1, 
                     cv::Point & newPt0,
                     cv::Point & newPt1,
                     cv::Mat & src )
{
    cv::Point edgeLeft, edgeRight;
    cv::LineIterator it(src, pt0, pt1, 8, true);

    newPt0 = it.pos();
    for (int i = 0; i < it.count; ++it, ++i) {} // iterate to the last item
    newPt1 = it.pos();
}

std::vector<cv::Point> HoughLineToPoints( cv::Vec2f line, 
                                          cv::Mat & border )
{
    std::vector<cv::Point> points;
    cv::Point pt0,    pt1,
              newPt0, newPt1;

    float rho   = line[0];
    float theta = line[1];

    double a = cos(theta);
    double b = sin(theta);

    double x0 = a*rho;
    double y0 = b*rho;

    int factor = 800;

    pt0.x = cvRound(x0 + factor*(-b));
    pt0.y = cvRound(y0 + factor*(a));
    pt1.x = cvRound(x0 - factor*(-b));
    pt1.y = cvRound(y0 - factor*(a));

    GetEdgePoints(pt0, pt1, newPt0, newPt1, border);

    // TODO delete
    /*
    std::cout << "[" << pt0.x    << ", " << pt0.y << "]" << " >> "
              << "[" << newPt0.x << ", " << newPt0.y << "]" << std::endl;

    std::cout << "[" << pt1.x    << ", " << pt1.y << "]" << " >> "
              << "[" << newPt1.x << ", " << newPt1.y << "]" << std::endl << std::endl;
    */
 
    points.push_back(newPt0);
    points.push_back(newPt1);

    return points;
}

// assumes that the first point in vector lays in the left side, thus the second
// point lays in the right side
bool AllowedDistance( std::vector<cv::Point> l0, 
                      std::vector<cv::Point> l1,
                      float maxDistance )
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

    // TODO delete
    /*
    std::cout << l0 << std::endl;
    std::cout << l1 << std::endl;
    std::cout << leftPointDistance << " " << rightPointDistance << std::endl;
    std::cout << std::endl;
    */

    if (leftPointDistance < maxDistance && rightPointDistance < maxDistance)
        return true;
    else
        return false;
}

std::vector<cv::Point> Merge( std::vector<std::vector<cv::Point> > lines )
{
    std::vector<cv::Point> mergedLine;
    std::vector<cv::Point> leftPoints, 
                       rightPoints;
    float leftX  = 0,  
          leftY  = 0,
          rightX = 0, 
          rightY = 0;
    int numberOfLines = lines.size();

    // TODO delete
    /*
    std::cout << numberOfLines << std::endl;
    std::cout << lines.at(0) << std::endl;
    std::cout << lines.at(1) << std::endl;
    */

    // divide to left and right points
    for (std::vector<cv::Point> l : lines) {
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

bool FindIntInVector( std::vector<int> v, 
                   int i )
{
    if (std::find(v.begin(), v.end(), i) != v.end())
        return true;
    else
        return false;
}

std::vector<cv::Point> ControlMerge( std::vector<std::vector<cv::Point> > & lines,
                                     int index,
                                     std::vector<int> & usedIndices )
{
    std::vector<std::vector<cv::Point> > toMerge;

    // TODO setupable
    float maxDistance = 12;

    bool found = false;
    std::vector<cv::Point> pt0 = lines.at(index);
    std::vector<cv::Point> pt1;

    toMerge.push_back(pt0);

    for (int i = index+1; i < lines.size(); ++i) {
        if (!FindIntInVector(usedIndices, i)) {
            pt1 = lines.at(i);

            if (AllowedDistance(pt0, pt1, maxDistance)) {
                usedIndices.push_back(i);
                toMerge.push_back(pt1);
            }
        }
    }

    // MERGE
    if (toMerge.size() == 1) // line is not supposed to merge
        return pt0;
    else 
        return Merge(toMerge);
}

// method of stored lines changes in this method
// lines are not stored in Vec2f anymore
std::vector<std::vector<cv::Point> > 
MergeSimilarLines( std::vector<std::vector<cv::Point> > & lines )
{
    std::vector<std::vector<cv::Point> > mergedLines;
    std::vector<int> usedIndices;
    std::vector<cv::Point> line;

    for(int i = 0; i < lines.size(); ++i) {
        if (!FindIntInVector(usedIndices, i)) {
            usedIndices.push_back(i);
            line = ControlMerge(lines, i, usedIndices);
            mergedLines.push_back(line);
        }
    }

    return mergedLines;
}

std::vector<cv::Point> Vec4iLinesToPointLines ( const cv::Vec4i & vl,
                                                const cv::Mat & src )
{
    cv::Vec4i v;
    cv::Point leftPoint, rightPoint;
    std::vector<cv::Point> line;

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

std::vector<cv::Point> HighestLine( const std::vector<std::vector<cv::Point> > & lines)
{
    std::vector<cv::Point> highestLine;
    if (lines.size() > 0) {
        highestLine = lines.at(0);
        for (std::vector<cv::Point> l : lines) {
            if (highestLine.at(0).y > l.at(0).y)
                highestLine = l;
        }
    }

    return highestLine;
}

void ThreeLowestLines( const std::vector<std::vector<cv::Point> > & lines,
                       std::vector<cv::Point> & first,
                       std::vector<cv::Point> & second,
                       std::vector<cv::Point> & third )
{
    float tmpY;
    std::vector<cv::Point> highestLine = HighestLine(lines);

    first  = highestLine;
    second = highestLine;
    third  = highestLine;

    for (std::vector<cv::Point> l : lines) {
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

inline bool IsAscending( const std::vector<cv::Point> & line )
{
    if (line.at(0).y < line.at(1).y)
        return true;
    else
        return false;
}

inline float GetHypotenuse( const std::vector<cv::Point> & line )
{
    return std::sqrt( std::pow(std::abs(line.at(0).x-line.at(1).x), 2) + 
                      std::pow(std::abs(line.at(0).y-line.at(1).y), 2) );
}

inline float GetAdjacentCathetus( const std::vector<cv::Point> & line )
{
    return std::abs(line.at(0).x - line.at(1).x);
}

float GetLineAngle( std::vector<cv::Point> &line )
{
    float adjacentCathetus = GetAdjacentCathetus(line);
    float hypotenuse = GetHypotenuse(line);
    float angle = std::acos(adjacentCathetus/hypotenuse) * 180.0 / PI;;
    bool ascending = IsAscending(line);
    
    if (ascending)
        return angle;
    else
        return -angle;
}

inline bool EqualLines ( const std::vector<cv::Point> l0,
                         const std::vector<cv::Point> l1 )
{
    if ( l0.at(0).x == l1.at(0).x && l0.at(0).y == l1.at(0).y && // left points
         l0.at(1).x == l1.at(1).x && l0.at(1).y == l1.at(1).y )  // right points
        return true;
    else
        return false;
}

inline bool AbsoluteComparison( float f0,
                                float f1,
                                float degreeOfFreedom )
{
    if (std::abs(f0-f1) <= degreeOfFreedom)
        return true;
    else
        return false;
}

std::vector<std::vector<cv::Point> > 
FindCurbing( const std::vector<std::vector<cv::Point> > & lines )
{
    std::vector<cv::Point> firstLine, secondLine, thirdLine;
    std::vector<std::vector<cv::Point> > curbing;

    int tmpCurbingSize;
    int tmpLinesSize;
    float angleFirstLine  = NO_ANGLE, 
          angleSecondLine = NO_ANGLE, 
          angleThirdLine  = NO_ANGLE,
          tmpAngle        = NO_ANGLE;

    ThreeLowestLines(lines, firstLine, secondLine, thirdLine);

    // TODO make setupable ///////
    float maxAngleDiff= 10;
    //////////////////////////////

    tmpLinesSize = lines.size();

    // TODO delete
    std::cout << tmpLinesSize << std::endl;

    if (tmpLinesSize > 2) { 
    // potential fully found curb

        // process the first and second line from bottom
        if (!EqualLines(firstLine, secondLine)) {
           angleFirstLine = GetLineAngle(firstLine);
           angleSecondLine = GetLineAngle(secondLine);

           if (AbsoluteComparison(angleFirstLine, angleSecondLine, maxAngleDiff))
           {
               curbing.push_back(firstLine);
               curbing.push_back(secondLine);
           }
        }
        else {
            curbing.push_back(firstLine);
        }

        // process the second and third line from bottom
        int tmpCurbingSize = curbing.size();
        switch (tmpCurbingSize) {
            case 1: 
                if (!EqualLines(thirdLine, curbing.at(0))) {
                    tmpAngle = GetLineAngle(curbing.at(0));

                    if (AbsoluteComparison(angleThirdLine, tmpAngle, maxAngleDiff))
                        curbing.push_back(thirdLine);
                }
                break;

            case 2: 
                if (!EqualLines(thirdLine, secondLine)) {
                    angleThirdLine = GetLineAngle(thirdLine);

                    if (AbsoluteComparison(angleThirdLine, angleSecondLine, maxAngleDiff))
                        curbing.push_back(thirdLine);
                }
                break;
        }
    }
    else if (tmpLinesSize == 2) {
        if (!EqualLines(firstLine, secondLine)) {
            angleFirstLine = GetLineAngle(firstLine);
            angleSecondLine = GetLineAngle(secondLine);

            if (AbsoluteComparison(angleFirstLine, angleSecondLine, maxAngleDiff)){
                curbing.push_back(firstLine);
                curbing.push_back(secondLine);
            }
        }
        else {
                curbing.push_back(firstLine);
        }
    }
    else if (tmpLinesSize == 1) {
        curbing.push_back(firstLine);
    }

    // TODO delete //////////////////////////////////
    std::cout << GetLineAngle(firstLine) << std::endl
              << GetLineAngle(secondLine) << std::endl
              << GetLineAngle(thirdLine) << std::endl;
    //////////////////////////////////////////////////

    return curbing;
}

void HoughTransformProbabilistic( const cv::Mat & src, 
                                  cv::Mat & dst )
{
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Point> > mergedLines;
    std::vector<std::vector<cv::Point> > allLinesFromPoints;
    std::vector<std::vector<cv::Point> > curbing;

    cv::Point pt1,    pt2,
              newPt1, newPt2;

    // find hough lines
    HoughLinesP(src, lines, 1, CV_PI/180, 150, 50, 150);

    // convert HoughP lines to vector of points
    for (int i = 0; i < lines.size(); i++)
        allLinesFromPoints.push_back( Vec4iLinesToPointLines(lines[i], src) );

    // merge lines
    if (allLinesFromPoints.size() > 0) {
        mergedLines = MergeSimilarLines(allLinesFromPoints);
        curbing = FindCurbing(mergedLines);
    }

    /*
    // draw merged lines
    for (int i = 0; i < mergedLines.size(); ++i) {
        line( dst, 
              mergedLines.at(i).at(0), mergedLines.at(i).at(1), 
              cv::Scalar(0,0,255), 1, CV_AA );
    }
    */

    // draw curbing
    for (int i = 0; i < curbing.size(); ++i) {
        line( dst, 
              curbing.at(i).at(0), curbing.at(i).at(1), 
              cv::Scalar(0,0,255), 1, CV_AA );
    }

    std::cout << lines.size() << " / " << mergedLines.size() << std::endl;
}

int main( int argc, char** argv) {
    cv::Mat src, dst;
    cv::Mat conc;
    cv::Mat edges, no_freckles;
    cv::Mat gray;

    std::string path = "../../../curbing/subset0/";
    //std::string path = "../../../curbing2/second_shots/subset2/";
    std::vector<std::string> pathToImages = CreateImageNames(path, 7);
    //std::string type = "hough-";

    int i = 0;
    for (std::string s : pathToImages) {
        //s = path + "5.jpg";
        std::cout << s << std::endl;

        // preprocessing
        src = cv::imread(s);
        src = DownSample(src, 6);
        //src = DownSample(src, 3);
        cv::cvtColor(src, gray, CV_BGR2GRAY);

        cv::equalizeHist(gray, dst);
        int med = 11;
        Blur typeBlur = MEDIAN;
        BlurImage(dst, dst, typeBlur, med);
        BlurImage(dst, dst, typeBlur, med);
        BlurImage(dst, dst, typeBlur, med);

        edges = DetectEdges(src);
        //no_freckles = ClearFreckles(edges);
        HoughTransformProbabilistic(edges, dst);
        
        hconcat(dst, edges, conc);
        imshow("concatenation", conc);
        cv::waitKey();
        //exit(1);
        
        //cv::imwrite(path+type+std::to_string(i)+std::string(".jpg"), conc);
        ++i;
    }

    /*
    cv::VideoCapture stream(1);
    cv::Mat frame;
    char k;

    if (!stream.isOpened()) { 
        std::cout << "Cannot open the camera!" << std::endl;
        return -1;
    }

    stream.read(frame);

    while (true) {
        stream.read(frame);

        DetectEdges(frame, dst);
        imshow("Cam", dst);
        //imshow("RGB", frame);

        k = cv::waitKey(1);
        if (k == 'q')
            break;
    }
    */

  return 0;
}

/**
 * Blurs image. Either Gaussian, Median or Bilateral.
 *
 * @param  src     input image
 * @param  dst     blurred image
 * @param  k_size  size of kernel 
 */
void BlurImage(cv::Mat & src, cv::Mat & dst, Blur type, int k_size) {
  switch (type) {
    case GAUSSIAN: 
      GaussianBlur(src, dst, cv::Size(k_size, k_size), 1, 1); 
      break;

    case MEDIAN: 
      medianBlur(src, dst, k_size);
      break;

    case BILATERAL:
      cv::Mat dst_tmp;
      bilateralFilter(src, dst_tmp, k_size, k_size*2, k_size/2);
      dst = dst_tmp;
      break;
  }
}

/**
 * Detects edges.
 *
 * @param   src
 * @param   dst
 * @param   type  type of detected edge
 */
void EdgeDetect(cv::Mat & src, cv::Mat & dst, Edge type) {
  // canny
  int low_threshold = 20;
  int ratio = 3;
  int k_size = 3;

  // sobel
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  switch (type) {
    case CANNY:
      cv::Canny(src, dst, low_threshold, low_threshold*ratio, k_size);
      break;

    case SOBEL:
      // gradient X
      cv::Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
      cv::convertScaleAbs(grad_x, abs_grad_x);

      // gradient Y
      cv::Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
      cv::convertScaleAbs(grad_y, abs_grad_y);

      /// total gradient (approximate)
      addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
      break;
  }
}

/**
 * Fills the holes in edges.
 */
void MorphologicalClosing(cv::Mat & src, cv::Mat & dst) {
    cv::Mat kernel = (cv::Mat_<int>(3,3) << 0, 1, 0, 
            1, 1, 1, 
            0, 1, 0);
    cv::filter2D(src, dst, -1, kernel, cv::Point(-1,-1));
}

/**
 * Creates mask of single painting given coordinates/contours.
 *
 * @param   size
 * @param   coordinates
 */
cv::Mat CreateMask(cv::Size size, std::vector<cv::Point> & coordinates) {
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC3);
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(coordinates);

    drawContours(mask, contours, 0, cv::Scalar::all(255), CV_FILLED, 8);

    return mask;
}

/**
 * Extracts base name from given path.
 */
std::string basename(std::string const& pathname)
{
    return std::string(std::find_if(pathname.rbegin(), pathname.rend(), MatchPathSeparator()).base(),
            pathname.end());
}
