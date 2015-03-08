#ifndef DETECTPAINTINGEDGE_H
#define DETECTPAINTINGEDGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "opencv/cv.h"

#include <algorithm>
#include <cstdlib> 
#include <ctime> 
#include <queue>  
#include <cmath>
#include <stdlib.h>
#include <string>

#define PI        3.14159265
#define NO_ANGLE  999

//enum State {NEW, VALID, INVALID};
enum Blur {GAUSSIAN, MEDIAN, BILATERAL};
enum Edge {CANNY, SOBEL};

// TODO decide whether use coordinates or mask only
struct Painting {
  cv::Point center;
  std::vector<cv::Point> coordinates;
  std::vector<cv::Point> approx;
  cv::Mat mask;
};

typedef std::vector<Painting>  vec_paint;

struct WindowCell {
  vec_paint * paintings;
  cv::Mat source;
};

struct Frame {
  cv::Mat frame;
  cv::Mat mask;
  vec_paint * paintings;
};

struct ClusterParticle {
  cv::Scalar std_dev;
  cv::Mat img;
};

typedef std::vector<std::vector<cv::Point>> vec_cont;

vec_paint * ProcessImage(cv::Mat & src, cv::Mat & dst, cv::Mat * main_source);
void ShowPainting(cv::Mat * src, cv::Mat * mask, cv::Rect rect, int id);
vec_paint * DetectPaintings(cv::Mat * src, cv::Mat * dst);
void BlurImage(cv::Mat & src, cv::Mat & dst, Blur type, int k_size);
void EdgeDetect(cv::Mat & src, cv::Mat & dst, Edge type);
void MorphologicalClosing(cv::Mat & src, cv::Mat & dst);
void DisplayImage (cv::Mat & src);
cv::Point FindCenter(std::vector<cv::Point> contour);
float MeasureDistance(cv::Point p1, cv::Point p2);
void InitializeWindow(cv::VideoCapture & video, cv::Mat & src, cv::Mat & dst, std::queue<WindowCell> & window);
void ProcessWindow(std::queue<WindowCell> & window);
bool ReparableWindow(std::queue<WindowCell> & window);
WindowCell GetFrame(std::queue<WindowCell> & window, int k);
cv::Point ComputeCenterPoint(cv::Point p1, cv::Point p2);
bool CompareEdgeFrames(std::queue<WindowCell> & window, vec_paint * frame_info);
void UpdateFrameInfo(vec_cont contours, int i, std::vector<cv::Point> approx, cv::Size src_size, vec_paint * frame_info);
void CopyFrameInfo(Painting * src, vec_paint * dst);
void InsertPaintings(std::queue<WindowCell> & window, vec_paint & frame_info);
void SumMasks(vec_paint * frame_info, cv::Mat & result_mask);
void CleanFrame(WindowCell * window_cell);
std::vector<cv::Point> ComputeCoordinates(std::vector<cv::Point> c1, std::vector<cv::Point> c2);
WindowCell CreateWindowCell(cv::Mat & src, vec_paint * vp);
bool IsPainting(vec_cont & contours, int i, std::vector<cv::Point> & approx, int image_area);
void SaveMergedFrames(cv::Mat & img1, cv::Mat & img2);
void GetFramedObjects(cv::VideoCapture & video, std::vector<cv::Mat> & out_imgs);
Frame ProcessVideo(cv::VideoCapture & video, std::queue<WindowCell> & window);
cv::Mat CreateMask(cv::Size size, std::vector<cv::Point> & coordinates);
void InsertFramedObjects(Frame & processed_frame, std::vector<cv::Mat> & out_imgs);
void SavePaintings(std::vector<ClusterParticle> & out_imgs, std::string dir);
void InitializeVideo(cv::VideoCapture & video, cv::Mat & src, cv::Mat & dst, std::queue<WindowCell> & window);
bool DisplayPaintings(cv::Mat & src, cv::Mat & mask);
std::vector<std::vector<ClusterParticle>> ClusterPaintings(std::vector<cv::Mat> & images);
bool CompareStdDev(cv::Scalar std_dev1, cv::Scalar std_dev2, float max_diff);
void SaveClusterPaintings(std::vector<std::vector<ClusterParticle>> all_clusters);
void FillClusterParticle(ClusterParticle & cl_particle, cv::Scalar & std_dev, cv::Mat & img);
void CreateCluster(std::vector<std::vector<ClusterParticle>> & all_clusters, ClusterParticle cl_particle);

// extra functionality for searching base name
// instead of using boost
std::string basename(std::string const& pathname);
struct MatchPathSeparator
{
  bool operator()(char ch) const
  {
    return ch == '/';
  }
};

#endif //DETECTPAINTINGEDGE_H
