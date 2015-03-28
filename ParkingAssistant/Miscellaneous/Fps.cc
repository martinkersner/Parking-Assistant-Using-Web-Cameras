/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Designed for measuring frames per second.
 * Inspired by http://stackoverflow.com/users/1662574/zaw-lin
 *
 * Snippet usage ***************************************************************
 * Fps fps;
 * cv::Mat frame;
 * cv::VideoCapture cap(0);
 *
 * while (true)
 * {
 *     cap >> frame;
 *     fps.Before();
 *     process(frame);
 *     fps.After();
 *     std::cout << "Fps: " << fps.GetFps() << std::endl;
 * }
 *******************************************************************************
 *
 * m.kersner@gmail.com
 * 01/25/2015
 */

#include "Fps.h"

/**
 * Measuring before frame processing.
 */
void Fps::Before()
{
    this->start = Clock();
}

/**
 * Measuring after frame processing.
 */
void Fps::After()
{
    this->dur = Clock() - this->start;
}

double Fps::Clock()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,  &t);
    return (t.tv_sec * 1000) + (t.tv_nsec*1e-6);
}

double Fps::AvgFps()
{
    if (Clock()-this->fpsStart > 1000) {
        this->fpsStart = Clock();
        this->avgFps = 0.7*this->avgFps + 0.3*this->fps1sec;
        this->fps1sec = 0;
    }

    this->fps1sec++;

    return this->avgFps;
}

double Fps::GetFps()
{
    return AvgFps();
}
