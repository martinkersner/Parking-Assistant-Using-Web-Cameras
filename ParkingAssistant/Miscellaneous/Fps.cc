/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Computing FPS.
 *
 * Snippet usage ***************************************************************
 * FPS fps;
 * cv::Mat frame;
 * cv::VideoCapture cap(0);
 *
 * while (true)
 * {
 *     cap >> frame;
 *     fps.before();
 *     process(frame);
 *     fps.after();
 *     std::cout << "fps: " << fps.getFps() << std::endl;
 * }
 ** ****************************************************************************
 *
 * m.kersner@gmail.com
 * 01/25/2015
 *
 * Inspired by http://stackoverflow.com/users/1662574/zaw-lin
 */

#include "Fps.h"

/**
 * Measuring before frame processing.
 */
void FPS::before()
{
    this->start = clock();
}

/**
 * Measuring after frame processing.
 */
void FPS::after()
{
    this->dur = clock()-this->start;
}

double FPS::clock()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,  &t);
    return (t.tv_sec * 1000)+(t.tv_nsec*1e-6);
}

double FPS::avgFps()
{
    if(clock()-this->fpsstart > 1000) {
        this->fpsstart=clock();
        this->avgfps=0.7*this->avgfps+0.3*this->fps1sec;
        this->fps1sec=0;
    }
    this->fps1sec++;
    return this->avgfps;
}

double FPS::getFps()
{
    return avgFps();
}
