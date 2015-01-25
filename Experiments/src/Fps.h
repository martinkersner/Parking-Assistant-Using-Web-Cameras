/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Computing FPS.
 *
 * m.kersner@gmail.com
 * 01/25/2015
 *
 * Inspired by http://stackoverflow.com/users/1662574/zaw-lin
 */

#include <opencv/cv.h>

class FPS {

    double avgdur   = 0;
    double fpsstart = 0;
    double avgfps   = 0;
    double fps1sec  = 0;

    double start    = 0;
    double dur      = 0;

    double clock();
    double avgFps();

    public:
        void before();
        void after();
        double getFps();
};
