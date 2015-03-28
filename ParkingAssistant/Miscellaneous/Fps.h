#ifndef MISCELLANEOUS_FPS_H
#define MISCELLANEOUS_FPS_H

#include <opencv2/opencv.hpp>

class Fps {
    double fpsStart = 0;
    double avgFps   = 0;
    double fps1sec  = 0;

    double start    = 0;
    double dur      = 0;

    double Clock();
    double AvgFps();

    public:
        void Before();
        void After();
        double GetFps();
};

#endif // MISCELLANEOUS_FPS_H
