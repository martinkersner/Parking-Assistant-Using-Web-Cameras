#ifndef MISCELLANEOUS_TIMEMEASURE_H_
#define MISCELLANEOUS_TIMEMEASURE_H_

#include <chrono>
#include <vector>

enum Precision {MILI, MICRO};

class TimeMeasure {
    Precision precision;
    std::vector<long int> duration;
    std::chrono::high_resolution_clock::time_point start,
                                                   end;

    double AverageLongIntVector(const std::vector<long int> & liv);

    public:
        TimeMeasure(Precision p);
        TimeMeasure();
        void Start();
        void Stop();
        double Duration();
};

#endif // MISCELLANEOUS_TIMEMEASURE_H_
