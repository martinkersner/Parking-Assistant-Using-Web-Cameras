/**
 * Parking Assistant Using Web Cameras
 * Martin Kersner's Master Thesis
 *
 * Measures a time of code execution.
 *
 * m.kersner@gmail.com
 * 03/07/2015
 */

#include "TimeMeasure.h"

TimeMeasure::TimeMeasure(Precision p) : precision(p)
{}

TimeMeasure::TimeMeasure()
{
    this->precision = MILI;
}

void TimeMeasure::Start()
{
    this->start = std::chrono::high_resolution_clock::now();
}

void TimeMeasure::Stop()
{
    this->end = std::chrono::high_resolution_clock::now();

    long int d;
    auto diff = this->end - this->start;

    switch (this->precision) {
        case MICRO:
            d = std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
            break;
        case MILI:
            d = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
            break;
    }

    this->duration.push_back(d);
}

double TimeMeasure::Duration()
{
    return AverageLongIntVector(this->duration);
}

double TimeMeasure::AverageLongIntVector(const std::vector<long int> & liv)
{
    long int totalSum = 0;
    int number = liv.size();

    for (long int d : liv)
        totalSum += d;

    return double(totalSum) / number;
}
