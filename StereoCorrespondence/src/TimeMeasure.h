#include <chrono>
#include <vector>

enum Precision {MILI, MICRO};

class TimeMeasure {
    double AverageLongIntVector(const std::vector<long int> & liv);

    Precision precision;
    std::vector<long int> duration;
    std::chrono::high_resolution_clock::time_point start,
                                                   end;

    public:
        TimeMeasure(Precision p);
        TimeMeasure();
        void Start();
        void Stop();
        double Duration();
};
