#pragma once

namespace kintrol
{
class LowPassFilter
{
public:
    LowPassFilter();
    LowPassFilter(double cutoff, double dt);

    double update(double input);
    void reconfigureFilter(double cutoff, double dt);
private:
    double output_;
    double e_pow_;
};

} // kintrol