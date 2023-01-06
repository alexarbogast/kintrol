#include "kintrol/low_pass_filter.h"
#include <cmath>

namespace kintrol
{
LowPassFilter::LowPassFilter()
    : output_(0), e_pow_(0)
{
}

LowPassFilter::LowPassFilter(double cutoff, double dt)
    : output_(0)
{
    reconfigureFilter(cutoff, dt);
}

double LowPassFilter::update(double input)
{
    return output_ += (input - output_) * e_pow_;
}

double LowPassFilter::reconfigureFilter(double cutoff, double dt)
{
    e_pow_ = 1 - exp(-dt * 2 * M_PI * cutoff);
}


} // namespace kintrol
