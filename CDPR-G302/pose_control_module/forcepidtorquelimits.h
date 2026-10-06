#ifndef FORCEPIDTORQUELIMITS_H
#define FORCEPIDTORQUELIMITS_H
#include <algorithm>

// Shared with the original 0525 worker. Callers validate finite bounds/dt.
inline double forcePidTorqueClamp(double value, double lower, double upper)
{
    return std::min(std::max(value, lower), upper);
}
inline double forcePidTorqueSlew(double value, double previous, double rate, double dt)
{
    return forcePidTorqueClamp(value, previous-rate*dt, previous+rate*dt);
}
#endif

