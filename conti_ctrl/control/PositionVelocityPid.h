#ifndef POSITIONVELOCITYPID_H
#define POSITIONVELOCITYPID_H

#include "common/ContiTypes.h"

struct PositionVelocityPidOutput
{
    double feedforward = 0.0;
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double rawVelocity = 0.0;
    double commandVelocity = 0.0;
    bool velocitySaturated = false;
    bool accelerationLimited = false;
    bool integralFrozen = false;
};

class PositionVelocityPid
{
public:
    void reset(double initialCommandVelocity = 0.0);
    PositionVelocityPidOutput update(const VelocityControlConfig &config,
                                     double positionErrorDegree,
                                     double referenceVelocityDegreePerSecond,
                                     double actualVelocityDegreePerSecond,
                                     double dtSeconds);

private:
    double integralState_ = 0.0;
    double previousCommandVelocity_ = 0.0;
};

#endif // POSITIONVELOCITYPID_H
