#ifndef FORCEPID0525_H
#define FORCEPID0525_H

#include <cmath>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cassert>

#include <QDebug>

class ForcePid0525
{
public:
    ForcePid0525();
    ForcePid0525(std::vector<double> _kp,
                 std::vector<double> _ki,
                 std::vector<double> _kd,
                 double _Tms);
    ~ForcePid0525();

    std::vector<double> update(std::vector<double> act,
                               std::vector<double> exp,
                               const std::vector<int>& freezeIntegral = std::vector<int>());
    std::vector<double> updateWithRelativeDeadband(std::vector<double> act,
                                                   std::vector<double> exp,
                                                   double deadbandRatio,
                                                   const std::vector<int>& freezeIntegral = std::vector<int>());

    void updatePara(std::vector<double> _kp,
                    std::vector<double> _ki,
                    std::vector<double> _kd,
                    double _Tms);
    void updateTustinPara(std::vector<double> _der_tau,
                          std::vector<double> _integ_min,
                          std::vector<double> _integ_max,
                          std::vector<double> _output_min = std::vector<double>(),
                          std::vector<double> _output_max = std::vector<double>());

    void resetTolErr();
    void resetIntegral(size_t index);
    void resetChannel(size_t index);
    void resetAll();

    const std::vector<double>& debugError() const { return debugErr; }
    const std::vector<double>& debugPTerm() const { return debugP; }
    const std::vector<double>& debugITerm() const { return debugI; }
    const std::vector<double>& debugDTerm() const { return debugD; }
    const std::vector<double>& debugIntegral() const { return tolErr; }
    const std::vector<double>& debugOutput() const { return debugOutputNm; }

private:
    double T;
    std::vector<double> kp, ki, kd;
    std::vector<double> lastErr, tolErr;
    std::vector<double> integrator;
    std::vector<double> lastDer;
    std::vector<double> der_tau;
    std::vector<double> integ_min, integ_max;
    std::vector<double> output_min, output_max;
    std::vector<double> debugErr, debugP, debugI, debugD, debugOutputNm;
};

#endif // FORCEPID0525_H
