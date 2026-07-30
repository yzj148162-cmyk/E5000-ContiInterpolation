#ifndef CDPRKINEMATICS_H
#define CDPRKINEMATICS_H

#include <array>

#include <QString>

#include "CdprConfiguration.h"
#include "CdprControlTypes.h"

struct CdprInverseKinematicsResult
{
    bool valid = false;
    QString errorText;
    CdprCableState8 cables;
    CdprMatrix8x6 lengthJacobian {};
    std::array<CdprVector3, kCdprCableCount> platformAnchorWorldM {};
    std::array<CdprVector3, kCdprCableCount> cableUnitVector {};
};

struct CdprForwardKinematicsOptions
{
    int maximumIterations = 40;
    double residualToleranceM = 1.0e-8;
    double translationDifferenceStepM = 1.0e-6;
    double angleDifferenceStepRad = 1.0e-6;
    double initialDamping = 1.0e-10;
};

struct CdprForwardKinematicsResult
{
    bool valid = false;
    bool converged = false;
    QString errorText;
    CdprVector6 pose {};
    int iterations = 0;
    double rmsResidualM = 0.0;
    double maximumResidualM = 0.0;
};

// 第一版只采用框架出绳点到平台连接点的直线距离。
// 本类复制一份只读几何参数，不保存当前机器人运行状态，也不使用静态共享变量。
class CdprKinematics
{
public:
    explicit CdprKinematics(const CdprConfiguration &configuration);

    bool geometryValid(QString *errorText = nullptr) const;
    CdprInverseKinematicsResult inverse(
        const CdprPlatformState6 &platform) const;
    CdprForwardKinematicsResult forward(
        const CdprVector8 &measuredLengthsM,
        const CdprVector6 &initialGuess,
        const CdprForwardKinematicsOptions &options = {}) const;

private:
    bool evaluateLengths(const CdprVector6 &pose, CdprVector8 &lengths,
                         QString *errorText = nullptr) const;

    std::array<CdprVector3, kCdprCableCount> frameAnchorsM_ {};
    std::array<CdprVector3, kCdprCableCount> platformAnchorsM_ {};
};

#endif // CDPRKINEMATICS_H
