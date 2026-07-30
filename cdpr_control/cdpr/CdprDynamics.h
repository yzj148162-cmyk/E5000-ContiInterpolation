#ifndef CDPRDYNAMICS_H
#define CDPRDYNAMICS_H

#include "CdprConfiguration.h"
#include "CdprControlTypes.h"

#include <QString>

struct CdprNewmarkConfig
{
    double beta = 0.25;
    double gamma = 0.5;
    double convergenceTolerance = 1.0e-9;
    int maximumIterations = 20;
};

struct CdprDynamicsResult
{
    bool valid = false;
    bool converged = false;
    int iterations = 0;
    double residual = 0.0;
    CdprPlatformState6 state;
    QString errorText;
};

// 纯惯性自由刚体Newmark-β单步模块。
// 输入力旋量必须位于平台质心、在平台body frame表达；
// 对外状态的角速度和角加速度在world frame表达。
class CdprDynamics
{
public:
    bool configure(const CdprRigidBodyConfig &rigidBody,
                   const CdprNewmarkConfig &newmark,
                   QString *errorText = nullptr);
    bool reset(const CdprPlatformState6 &initialState,
               QString *errorText = nullptr);
    CdprDynamicsResult step(const CdprWrenchSample &platformWrench,
                            double timeStepSecond);

    bool configured() const;
    bool initialized() const;
    CdprPlatformState6 currentState() const;
    CdprNewmarkConfig newmarkConfig() const;

private:
    bool acceleration(const CdprVector6 &pose,
                      const CdprVector6 &generalizedVelocity,
                      const CdprVector6 &bodyWrench,
                      CdprVector6 &generalizedAcceleration,
                      QString *errorText) const;
    CdprPlatformState6 externalState(
        const CdprVector6 &pose,
        const CdprVector6 &generalizedVelocity,
        const CdprVector6 &generalizedAcceleration) const;

    CdprRigidBodyConfig rigidBody_;
    CdprNewmarkConfig newmark_;
    CdprVector6 pose_ {};
    CdprVector6 generalizedVelocity_ {};
    CdprVector6 generalizedAcceleration_ {};
    bool configured_ = false;
    bool initialized_ = false;
};

#endif // CDPRDYNAMICS_H
