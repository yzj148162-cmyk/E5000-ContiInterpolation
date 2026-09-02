#ifndef CDPRDYNAMICS_H
#define CDPRDYNAMICS_H

#include "forceinteractiontypes.h"

struct NewmarkBetaConfig
{
    double beta = 0.25;
    double gamma = 0.5;
    double convergenceTolerance = 1.0e-9;
    int maximumIterations = 20;
};

struct CdprDynamicsStepResult
{
    bool valid = false;
    bool converged = false;
    int iterations = 0;
    double residual = 0.0;
    ForceInteractionPlatformState state;
    QString errorMessage;
};

// 纯惯性自由刚体 Newmark-β 单步模块，不含回中刚度或人为阻尼。
// 输入力旋量位于平台质心、在平台 body frame 表达。
class CdprDynamics
{
public:
    bool configure(const ForceInteractionRigidBodyConfig& rigidBody,
                   const NewmarkBetaConfig& newmark,
                   QString* errorMessage = nullptr);
    bool reset(const ForceInteractionPlatformState& initialState,
               QString* errorMessage = nullptr);
    CdprDynamicsStepResult step(
            const ForceInteractionWrenchSample& platformWrench,
            double timeStepSecond);

    bool configured() const;
    bool initialized() const;
    ForceInteractionPlatformState currentState() const;

private:
    bool acceleration(const ForceInteractionVector6& pose,
                      const ForceInteractionVector6& generalizedVelocity,
                      const ForceInteractionVector6& bodyWrench,
                      ForceInteractionVector6& generalizedAcceleration,
                      QString* errorMessage) const;
    ForceInteractionPlatformState externalState(
            const ForceInteractionVector6& pose,
            const ForceInteractionVector6& generalizedVelocity,
            const ForceInteractionVector6& generalizedAcceleration) const;

    ForceInteractionRigidBodyConfig rigidBody_;
    NewmarkBetaConfig newmark_;
    ForceInteractionVector6 pose_{};
    ForceInteractionVector6 generalizedVelocity_{};
    ForceInteractionVector6 generalizedAcceleration_{};
    bool configured_ = false;
    bool initialized_ = false;
};

#endif // CDPRDYNAMICS_H
