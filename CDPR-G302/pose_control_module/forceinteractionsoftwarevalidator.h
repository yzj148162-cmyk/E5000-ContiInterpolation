#ifndef FORCEINTERACTIONSOFTWAREVALIDATOR_H
#define FORCEINTERACTIONSOFTWAREVALIDATOR_H

#include "cdprdynamics.h"
#include "compensatedcablekinematics.h"
#include "wrenchsource.h"

#include <atomic>

#include <QThread>

struct ForceInteractionValidationConfig
{
    QString machineTemplateName;
    ForceInteractionRigidBodyConfig rigidBody;
    NewmarkBetaConfig newmark;
    SimulatedWrenchProfile wrenchProfile;
    ForceSensorTransformConfig sensorTransform;
    bool translationOnly = false;
    double controlPeriodS = 0.005;
    double durationS = 2.0;
    ForceInteractionPlatformState initialState;
    CompensatedCableKinematics::Configuration kinematics;
    CompensatedCableKinematics::PoseMatrix initialPoseMmRad;
    std::vector<double> poseLowerBoundsMmRad;
    std::vector<double> poseUpperBoundsMmRad;
};

struct ForceInteractionValidationResult
{
    bool valid = false;
    bool cancelled = false;
    int completedSteps = 0;
    int forwardKinematicsChecks = 0;
    int maximumNewmarkIterations = 0;
    double maximumNewmarkResidual = 0.0;
    double maximumTranslationRoundTripErrorMm = 0.0;
    double maximumOrientationRoundTripErrorDeg = 0.0;
    double maximumCableResidualMm = 0.0;
    double maximumRelativeMotorAngleRad = 0.0;
    int firstPoseBoundsViolationStep = 0;
    int firstRoundTripToleranceViolationStep = 0;
    int maximumTranslationErrorStep = 0;
    int maximumOrientationErrorStep = 0;
    int maximumCableResidualStep = 0;
    ForceInteractionPlatformState firstPoseBoundsViolationState;
    ForceInteractionPlatformState firstRoundTripToleranceViolationState;
    ForceInteractionPlatformState finalState;
    QString summary;
};

class ForceInteractionSoftwareValidator
{
public:
    static ForceInteractionValidationResult run(
            const ForceInteractionValidationConfig& configuration,
            const std::atomic_bool* cancellationRequested = nullptr);
};

class ForceInteractionValidationWorker final : public QThread
{
    Q_OBJECT

public:
    explicit ForceInteractionValidationWorker(
            const ForceInteractionValidationConfig& configuration,
            QObject* parent = nullptr);
    void requestCancellation();

signals:
    void validationCompleted(bool valid, bool cancelled, const QString& summary);

protected:
    void run() override;

private:
    ForceInteractionValidationConfig configuration_;
    std::atomic_bool cancellationRequested_{false};
};

#endif // FORCEINTERACTIONSOFTWAREVALIDATOR_H
