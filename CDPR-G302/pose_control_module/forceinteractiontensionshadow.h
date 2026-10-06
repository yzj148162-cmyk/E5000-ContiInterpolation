#ifndef FORCEINTERACTIONTENSIONSHADOW_H
#define FORCEINTERACTIONTENSIONSHADOW_H

#include "compensatedcablekinematics.h"
#include "forceinteractiontypes.h"
#include "redundanttorqueallocator.h"
#include "taskspacetorquecontrol.h"

#include <QString>

// 2026-10-05: M0/M1/M2 shared, calculation-only tension/torque backend.
// It never reads UI/global state and never sends a hardware command.
struct ForceInteractionTensionShadowConfig
{
    ForceInteractionRigidBodyConfig rigidBody;
    CompensatedCableKinematics::Configuration kinematics;
    TaskSpaceTorqueController::Config controller;
    RedundantTorqueAllocator::Config allocator;
    RedundantTorqueAllocator::Vector8d effectiveRadiusMPerRad =
            RedundantTorqueAllocator::Vector8d::Zero();
    bool translationOnly = true;
    double gravityMPerSec2 = 9.8;
    double outerPeriodS = 0.025;

    bool validate(QString* errorMessage = nullptr) const;
};

struct ForceInteractionTensionShadowInput
{
    ForceInteractionPlatformState desired;
    ForceInteractionPlatformState observed;
    double dtS = 0.025;
};

struct ForceInteractionTensionShadowResult
{
    bool valid = false;
    QString errorMessage;
    TaskSpaceTorqueController::Step control;
    RedundantTorqueAllocator::Result allocation;
    EightCablePreviewTiming timing;
};

class ForceInteractionTensionShadow
{
public:
    bool configure(const ForceInteractionTensionShadowConfig& config,
                   QString* errorMessage = nullptr);
    void reset();
    ForceInteractionTensionShadowResult evaluate(
            const ForceInteractionTensionShadowInput& input);

private:
    bool buildJacobian(const TaskSpaceTorqueController::Vector6d& pose,
                       TaskSpaceTorqueController::Matrix8x6d& jacobian,
                       QString* errorMessage) const;

    ForceInteractionTensionShadowConfig config_;
    TaskSpaceTorqueController controller_;
    bool configured_ = false;
};

// Seed used only by shadow calculation.  It reproduces the reference Lite
// controller's proven translational parameters while preserving the current
// G302 geometry, mappings and D1 safety limits supplied by the caller.
ForceInteractionTensionShadowConfig makeDefaultG302TranslationShadowConfig(
        const ForceInteractionRigidBodyConfig& rigidBody,
        const CompensatedCableKinematics::Configuration& kinematics,
        double tensionMinimumN,
        double tensionMaximumN,
        double torqueLimitNm = 34.5);

#endif // FORCEINTERACTIONTENSIONSHADOW_H
