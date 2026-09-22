#ifndef FORCEWRENCHCONDITIONER_H
#define FORCEWRENCHCONDITIONER_H

#include "forceinteractiontypes.h"

#include <QString>

// 真实F/T进入动力学前的确定性输入调理。原始Trace、预热判稳和软件取零均不经过
// 本模块；它只处理已经去零并转换到动平台质心的力旋量。
struct ForceWrenchConditioningConfig
{
    bool lowPassEnabled = true;
    double lowPassCutoffHz = 10.0;
    double forceStartThresholdN = 0.30;
    double forceReleaseThresholdN = 0.15;
    double torqueStartThresholdNm = 0.010;
    double torqueReleaseThresholdNm = 0.005;

    bool validate(double samplePeriodS,
                  QString* errorMessage = nullptr) const;
};

struct ForceWrenchConditioningResult
{
    ForceInteractionVector6 unfiltered{};
    ForceInteractionVector6 filtered{};
    ForceInteractionVector6 output{};
    double forceNormN = 0.0;
    double torqueNormNm = 0.0;
    bool forceActive = false;
    bool torqueActive = false;
    bool valid = false;
};

class ForceWrenchConditioner
{
public:
    bool configure(const ForceWrenchConditioningConfig& config,
                   double samplePeriodS,
                   QString* errorMessage = nullptr);
    void reset();
    ForceWrenchConditioningResult process(
            const ForceInteractionVector6& platformWrench);

    static bool runSelfChecks(QString* errorMessage = nullptr);

private:
    static double norm3(const ForceInteractionVector6& values, int offset);
    static void updateGate(double norm,
                           double startThreshold,
                           double releaseThreshold,
                           bool& active);

    ForceWrenchConditioningConfig config_;
    ForceInteractionVector6 filtered_{};
    double alpha_ = 1.0;
    bool configured_ = false;
    bool filterInitialized_ = false;
    bool forceActive_ = false;
    bool torqueActive_ = false;
};

#endif // FORCEWRENCHCONDITIONER_H
