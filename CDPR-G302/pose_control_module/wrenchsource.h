#ifndef WRENCHSOURCE_H
#define WRENCHSOURCE_H

#include "forceinteractiontypes.h"

#include <functional>

class IWrenchSource
{
public:
    virtual ~IWrenchSource() = default;
    virtual ForceInteractionWrenchSample sample(
            const ForceInteractionFrameStamp& stamp,
            double elapsedS) const = 0;
    virtual QString summary() const = 0;
};

class SimulatedWrenchSource final : public IWrenchSource
{
public:
    bool configure(const SimulatedWrenchProfile& profile,
                   double samplePeriodS,
                   QString* errorMessage = nullptr);
    SimulatedWrenchProfile profile() const;
    bool evaluate(double elapsedS,
                  ForceInteractionVector6& wrench,
                  QString* errorMessage = nullptr) const;
    ForceInteractionWrenchSample sample(
            const ForceInteractionFrameStamp& stamp,
            double elapsedS) const override;
    QString summary() const override;

private:
    SimulatedWrenchProfile profile_;
    std::array<std::function<double(double)>, kForceInteractionDofCount>
            evaluators_{};
    double samplePeriodS_ = 0.0;
    bool configured_ = false;
};

#endif // WRENCHSOURCE_H
