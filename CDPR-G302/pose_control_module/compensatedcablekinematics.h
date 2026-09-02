#ifndef COMPENSATEDCABLEKINEMATICS_H
#define COMPENSATEDCABLEKINEMATICS_H

#include "ropeelasticcompensation.h"
#include "winchcompensation.h"

#include <QString>

#include <vector>

// 统一封装“末端位姿 -> 几何绳长 -> 绞盘/弹性补偿 -> 电机角度”。
// State 显式保存多解绞盘模型的连续解，调用方可以先计算候选状态，只有在
// 控制命令真正提交后再保存该状态。
class CompensatedCableKinematics
{
public:
    using PoseMatrix = std::vector<std::vector<double>>;

    struct Configuration {
        std::vector<double> cableMotorScaleRadPerMm;
        std::vector<WinchCompensation::AxisConfig> winchConfig;
        PoseMatrix winchReferencePose;
        std::vector<std::vector<std::vector<double>>> endCableContactPos;
        std::vector<std::vector<double>> anchorCableCoordinate;
        double pulleyRadiusMm = 0.0;
        RopeElasticCompensation::Config ropeElasticConfig;
    };

    struct State {
        std::vector<double> previousWinchTakeupMm;
    };

    struct Evaluation {
        bool valid = false;
        QString errorMessage;
        std::vector<double> cableLengthMm;
        std::vector<double> relativeMotorThetaRad;
        State nextState;
    };

    bool initialize(const Configuration& configuration,
                    const PoseMatrix& initialPose,
                    const std::vector<double>& initialCableTensionN = {},
                    QString* errorMessage = nullptr);

    bool isInitialized() const;
    int axisCount() const;
    const State& initialState() const;

    Evaluation evaluatePose(const PoseMatrix& pose,
                            const State& previousState,
                            const std::vector<double>& cableTensionN = {}) const;
    Evaluation evaluateCableLengths(const std::vector<double>& cableLengthMm,
                                    const State& previousState,
                                    const std::vector<double>& cableTensionN = {}) const;
    std::vector<double> cableLengthsForPose(const PoseMatrix& pose,
                                            QString* errorMessage = nullptr) const;

private:
    struct AbsoluteEvaluation {
        bool valid = false;
        QString errorMessage;
        std::vector<double> motorThetaRad;
        State nextState;
    };

    bool validateConfiguration(const Configuration& configuration,
                               const PoseMatrix& initialPose,
                               QString* errorMessage) const;
    AbsoluteEvaluation evaluateAbsoluteMotorTheta(
            const std::vector<double>& cableLengthMm,
            const State& previousState,
            const std::vector<double>& cableTensionN) const;
    PoseMatrix normalizedReferencePose() const;

    Configuration config;
    std::vector<double> referenceCableLengthMm;
    std::vector<double> initialMotorThetaRad;
    State stateAfterInitialReference;
    bool initialized = false;
};

#endif // COMPENSATEDCABLEKINEMATICS_H
