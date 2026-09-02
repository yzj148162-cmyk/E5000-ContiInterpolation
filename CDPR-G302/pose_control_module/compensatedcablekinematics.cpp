#include "compensatedcablekinematics.h"

#include "MatrixFun.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kWinchWarmupStepCount = 50;

bool finitePose6(const std::vector<double>& pose)
{
    if(pose.size() < 6){
        return false;
    }
    for(int dim = 0; dim < 6; ++dim){
        if(!std::isfinite(pose[dim])){
            return false;
        }
    }
    return true;
}

std::vector<double> rotateContactPoint(const std::vector<double>& point,
                                       double rx,
                                       double ry,
                                       double rz)
{
    if(point.size() < 3){
        return {};
    }
    const double cx = std::cos(rx);
    const double sx = std::sin(rx);
    const double cy = std::cos(ry);
    const double sy = std::sin(ry);
    const double cz = std::cos(rz);
    const double sz = std::sin(rz);

    const double x1 = point[0];
    const double y1 = cx * point[1] - sx * point[2];
    const double z1 = sx * point[1] + cx * point[2];
    const double x2 = cy * x1 + sy * z1;
    const double y2 = y1;
    const double z2 = -sy * x1 + cy * z1;
    return {
        cz * x2 - sz * y2,
        sz * x2 + cz * y2,
        z2
    };
}

}

bool CompensatedCableKinematics::initialize(
        const Configuration& configuration,
        const PoseMatrix& initialPose,
        const std::vector<double>& initialCableTensionN,
        QString* errorMessage)
{
    initialized = false;
    config = Configuration();
    referenceCableLengthMm.clear();
    initialMotorThetaRad.clear();
    stateAfterInitialReference = State();

    if(!validateConfiguration(configuration, initialPose, errorMessage)){
        return false;
    }
    config = configuration;

    QString geometryError;
    const std::vector<double> initialCableLength =
            cableLengthsForPose(initialPose, &geometryError);
    const PoseMatrix referencePose = normalizedReferencePose();
    std::vector<double> referenceCableLength =
            cableLengthsForPose(referencePose, &geometryError);
    const int count = axisCount();
    if(static_cast<int>(initialCableLength.size()) != count ||
            static_cast<int>(referenceCableLength.size()) != count){
        if(errorMessage){
            *errorMessage = geometryError.isEmpty() ?
                        QStringLiteral("无法计算遥控初始位姿或绞盘参考位姿的八轴绳长") :
                        geometryError;
        }
        config = Configuration();
        return false;
    }

    State warmupState;
    warmupState.previousWinchTakeupMm.assign(count, 0.0);
    std::vector<bool> useCurrentAxialOffsetReference(count, false);
    for(int axis = 0; axis < count; ++axis){
        const WinchCompensation::AxisConfig axisConfig =
                axis < static_cast<int>(config.winchConfig.size()) ?
                    config.winchConfig[axis] : WinchCompensation::AxisConfig();
        if(WinchCompensation::isEnabled(axisConfig) &&
                axisConfig.initialAxialOffsetValid){
            referenceCableLength[axis] = initialCableLength[axis];
            useCurrentAxialOffsetReference[axis] = true;
        }
    }
    referenceCableLengthMm = referenceCableLength;

    for(int step = 0; step < kWinchWarmupStepCount; ++step){
        const double ratio = kWinchWarmupStepCount <= 1 ? 1.0 :
                    static_cast<double>(step) /
                    static_cast<double>(kWinchWarmupStepCount - 1);
        PoseMatrix warmupPose = referencePose;
        for(int end = 0; end < static_cast<int>(initialPose.size()); ++end){
            for(int dim = 0; dim < 6; ++dim){
                warmupPose[end][dim] = referencePose[end][dim] +
                        (initialPose[end][dim] - referencePose[end][dim]) * ratio;
            }
        }
        const std::vector<double> warmupCableLength =
                cableLengthsForPose(warmupPose, &geometryError);
        if(static_cast<int>(warmupCableLength.size()) != count){
            if(errorMessage){
                *errorMessage = geometryError.isEmpty() ?
                            QStringLiteral("绞盘连续解初始化时无法计算八轴绳长") :
                            geometryError;
            }
            config = Configuration();
            referenceCableLengthMm.clear();
            return false;
        }
        for(int axis = 0; axis < count; ++axis){
            if(useCurrentAxialOffsetReference[axis]){
                continue;
            }
            const WinchCompensation::AxisConfig axisConfig =
                    axis < static_cast<int>(config.winchConfig.size()) ?
                        config.winchConfig[axis] : WinchCompensation::AxisConfig();
            const WinchCompensation::SolveResult result =
                    WinchCompensation::solveTakeupFromPlatformDelta(
                        axisConfig,
                        referenceCableLengthMm[axis] - warmupCableLength[axis],
                        warmupState.previousWinchTakeupMm[axis]);
            if(!result.valid || !std::isfinite(result.takeupMm)){
                if(errorMessage){
                    *errorMessage = QStringLiteral("绞盘连续解初始化失败：轴%1没有有效收绳量")
                            .arg(axis + 1);
                }
                config = Configuration();
                referenceCableLengthMm.clear();
                return false;
            }
            warmupState.previousWinchTakeupMm[axis] = result.takeupMm;
        }
    }

    const AbsoluteEvaluation initialEvaluation =
            evaluateAbsoluteMotorTheta(initialCableLength,
                                       warmupState,
                                       initialCableTensionN);
    if(!initialEvaluation.valid){
        if(errorMessage){
            *errorMessage = initialEvaluation.errorMessage;
        }
        config = Configuration();
        referenceCableLengthMm.clear();
        return false;
    }
    initialMotorThetaRad = initialEvaluation.motorThetaRad;
    stateAfterInitialReference = initialEvaluation.nextState;
    initialized = true;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool CompensatedCableKinematics::isInitialized() const
{
    return initialized;
}

int CompensatedCableKinematics::axisCount() const
{
    return static_cast<int>(config.anchorCableCoordinate.size());
}

const CompensatedCableKinematics::State&
CompensatedCableKinematics::initialState() const
{
    return stateAfterInitialReference;
}

CompensatedCableKinematics::Evaluation
CompensatedCableKinematics::evaluatePose(
        const PoseMatrix& pose,
        const State& previousState,
        const std::vector<double>& cableTensionN) const
{
    QString error;
    const std::vector<double> lengths = cableLengthsForPose(pose, &error);
    if(lengths.empty()){
        Evaluation result;
        result.errorMessage = error;
        return result;
    }
    Evaluation result = evaluateCableLengths(lengths, previousState, cableTensionN);
    result.cableLengthMm = lengths;
    return result;
}

CompensatedCableKinematics::Evaluation
CompensatedCableKinematics::evaluateCableLengths(
        const std::vector<double>& cableLengthMm,
        const State& previousState,
        const std::vector<double>& cableTensionN) const
{
    Evaluation result;
    if(!initialized ||
            static_cast<int>(initialMotorThetaRad.size()) != axisCount()){
        result.errorMessage = QStringLiteral("补偿运动学尚未初始化");
        return result;
    }
    const AbsoluteEvaluation absolute =
            evaluateAbsoluteMotorTheta(cableLengthMm,
                                       previousState,
                                       cableTensionN);
    if(!absolute.valid){
        result.errorMessage = absolute.errorMessage;
        return result;
    }
    result.relativeMotorThetaRad.resize(axisCount(), 0.0);
    for(int axis = 0; axis < axisCount(); ++axis){
        result.relativeMotorThetaRad[axis] =
                absolute.motorThetaRad[axis] - initialMotorThetaRad[axis];
        if(!std::isfinite(result.relativeMotorThetaRad[axis])){
            result.errorMessage = QStringLiteral("轴%1补偿后相对电机角度无效")
                    .arg(axis + 1);
            return result;
        }
    }
    result.nextState = absolute.nextState;
    result.cableLengthMm = cableLengthMm;
    result.valid = true;
    return result;
}

std::vector<double> CompensatedCableKinematics::cableLengthsForPose(
        const PoseMatrix& pose,
        QString* errorMessage) const
{
    if(pose.size() != config.endCableContactPos.size()){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端位姿数量与绳索接触点配置不一致");
        }
        return {};
    }
    std::vector<double> lengths;
    lengths.reserve(config.anchorCableCoordinate.size());
    int anchorIndex = 0;
    for(int end = 0; end < static_cast<int>(pose.size()); ++end){
        if(!finitePose6(pose[end])){
            if(errorMessage){
                *errorMessage = QStringLiteral("末端%1位姿无效").arg(end + 1);
            }
            return {};
        }
        for(const std::vector<double>& contact : config.endCableContactPos[end]){
            if(anchorIndex >= static_cast<int>(config.anchorCableCoordinate.size())){
                if(errorMessage){
                    *errorMessage = QStringLiteral("绳索接触点数量超过固定锚点数量");
                }
                return {};
            }
            const std::vector<double> rotated = rotateContactPoint(
                        contact, pose[end][3], pose[end][4], pose[end][5]);
            if(rotated.size() < 3){
                if(errorMessage){
                    *errorMessage = QStringLiteral("末端%1接触点旋转计算失败")
                            .arg(end + 1);
                }
                return {};
            }
            const std::vector<double> globalPoint = {
                pose[end][0] + rotated[0],
                pose[end][1] + rotated[1],
                pose[end][2] + rotated[2]
            };
            const double length = MatrixFun::cableLengthCalculate(
                        globalPoint,
                        config.anchorCableCoordinate[anchorIndex],
                        config.pulleyRadiusMm).idealLength;
            if(!std::isfinite(length) || length < 0.0){
                if(errorMessage){
                    *errorMessage = QStringLiteral("轴%1滑轮绳长计算结果无效")
                            .arg(anchorIndex + 1);
                }
                return {};
            }
            lengths.push_back(length);
            ++anchorIndex;
        }
    }
    if(anchorIndex != static_cast<int>(config.anchorCableCoordinate.size())){
        if(errorMessage){
            *errorMessage = QStringLiteral("绳索接触点数量少于固定锚点数量");
        }
        return {};
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return lengths;
}

bool CompensatedCableKinematics::validateConfiguration(
        const Configuration& configuration,
        const PoseMatrix& initialPose,
        QString* errorMessage) const
{
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(configuration.endCableContactPos.empty() ||
            configuration.anchorCableCoordinate.empty()){
        return fail(QStringLiteral("补偿运动学缺少末端接触点或固定锚点"));
    }
    if(initialPose.size() != configuration.endCableContactPos.size()){
        return fail(QStringLiteral("初始末端位姿数量与接触点配置不一致"));
    }
    int contactCount = 0;
    for(int end = 0; end < static_cast<int>(configuration.endCableContactPos.size()); ++end){
        if(configuration.endCableContactPos[end].empty() ||
                !finitePose6(initialPose[end])){
            return fail(QStringLiteral("末端%1接触点或初始位姿无效").arg(end + 1));
        }
        for(const std::vector<double>& point : configuration.endCableContactPos[end]){
            if(point.size() < 3 ||
                    !std::isfinite(point[0]) ||
                    !std::isfinite(point[1]) ||
                    !std::isfinite(point[2])){
                return fail(QStringLiteral("末端%1包含无效绳索接触点").arg(end + 1));
            }
            ++contactCount;
        }
    }
    if(contactCount != static_cast<int>(configuration.anchorCableCoordinate.size())){
        return fail(QStringLiteral("绳索接触点数量与固定锚点数量不一致"));
    }
    if(configuration.cableMotorScaleRadPerMm.size() !=
            configuration.anchorCableCoordinate.size()){
        return fail(QStringLiteral("绳索电机换算系数数量与固定锚点数量不一致"));
    }
    for(int axis = 0; axis < contactCount; ++axis){
        const std::vector<double>& anchor = configuration.anchorCableCoordinate[axis];
        if(anchor.size() < 3 ||
                !std::isfinite(anchor[0]) ||
                !std::isfinite(anchor[1]) ||
                !std::isfinite(anchor[2]) ||
                !std::isfinite(configuration.cableMotorScaleRadPerMm[axis])){
            return fail(QStringLiteral("轴%1锚点或电机换算系数无效").arg(axis + 1));
        }
    }
    if(!std::isfinite(configuration.pulleyRadiusMm) ||
            configuration.pulleyRadiusMm < 0.0){
        return fail(QStringLiteral("滑轮半径无效"));
    }
    return true;
}

CompensatedCableKinematics::AbsoluteEvaluation
CompensatedCableKinematics::evaluateAbsoluteMotorTheta(
        const std::vector<double>& cableLengthMm,
        const State& previousState,
        const std::vector<double>& cableTensionN) const
{
    AbsoluteEvaluation result;
    const int count = axisCount();
    if(static_cast<int>(cableLengthMm.size()) != count ||
            static_cast<int>(referenceCableLengthMm.size()) != count ||
            static_cast<int>(previousState.previousWinchTakeupMm.size()) != count){
        result.errorMessage = QStringLiteral("绳长、参考绳长或绞盘状态不是完整八轴数据");
        return result;
    }
    result.motorThetaRad.resize(count, 0.0);
    result.nextState.previousWinchTakeupMm.resize(count, 0.0);
    for(int axis = 0; axis < count; ++axis){
        if(!std::isfinite(cableLengthMm[axis])){
            result.errorMessage = QStringLiteral("轴%1绳长无效").arg(axis + 1);
            return result;
        }
        const WinchCompensation::AxisConfig axisConfig =
                axis < static_cast<int>(config.winchConfig.size()) ?
                    config.winchConfig[axis] : WinchCompensation::AxisConfig();
        const double platformDelta =
                referenceCableLengthMm[axis] - cableLengthMm[axis];
        const WinchCompensation::SolveResult winchResult =
                WinchCompensation::solveTakeupFromPlatformDelta(
                    axisConfig,
                    platformDelta,
                    previousState.previousWinchTakeupMm[axis]);
        if(!winchResult.valid || !std::isfinite(winchResult.takeupMm)){
            result.errorMessage = QStringLiteral("轴%1绞盘补偿没有连续有效解")
                    .arg(axis + 1);
            return result;
        }
        result.nextState.previousWinchTakeupMm[axis] = winchResult.takeupMm;
        const double winchDelta = platformDelta - winchResult.takeupMm;
        const double tension = axis < static_cast<int>(cableTensionN.size()) ?
                    cableTensionN[axis] : 0.0;
        const double elasticDelta = RopeElasticCompensation::lengthDeltaMm(
                    config.ropeElasticConfig,
                    axis,
                    cableLengthMm[axis],
                    tension);
        const double compensatedLength =
                cableLengthMm[axis] + winchDelta + elasticDelta;
        const double motorTakeup =
                referenceCableLengthMm[axis] - compensatedLength;
        result.motorThetaRad[axis] = WinchCompensation::motorThetaFromTakeup(
                    axisConfig,
                    motorTakeup,
                    config.cableMotorScaleRadPerMm[axis]);
        if(!std::isfinite(result.motorThetaRad[axis])){
            result.errorMessage = QStringLiteral("轴%1补偿后电机角度无效")
                    .arg(axis + 1);
            return result;
        }
    }
    result.valid = true;
    return result;
}

CompensatedCableKinematics::PoseMatrix
CompensatedCableKinematics::normalizedReferencePose() const
{
    PoseMatrix pose(config.endCableContactPos.size(), std::vector<double>(6, 0.0));
    for(int end = 0; end < static_cast<int>(pose.size()); ++end){
        if(end < static_cast<int>(config.winchReferencePose.size()) &&
                finitePose6(config.winchReferencePose[end])){
            std::copy_n(config.winchReferencePose[end].begin(), 6, pose[end].begin());
        }
    }
    return pose;
}
