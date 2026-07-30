#include "CdprCoordinator.h"

#include <algorithm>
#include <cmath>

namespace {
QString vectorText(const CdprVector3 &value)
{
    return QStringLiteral("%1, %2, %3")
        .arg(value.x, 0, 'f', 4)
        .arg(value.y, 0, 'f', 4)
        .arg(value.z, 0, 'f', 4);
}
}

CdprCoordinator::CdprCoordinator(QObject *parent)
    : QObject(parent)
{
}

void CdprCoordinator::loadConfiguration(const QString &path)
{
    CdprConfiguration candidate;
    QStringList messages;
    if (!CdprConfigurationFile::load(path, candidate, messages)) {
        configurationLoaded_ = false;
        configurationPath_ = path;
        validationMessages_ = messages;
        rebuildInitialKinematics();
        emit logMessage(QStringLiteral("CDPR配置加载失败：%1")
                            .arg(messages.join(QStringLiteral("；"))));
        publishStatus();
        return;
    }

    configuration_ = candidate;
    configurationPath_ = path;
    configurationLoaded_ = true;
    validationMessages_ = messages;
    rebuildInitialKinematics();
    const int problemCount = validationMessages_.size()
        + (kinematicsError_.isEmpty() ? 0 : 1);
    emit logMessage(problemCount == 0
        ? QStringLiteral("CDPR配置已加载并通过校验：%1").arg(path)
        : QStringLiteral("CDPR配置已加载，但有%1项校验问题。")
              .arg(problemCount));
    publishStatus();
}

void CdprCoordinator::validateConfiguration()
{
    if (!configurationLoaded_) {
        validationMessages_ = {QStringLiteral("请先加载CDPR配置文件。")};
    } else {
        validationMessages_ = CdprConfigurationFile::validate(configuration_);
        rebuildInitialKinematics();
    }
    publishStatus();
}

void CdprCoordinator::writeConfigurationTemplate(const QString &path)
{
    QString error;
    if (!CdprConfigurationFile::writeTemplate(path, error)) {
        emit logMessage(QStringLiteral("CDPR配置模板生成失败：%1").arg(error));
        return;
    }
    emit logMessage(QStringLiteral(
        "CDPR配置模板已生成：%1；核对参数后将parameters_confirmed改为true。")
                        .arg(path));
}

void CdprCoordinator::updateHardwareStatus(const ContiStatus &status)
{
    const int boundedAxisCount = qBound(0, status.detectedAxisCount, 8);
    if (boardInitialized_ == status.boardInitialized
        && detectedAxisCount_ == boundedAxisCount
        && enabledAxisMask_ == status.enabledAxisMask) {
        return;
    }
    boardInitialized_ = status.boardInitialized;
    detectedAxisCount_ = boundedAxisCount;
    enabledAxisMask_ = status.enabledAxisMask;
    publishStatus();
}

void CdprCoordinator::rebuildInitialKinematics()
{
    kinematics_.reset();
    kinematicsReady_ = false;
    kinematicsSummary_.clear();
    kinematicsError_.clear();
    robotState_ = {};
    if (!configurationLoaded_) {
        return;
    }

    kinematics_ = std::make_unique<CdprKinematics>(configuration_);
    QString geometryError;
    if (!kinematics_->geometryValid(&geometryError)) {
        kinematicsError_ =
            QStringLiteral("直线绳段运动学无效：%1").arg(geometryError);
        return;
    }

    CdprPlatformState6 initialPlatform;
    initialPlatform.pose = configuration_.initialPlatformPose;
    initialPlatform.poseValid = true;
    initialPlatform.twistValid = true;
    const CdprInverseKinematicsResult inverse =
        kinematics_->inverse(initialPlatform);
    if (!inverse.valid) {
        kinematicsError_ = QStringLiteral("初始位姿逆运动学失败：%1")
                               .arg(inverse.errorText);
        return;
    }

    // 用轻微偏移的初值回算同一组绳长，检查正逆运动学是否闭合。
    CdprVector6 forwardGuess = configuration_.initialPlatformPose;
    forwardGuess[0] += 0.005;
    forwardGuess[1] -= 0.004;
    forwardGuess[2] += 0.003;
    forwardGuess[3] += 0.005;
    forwardGuess[4] -= 0.004;
    forwardGuess[5] += 0.003;
    const CdprForwardKinematicsResult forward =
        kinematics_->forward(inverse.cables.lengthM, forwardGuess);
    if (!forward.valid || !forward.converged) {
        kinematicsError_ =
            QStringLiteral("直线绳段正运动学闭环自检失败：%1")
                .arg(forward.errorText);
        return;
    }

    double maximumPoseDifference = 0.0;
    for (int index = 0; index < kCdprDofCount; ++index) {
        maximumPoseDifference = std::max(
            maximumPoseDifference,
            std::abs(forward.pose[static_cast<size_t>(index)]
                     - configuration_.initialPlatformPose[
                         static_cast<size_t>(index)]));
    }
    if (maximumPoseDifference > 1.0e-5) {
        kinematicsError_ = QStringLiteral(
            "直线绳段正运动学闭环误差过大：最大位姿分量误差=%1。")
            .arg(maximumPoseDifference, 0, 'g', 6);
        return;
    }

    // 这是配置阶段的静态初值，并非来自真实控制周期，因此不伪造有效时间戳。
    robotState_.stamp = {0, 0, false};
    robotState_.runState = CdprRunState::Configured;
    robotState_.desiredPlatform = initialPlatform;
    robotState_.desiredCables = inverse.cables;
    // 尚无8轴同帧反馈，实际平台和实际绳索必须保持无效。
    kinematicsReady_ = true;

    const auto minimumMaximum = std::minmax_element(
        inverse.cables.lengthM.begin(), inverse.cables.lengthM.end());
    kinematicsSummary_ = QStringLiteral(
        "直线绳段运动学：已通过正逆解闭环自检\n"
        "初始绳长范围：%1～%2 m\n"
        "正运动学：%3次迭代，最大绳长残差=%4 m")
        .arg(*minimumMaximum.first, 0, 'f', 6)
        .arg(*minimumMaximum.second, 0, 'f', 6)
        .arg(forward.iterations)
        .arg(forward.maximumResidualM, 0, 'g', 6);
}

void CdprCoordinator::publishStatus()
{
    CdprUiStatus status;
    status.configurationLoaded = configurationLoaded_;
    status.configurationValid =
        configurationLoaded_ && validationMessages_.isEmpty();
    status.configurationPath = configurationPath_;
    status.configurationId = configurationLoaded_
        ? CdprConfigurationFile::identifier(configuration_) : QString();
    status.summary = configurationLoaded_
        ? CdprConfigurationFile::summary(configuration_) : QString();
    if (!kinematicsSummary_.isEmpty()) {
        status.summary += QStringLiteral("\n\n") + kinematicsSummary_;
    }
    status.validationMessages = validationMessages_;
    if (!kinematicsError_.isEmpty()) {
        status.validationMessages.append(kinematicsError_);
    }
    status.onlineAxisCount = detectedAxisCount_;
    status.boardInitialized = boardInitialized_;
    status.kinematicsReady = kinematicsReady_;
    status.controlStartAvailable = false;

    if (!configurationLoaded_) {
        status.stateText = QStringLiteral("未加载配置");
    } else if (!status.configurationValid) {
        status.stateText = QStringLiteral("配置校验未通过");
    } else if (!kinematicsReady_) {
        status.stateText = QStringLiteral("运动学自检未通过");
    } else if (!boardInitialized_) {
        status.stateText =
            QStringLiteral("配置与运动学有效，等待控制卡初始化");
    } else if (detectedAxisCount_ < 8) {
        status.stateText =
            QStringLiteral("配置有效，在线轴不足8个（%1/8）")
                .arg(detectedAxisCount_);
    } else {
        status.stateText =
            QStringLiteral("数学骨架就绪，8轴运动控制尚未接入");
    }

    if (configurationLoaded_) {
        status.axes.reserve(8);
        for (const CdprCableAxisConfig &cable : configuration_.cables) {
            CdprAxisView axis;
            axis.cable = cable.cable;
            axis.axis = cable.axis;
            axis.direction = cable.direction;
            axis.frameAnchor = vectorText(cable.frameAnchorM);
            axis.platformAnchor = vectorText(cable.platformAnchorM);
            axis.online = boardInitialized_
                && cable.axis >= 0 && cable.axis < detectedAxisCount_;
            axis.enabled = axis.online
                && (enabledAxisMask_
                    & static_cast<quint16>(1U << cable.axis)) != 0U;
            status.axes.append(axis);
        }
    }
    emit statusChanged(status);
}
