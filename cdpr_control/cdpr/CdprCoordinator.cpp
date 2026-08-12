#include "CdprCoordinator.h"
#include "CdprTrajectoryFile.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <QTimer>
#include <QCoreApplication>
#include <QDir>

namespace {
constexpr double kPi = 3.14159265358979323846;

QString vectorText(const CdprVector3 &value)
{
    return QStringLiteral("%1, %2, %3")
        .arg(value.x, 0, 'f', 4)
        .arg(value.y, 0, 'f', 4)
        .arg(value.z, 0, 'f', 4);
}

qint64 monotonicNowUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

QString vector6Text(const CdprVector6 &value, int precision)
{
    QStringList parts;
    parts.reserve(6);
    for (double component : value) {
        parts.append(QString::number(component, 'f', precision));
    }
    return parts.join(QStringLiteral(", "));
}
}

CdprCoordinator::CdprCoordinator(QObject *parent)
    : QObject(parent)
    , statusTimer_(new QTimer(this))
{
    statusTimer_->setInterval(200);
    connect(statusTimer_, &QTimer::timeout,
            this, &CdprCoordinator::publishStatus);
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
        && ethercatOperational_ == status.ethercatOperational
        && detectedAxisCount_ == boundedAxisCount
        && enabledAxisMask_ == status.enabledAxisMask) {
        return;
    }
    boardInitialized_ = status.boardInitialized;
    ethercatOperational_ = status.ethercatOperational;
    detectedAxisCount_ = boundedAxisCount;
    enabledAxisMask_ = status.enabledAxisMask;
    publishStatus();
}

void CdprCoordinator::setInitialPoseSource(int source)
{
    initialPoseSource_ = source
            == static_cast<int>(CdprInitialPoseSource::NokovMarkers)
        ? CdprInitialPoseSource::NokovMarkers
        : CdprInitialPoseSource::Preset;
    startupState_ = {};
    startupState_.poseSource = initialPoseSource_;
    publishStatus();
}

void CdprCoordinator::setPresetInitialPose(
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    presetInitialPose_ = {x, y, z, roll, pitch, yaw};
    if (initialPoseSource_ == CdprInitialPoseSource::Preset) {
        startupState_ = {};
        startupState_.poseSource = initialPoseSource_;
    }
    publishStatus();
}

void CdprCoordinator::connectNokov(const QString &serverAddress)
{
    QString error;
    if (!nokovProvider_.connectToServer(serverAddress, &error)) {
        emit logMessage(error);
    } else {
        emit logMessage(QStringLiteral(
            "Nokov已连接：%1；仅采集LabeledMarkers，"
            "明确忽略SDK刚体姿态。")
                            .arg(serverAddress.trimmed()));
        if (!statusTimer_->isActive()) {
            statusTimer_->start();
        }
    }
    publishStatus();
}

void CdprCoordinator::disconnectNokov()
{
    nokovProvider_.disconnectFromServer();
    emit logMessage(QStringLiteral("Nokov已断开。"));
    publishStatus();
}

void CdprCoordinator::captureInitialState()
{
    if (!configurationLoaded_ || !kinematicsReady_ || !kinematics_) {
        emit logMessage(QStringLiteral(
            "无法建立启动基准：请先加载有效CDPR配置并通过运动学自检。"));
        return;
    }

    CdprPlatformState6 platform;
    bool poseStable = false;
    if (initialPoseSource_ == CdprInitialPoseSource::Preset) {
        platform.pose = presetInitialPose_;
        platform.poseValid = true;
        platform.twistValid = true;
        platform.accelerationValid = true;
        poseStable = true;
    } else {
        const NokovMarkerStatus markerStatus = nokovProvider_.status();
        const CdprMarkerPoseEstimateResult estimate =
            markerPoseEstimator_.estimate(markerStatus.latestFrame);
        if (!estimate.valid) {
            emit logMessage(QStringLiteral(
                "无法从Nokov建立初始位姿：%1")
                                .arg(estimate.errorText));
            publishStatus();
            return;
        }
        platform = estimate.platform;
        poseStable = true;
    }

    const CdprInverseKinematicsResult inverse =
        kinematics_->inverse(platform);
    if (!inverse.valid) {
        emit logMessage(QStringLiteral(
            "启动位姿逆运动学失败：%1").arg(inverse.errorText));
        return;
    }
    startupState_ = {};
    startupState_.stamp = {
        ++previewSequence_, monotonicNowUs(), true
    };
    startupState_.poseSource = initialPoseSource_;
    startupState_.initialPlatform = platform;
    startupState_.initialCables = inverse.cables;
    startupState_.poseStable = poseStable;
    // 8轴编码器基准尚未采集，因此完整实机启动基准仍保持无效。
    startupState_.valid = false;

    QString error;
    if (!dynamics_.reset(platform, &error)) {
        dynamicsError_ = error;
        emit logMessage(QStringLiteral(
            "启动位姿已建立，但Newmark重置失败：%1").arg(error));
    } else {
        dynamicsError_.clear();
        robotState_.desiredPlatform = platform;
        robotState_.desiredCables = inverse.cables;
        emit logMessage(QStringLiteral(
            "已建立%1初始位姿和8绳长度基准；"
            "8轴编码器基准尚未接入，因此仍不会开放实机控制。")
                            .arg(initialPoseSource_
                                     == CdprInitialPoseSource::Preset
                                 ? QStringLiteral("预设")
                                 : QStringLiteral("Nokov")));
    }
    publishStatus();
}

void CdprCoordinator::setForceInputSource(int source)
{
    forceInputSource_ = source
            == static_cast<int>(CdprForceInputSource::TraceFtSensor)
        ? CdprForceInputSource::TraceFtSensor
        : CdprForceInputSource::Simulated;
    publishStatus();
}

void CdprCoordinator::setSimulatedSensorWrench(
    double fx, double fy, double fz,
    double mx, double my, double mz)
{
    simulatedWrenchSource_.setSensorWrench(
        {fx, fy, fz, mx, my, mz});
    emit logMessage(QStringLiteral(
        "模拟F/T输入已更新（传感器坐标系）：[%1]。")
                        .arg(vector6Text(
                            simulatedWrenchSource_.sensorWrench(), 3)));
    publishStatus();
}

void CdprCoordinator::clearSimulatedSensorWrench()
{
    simulatedWrenchSource_.setSensorWrench({});
    emit logMessage(QStringLiteral("模拟F/T输入已清零。"));
    publishStatus();
}

void CdprCoordinator::resetDynamics()
{
    QString error;
    if (!resetDynamicsFromSelectedPose(&error)) {
        dynamicsError_ = error;
        emit logMessage(QStringLiteral("Newmark重置失败：%1").arg(error));
    } else {
        dynamicsError_.clear();
        emit logMessage(QStringLiteral(
            "Newmark已按当前初始位姿重置；纯惯性、无回中刚度和阻尼。"));
    }
    publishStatus();
}

void CdprCoordinator::advanceDynamicsOnce()
{
    if (!dynamics_.initialized() && !resetDynamicsFromSelectedPose(
            &dynamicsError_)) {
        emit logMessage(QStringLiteral(
            "Newmark软件单步无法开始：%1").arg(dynamicsError_));
        publishStatus();
        return;
    }
    const CdprFrameStamp stamp {
        ++previewSequence_, monotonicNowUs(), true
    };
    const CdprWrenchTransformResult transformed =
        currentPlatformWrench(stamp);
    if (!transformed.sample.valid) {
        emit logMessage(QStringLiteral(
            "Newmark软件单步无有效外力输入：%1")
                            .arg(transformed.errorText));
        publishStatus();
        return;
    }
    const CdprDynamicsResult result = dynamics_.step(
        transformed.sample,
        static_cast<double>(configuration_.controlPeriodUs) / 1.0e6);
    if (!result.valid) {
        dynamicsError_ = result.errorText;
        emit logMessage(QStringLiteral(
            "Newmark软件单步失败：%1").arg(result.errorText));
        publishStatus();
        return;
    }

    const CdprInverseKinematicsResult inverse =
        kinematics_->inverse(result.state);
    if (!inverse.valid) {
        dynamicsError_ = inverse.errorText;
        emit logMessage(QStringLiteral(
            "Newmark结果逆运动学失败：%1").arg(inverse.errorText));
        publishStatus();
        return;
    }
    dynamicsError_.clear();
    robotState_.stamp = stamp;
    robotState_.rawWrench = forceInputSource_
            == CdprForceInputSource::Simulated
        ? simulatedWrenchSource_.sample(stamp)
        : traceFtSensorSource_.latestSample();
    robotState_.platformWrench = transformed.sample;
    robotState_.desiredPlatform = result.state;
    robotState_.desiredCables = inverse.cables;
    emit logMessage(QStringLiteral(
        "Newmark软件单步完成：序号=%1，迭代=%2，"
        "期望位姿=[%3]；未下发任何电机命令。")
                        .arg(stamp.sequence)
                        .arg(result.iterations)
                        .arg(vector6Text(result.state.pose, 6)));
    publishStatus();
}

void CdprCoordinator::prepareOfflinePvt(
    const CdprOfflinePvtRequest &request)
{
    prepareReferenceTrajectory(request, true);
}

void CdprCoordinator::prepareVelocityTrajectory(
    const CdprOfflinePvtRequest &request)
{
    prepareReferenceTrajectory(request, false);
}

void CdprCoordinator::prepareReferenceTrajectory(
    const CdprOfflinePvtRequest &request, bool enforcePvtPointLimit)
{
    CdprOfflinePvtPlan plan;
    plan.request = request;

    const auto publishPlan = [&](const CdprOfflinePvtPlan &result) {
        if (enforcePvtPointLimit) {
            emit offlinePvtPlanReady(result);
        } else {
            emit velocityTrajectoryReady(result);
        }
    };

    auto fail = [&](const QString &error) {
        plan.valid = false;
        plan.errorText = error;
        plan.summary = enforcePvtPointLimit
            ? QStringLiteral("离线PVT轨迹生成失败：%1").arg(error)
            : QStringLiteral("速度闭环参考轨迹生成失败：%1").arg(error);
        emit logMessage(plan.summary);
        publishPlan(plan);
    };

    if (!configurationLoaded_ || !kinematicsReady_ || !kinematics_) {
        fail(QStringLiteral("请先加载并通过校验的CDPR结构配置。"));
        return;
    }
    plan.configurationSnapshotJson =
        CdprConfigurationFile::serialize(configuration_);
    if (!std::isfinite(request.durationS) || request.durationS <= 0.0
        || request.samplePeriodMs <= 0
        || !std::isfinite(request.winchRadiusM)
        || request.winchRadiusM <= 0.0
        || !std::isfinite(request.maximumAxisVelocityDegreePerSecond)
        || request.maximumAxisVelocityDegreePerSecond <= 0.0
        || !std::isfinite(request.degreesPerCardUnit)
        || request.degreesPerCardUnit <= 0.0) {
        fail(QStringLiteral("轨迹时间、采样周期、绞盘半径、轴速度上限或板卡unit定义无效。"));
        return;
    }
    for (double component : request.relativePose) {
        if (!std::isfinite(component)) {
            fail(QStringLiteral("相对位姿中存在非有限数值。"));
            return;
        }
    }

    const double samplePeriodS = request.samplePeriodMs / 1000.0;
    const int intervalCount =
        std::max(1, static_cast<int>(std::ceil(request.durationS
                                               / samplePeriodS)));
    const int pointCount = intervalCount + 1;
    constexpr int kMaximumPvtPointCount = 5000;
    if (enforcePvtPointLimit && pointCount > kMaximumPvtPointCount) {
        fail(QStringLiteral("PVT点数%1超过首版上限%2；请增大采样周期或缩短轨迹时间。")
                 .arg(pointCount)
                 .arg(kMaximumPvtPointCount));
        return;
    }

    CdprPlatformState6 startPlatform;
    if (startupState_.valid && startupState_.initialPlatform.poseValid) {
        startPlatform = startupState_.initialPlatform;
    } else {
        startPlatform.pose = presetInitialPose_;
        startPlatform.poseValid = true;
    }
    startPlatform.twist = {};
    startPlatform.acceleration = {};
    startPlatform.twistValid = true;
    startPlatform.accelerationValid = true;

    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        const CdprCableAxisConfig &mapping =
            configuration_.cables[static_cast<size_t>(cable)];
        plan.axes[static_cast<size_t>(cable)] =
            static_cast<quint16>(mapping.axis);
        plan.directions[static_cast<size_t>(cable)] = mapping.direction;
        plan.axisPositionDegree[static_cast<size_t>(cable)].reserve(pointCount);
        plan.axisVelocityDegreePerSecond[static_cast<size_t>(cable)].reserve(pointCount);
    }
    plan.timeS.reserve(pointCount);

    const double maximumCableTravelM =
        CdprConfigurationFile::maximumCableTravelM(configuration_);
    CdprInverseKinematicsResult initialInverse;
    CdprInverseKinematicsResult finalInverse;
    CdprVector8 previousAxisDegree {};
    bool previousValid = false;

    for (int index = 0; index < pointCount; ++index) {
        const double timeS = index == intervalCount
            ? request.durationS
            : index * samplePeriodS;
        const double normalizedTime = qBound(0.0, timeS / request.durationS, 1.0);
        const double u2 = normalizedTime * normalizedTime;
        const double u3 = u2 * normalizedTime;
        const double u4 = u3 * normalizedTime;
        const double u5 = u4 * normalizedTime;
        const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;

        CdprPlatformState6 platform = startPlatform;
        for (int dof = 0; dof < kCdprDofCount; ++dof) {
            platform.pose[static_cast<size_t>(dof)] =
                startPlatform.pose[static_cast<size_t>(dof)]
                + blend * request.relativePose[static_cast<size_t>(dof)];
        }
        const CdprInverseKinematicsResult inverse =
            kinematics_->inverse(platform);
        if (!inverse.valid) {
            fail(QStringLiteral("第%1个轨迹点逆运动学失败：%2")
                     .arg(index)
                     .arg(inverse.errorText));
            return;
        }
        if (index == 0) {
            initialInverse = inverse;
        }
        if (index == intervalCount) {
            finalInverse = inverse;
        }

        plan.timeS.append(timeS);
        plan.platformPose.append(platform.pose);
        CdprVector8 currentAxisDegree {};
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            const size_t offset = static_cast<size_t>(cable);
            const double cableDeltaM =
                inverse.cables.lengthM[offset]
                - initialInverse.cables.lengthM[offset];
            if (std::abs(cableDeltaM) > maximumCableTravelM + 1.0e-9) {
                fail(QStringLiteral(
                    "绳%1相对启动绳长变化%2 m，超过绞盘±%3 m行程。")
                         .arg(cable)
                         .arg(cableDeltaM, 0, 'f', 6)
                         .arg(maximumCableTravelM, 0, 'f', 6));
                return;
            }
            const double axisDegree =
                plan.directions[offset] * cableDeltaM / request.winchRadiusM
                * 180.0 / kPi;
            currentAxisDegree[offset] = axisDegree;
            plan.axisPositionDegree[offset].append(axisDegree);
            if (previousValid) {
                const double dt = timeS - plan.timeS.at(index - 1);
                const double velocity =
                    std::abs(axisDegree - previousAxisDegree[offset]) / dt;
                plan.peakAxisVelocityDegreePerSecond[offset] =
                    std::max(plan.peakAxisVelocityDegreePerSecond[offset],
                             velocity);
            }
        }
        previousAxisDegree = currentAxisDegree;
        previousValid = true;
    }

    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        const size_t offset = static_cast<size_t>(cable);
        const QVector<double> &position = plan.axisPositionDegree[offset];
        QVector<double> &velocity = plan.axisVelocityDegreePerSecond[offset];
        for (int index = 0; index < position.size(); ++index) {
            double value = 0.0;
            if (index > 0 && index + 1 < position.size()) {
                const double dt = plan.timeS.at(index + 1) - plan.timeS.at(index - 1);
                value = dt > 0.0
                    ? (position.at(index + 1) - position.at(index - 1)) / dt : 0.0;
            } else if (index + 1 < position.size()) {
                const double dt = plan.timeS.at(index + 1) - plan.timeS.at(index);
                value = dt > 0.0
                    ? (position.at(index + 1) - position.at(index)) / dt : 0.0;
            } else if (index > 0) {
                const double dt = plan.timeS.at(index) - plan.timeS.at(index - 1);
                value = dt > 0.0
                    ? (position.at(index) - position.at(index - 1)) / dt : 0.0;
            }
            velocity.append(value);
        }
        if (plan.peakAxisVelocityDegreePerSecond[offset]
            > request.maximumAxisVelocityDegreePerSecond + 1.0e-9) {
            fail(QStringLiteral("轴%1峰值离散速度%2°/s超过上限%3°/s。")
                     .arg(plan.axes[offset])
                     .arg(plan.peakAxisVelocityDegreePerSecond[offset], 0, 'f', 3)
                     .arg(request.maximumAxisVelocityDegreePerSecond, 0, 'f', 3));
            return;
        }
        plan.finalAxisDisplacementDegree[offset] =
            plan.axisPositionDegree[offset].constLast();
    }

    plan.initialCables = initialInverse.cables;
    plan.finalCables = finalInverse.cables;
    if (!enforcePvtPointLimit) {
        QString cacheError;
        const QString cacheRoot = QDir(QCoreApplication::applicationDirPath())
                                      .filePath(QStringLiteral("trajectory_cache"));
        if (!CdprTrajectoryFile::prepareCache(plan, cacheRoot, cacheError)) {
            fail(QStringLiteral("期望轨迹缓存失败：%1").arg(cacheError));
            return;
        }
    }
    plan.valid = true;
    const double maximumPeakVelocity = *std::max_element(
        plan.peakAxisVelocityDegreePerSecond.begin(),
        plan.peakAxisVelocityDegreePerSecond.end());
    if (enforcePvtPointLimit) {
        plan.summary = QStringLiteral(
            "离线PVT轨迹有效：%1点，%2 s，采样%3 ms；8轴最大峰值速度%4°/s。")
                           .arg(pointCount)
                           .arg(request.durationS, 0, 'f', 3)
                           .arg(request.samplePeriodMs)
                           .arg(maximumPeakVelocity, 0, 'f', 3);
    } else {
        plan.summary = QStringLiteral(
            "速度闭环参考轨迹及缓存有效：%1点，%2 s，控制周期%3 ms；"
            "8轴最大峰值速度%4°/s；缓存ID=%5。")
                           .arg(pointCount)
                           .arg(request.durationS, 0, 'f', 3)
                           .arg(request.samplePeriodMs)
                           .arg(maximumPeakVelocity, 0, 'f', 3)
                           .arg(plan.planId.left(12));
    }
    emit logMessage(plan.summary);
    publishPlan(plan);
}

bool CdprCoordinator::resetDynamicsFromSelectedPose(QString *errorText)
{
    if (!configurationLoaded_ || !dynamics_.configured()) {
        if (errorText) {
            *errorText = QStringLiteral("CDPR配置或Newmark参数尚未就绪。");
        }
        return false;
    }
    CdprPlatformState6 initial;
    if (initialPoseSource_ == CdprInitialPoseSource::Preset) {
        initial.pose = presetInitialPose_;
        initial.poseValid = true;
        initial.twistValid = true;
        initial.accelerationValid = true;
    } else {
        const NokovMarkerStatus markerStatus = nokovProvider_.status();
        const CdprMarkerPoseEstimateResult estimate =
            markerPoseEstimator_.estimate(markerStatus.latestFrame);
        if (!estimate.valid) {
            if (errorText) {
                *errorText = estimate.errorText;
            }
            return false;
        }
        initial = estimate.platform;
    }
    if (!dynamics_.reset(initial, errorText)) {
        return false;
    }
    if (errorText) {
        errorText->clear();
    }
    return true;
}

CdprWrenchTransformResult CdprCoordinator::currentPlatformWrench(
    const CdprFrameStamp &stamp) const
{
    CdprWrenchTransformResult result;
    if (!wrenchTransformer_) {
        result.errorText = QStringLiteral("F/T安装变换尚未配置。");
        return result;
    }
    CdprWrenchSample raw;
    if (forceInputSource_ == CdprForceInputSource::Simulated) {
        raw = simulatedWrenchSource_.sample(stamp);
    } else {
        raw = traceFtSensorSource_.latestSample();
    }
    return wrenchTransformer_->toPlatformCenterOfMass(raw);
}

void CdprCoordinator::rebuildInitialKinematics()
{
    kinematics_.reset();
    wrenchTransformer_.reset();
    startupState_ = {};
    kinematicsReady_ = false;
    kinematicsSummary_.clear();
    kinematicsError_.clear();
    dynamicsError_.clear();
    robotState_ = {};
    if (!configurationLoaded_) {
        return;
    }

    presetInitialPose_ = configuration_.presetInitialPlatformPose;
    wrenchTransformer_ =
        std::make_unique<CdprWrenchTransformer>(
            configuration_.forceSensor);
    QString dynamicsConfigurationError;
    if (!dynamics_.configure(configuration_.physicalPlatform,
                             CdprNewmarkConfig {},
                             &dynamicsConfigurationError)) {
        dynamicsError_ = dynamicsConfigurationError;
    }

    kinematics_ = std::make_unique<CdprKinematics>(configuration_);
    QString geometryError;
    if (!kinematics_->geometryValid(&geometryError)) {
        kinematicsError_ =
            QStringLiteral("直线绳段运动学无效：%1").arg(geometryError);
        return;
    }

    CdprPlatformState6 initialPlatform;
    initialPlatform.pose = configuration_.presetInitialPlatformPose;
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
    CdprVector6 forwardGuess = configuration_.presetInitialPlatformPose;
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
                     - configuration_.presetInitialPlatformPose[
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
    if (dynamicsError_.isEmpty()) {
        QString resetError;
        if (!dynamics_.reset(initialPlatform, &resetError)) {
            dynamicsError_ = resetError;
        }
    }
    startupState_.poseSource = CdprInitialPoseSource::Preset;
    startupState_.initialPlatform = initialPlatform;
    startupState_.initialCables = inverse.cables;
    startupState_.poseStable = true;

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
    if (!dynamicsError_.isEmpty()) {
        status.validationMessages.append(
            QStringLiteral("Newmark：%1").arg(dynamicsError_));
    }
    status.onlineAxisCount = detectedAxisCount_;
    status.boardInitialized = boardInitialized_;
    status.ethercatOperational = ethercatOperational_;
    status.kinematicsReady = kinematicsReady_;
    status.dynamicsReady =
        dynamics_.configured() && dynamics_.initialized()
        && dynamicsError_.isEmpty();
    status.initialPoseSource = initialPoseSource_;
    status.forceInputSource = forceInputSource_;
    status.presetInitialPose = presetInitialPose_;
    status.simulatedSensorWrench =
        simulatedWrenchSource_.sensorWrench();
    status.dynamicsState = dynamics_.currentState();
    status.dynamicsText = dynamicsError_.isEmpty()
        ? (status.dynamicsReady
               ? QStringLiteral("Newmark就绪（β=0.25，γ=0.5，纯惯性）")
               : QStringLiteral("Newmark等待初始状态"))
        : QStringLiteral("Newmark错误：%1").arg(dynamicsError_);
    status.initialPoseReady =
        initialPoseSource_ == CdprInitialPoseSource::Preset
        ? kinematicsReady_
        : false;
    const CdprFrameStamp displayStamp {0, monotonicNowUs(), true};
    const CdprWrenchTransformResult displayWrench =
        currentPlatformWrench(displayStamp);
    status.forceInputReady = displayWrench.sample.valid;
    if (displayWrench.sample.valid) {
        status.platformWrench = displayWrench.sample.wrench;
    }
    status.traceFtText = traceFtSensorSource_.statusText();
    const NokovMarkerStatus nokovStatus = nokovProvider_.status();
    status.nokovConnected = nokovStatus.connected;
    status.nokovFrameNumber = nokovStatus.latestFrame.frameNumber;
    status.nokovFrameAgeMs = nokovStatus.frameAgeMs;
    status.nokovText = nokovStatus.connected
        ? (nokovStatus.latestFrame.valid
               ? QStringLiteral("已连接，标记点帧有效；位姿重建待实现")
               : QStringLiteral("已连接，等待标记点帧"))
        : (nokovStatus.errorText.isEmpty()
               ? QStringLiteral("未连接")
               : nokovStatus.errorText);
    status.markers.reserve(nokovStatus.latestFrame.markers.size());
    for (const NokovMarkerSample &marker
         : nokovStatus.latestFrame.markers) {
        CdprMarkerView view;
        view.id = marker.id;
        view.positionM = marker.positionM;
        status.markers.append(view);
    }
    status.controlStartAvailable = false;

    quint16 mappedAxisMask = 0;
    bool allMappedAxesOnline =
        status.configurationValid
        && boardInitialized_
        && ethercatOperational_
        && configuration_.cables.size() == 8;
    if (allMappedAxesOnline) {
        for (const CdprCableAxisConfig &cable : configuration_.cables) {
            if (cable.axis < 0 || cable.axis >= detectedAxisCount_
                || cable.axis >= 8
                || (mappedAxisMask
                    & static_cast<quint16>(1U << cable.axis)) != 0U) {
                allMappedAxesOnline = false;
                break;
            }
            mappedAxisMask |= static_cast<quint16>(1U << cable.axis);
        }
    }
    status.hardwareReady = allMappedAxesOnline;
    status.allMappedAxesEnabled =
        allMappedAxesOnline
        && (enabledAxisMask_ & mappedAxisMask) == mappedAxisMask;

    if (!configurationLoaded_) {
        status.stateText = QStringLiteral("未加载配置");
    } else if (!status.configurationValid) {
        status.stateText = QStringLiteral("配置校验未通过");
    } else if (!kinematicsReady_) {
        status.stateText = QStringLiteral("运动学自检未通过");
    } else if (!status.dynamicsReady) {
        status.stateText = QStringLiteral("动力学模块未就绪");
    } else if (!boardInitialized_ || !ethercatOperational_) {
        status.stateText =
            QStringLiteral("配置有效，等待控制卡与EtherCAT总线就绪");
    } else if (!status.hardwareReady) {
        status.stateText =
            QStringLiteral("配置有效，CDPR整机硬件未就绪（在线%1/8）")
                .arg(detectedAxisCount_);
    } else if (!status.allMappedAxesEnabled) {
        status.stateText =
            QStringLiteral("CDPR整机硬件就绪（8/8），电机未全部使能");
    } else {
        status.stateText =
            QStringLiteral("CDPR整机硬件就绪，8轴已全部使能");
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
            axis.online = boardInitialized_ && ethercatOperational_
                && cable.axis >= 0 && cable.axis < detectedAxisCount_;
            axis.enabled = axis.online
                && (enabledAxisMask_
                    & static_cast<quint16>(1U << cable.axis)) != 0U;
            status.axes.append(axis);
        }
    }
    emit statusChanged(status);
}
