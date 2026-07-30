#include "CdprCoordinator.h"

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
        emit logMessage(QStringLiteral("CDPR配置加载失败：%1")
                            .arg(messages.join(QStringLiteral("；"))));
        publishStatus();
        return;
    }

    configuration_ = candidate;
    configurationPath_ = path;
    configurationLoaded_ = true;
    validationMessages_ = messages;
    emit logMessage(messages.isEmpty()
        ? QStringLiteral("CDPR配置已加载并通过校验：%1").arg(path)
        : QStringLiteral("CDPR配置已加载，但有%1项校验问题。")
              .arg(messages.size()));
    publishStatus();
}

void CdprCoordinator::validateConfiguration()
{
    if (!configurationLoaded_) {
        validationMessages_ = {QStringLiteral("请先加载CDPR配置文件。")};
    } else {
        validationMessages_ = CdprConfigurationFile::validate(configuration_);
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
    status.validationMessages = validationMessages_;
    status.onlineAxisCount = detectedAxisCount_;
    status.boardInitialized = boardInitialized_;
    status.controlStartAvailable = false;

    if (!configurationLoaded_) {
        status.stateText = QStringLiteral("未加载配置");
    } else if (!status.configurationValid) {
        status.stateText = QStringLiteral("配置校验未通过");
    } else if (!boardInitialized_) {
        status.stateText = QStringLiteral("配置有效，等待控制卡初始化");
    } else if (detectedAxisCount_ < 8) {
        status.stateText =
            QStringLiteral("配置有效，在线轴不足8个（%1/8）")
                .arg(detectedAxisCount_);
    } else {
        status.stateText =
            QStringLiteral("阶段0骨架就绪，运动控制尚未接入");
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
