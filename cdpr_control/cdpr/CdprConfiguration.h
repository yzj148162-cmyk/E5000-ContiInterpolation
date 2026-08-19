#ifndef CDPRCONFIGURATION_H
#define CDPRCONFIGURATION_H

#include <array>

#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVector>

#include "CdprControlTypes.h"

struct CdprCableAxisConfig
{
    int cable = 0;
    int axis = 0;
    int direction = 1;
    CdprVector3 frameAnchorM;
    CdprVector3 platformAnchorM;
};

struct CdprRigidBodyConfig
{
    double massKg = 0.0;
    CdprVector3 centerOfMassM;
    std::array<double, 9> inertiaKgM2 {};
};

struct CdprForceSensorConfig
{
    CdprVector3 originInPlatformM;
    std::array<double, 9> rotationSensorToPlatform {};
    // +1：设备输出已经表示“环境作用于平台”的力旋量；
    // -1：设备输出为相反的作用/反作用定义，六个分量整体反号。
    int wrenchReactionSign = 1;
    std::array<double, 6> channelScale {1, 1, 1, 1, 1, 1};
    std::array<double, 6> channelBias {};
};

struct CdprWinchSafetyConfig
{
    double diameterM = 0.16;
    double maximumTurnsFromInitial = 6.5;
};

struct CdprConfiguration
{
    QString name;
    QString coordinateConvention;
    bool parametersConfirmed = false;
    // 仅作为无Nokov时的预设启动值和离线运动学自检参考，
    // 实机启动基准必须由CdprStartupState显式建立。
    std::array<double, 6> presetInitialPlatformPose {};
    CdprRigidBodyConfig physicalPlatform;
    CdprForceSensorConfig forceSensor;
    CdprWinchSafetyConfig winchSafety;
    std::array<CdprCableAxisConfig, 8> cables {};
    std::array<double, 8> referenceCableLengthsM {};
    int controlPeriodUs = 1000;
    double maximumPositionErrorDegree = 2.0;
};

// 连续力交互启动时由CdprCoordinator生成的一次性只读快照。
// 运行周期只更新模拟力样本，结构参数和初始位姿在本次运行中保持不变。
struct CdprForceControlRequest
{
    bool valid = false;
    QString errorText;
    CdprConfiguration configuration;
    CdprPlatformState6 initialPlatform;
    CdprCableState8 initialCables;
    CdprVector6 initialSimulatedSensorWrench {};
    CdprVelocityControlConfig velocityControl;
};

struct CdprForceControlStatus
{
    bool active = false;
    bool motionStarted = false;
    quint64 runId = 0;
    double elapsedS = 0.0;
    quint64 cycleCount = 0;
    double averageFullCycleMs = 0.0;
    double maximumFullCycleMs = 0.0;
    double averageTracePollMs = 0.0;
    double maximumTracePollMs = 0.0;
    double averageCalculationMs = 0.0;
    double maximumCalculationMs = 0.0;
    double averageApiTotalMs = 0.0;
    double maximumApiTotalMs = 0.0;
    quint64 executionOverrunCount = 0;
    quint64 schedulingOverrunCount = 0;
    double maximumTrackingErrorDegree = 0.0;
    double maximumCableTravelM = 0.0;
    CdprVector6 simulatedSensorWrench {};
    CdprPlatformState6 desiredPlatform;
    QString stateText = QStringLiteral("未运行");
};

struct CdprMarkerView
{
    int id = -1;
    CdprVector3 positionM;
};

struct CdprAxisView
{
    int cable = 0;
    int axis = 0;
    int direction = 1;
    QString frameAnchor;
    QString platformAnchor;
    bool online = false;
    bool enabled = false;
};

struct CdprUiStatus
{
    bool configurationLoaded = false;
    bool configurationValid = false;
    QString configurationPath;
    QString configurationId;
    QString stateText = QStringLiteral("未加载配置");
    QString summary;
    QStringList validationMessages;
    QVector<CdprAxisView> axes;
    int onlineAxisCount = 0;
    bool boardInitialized = false;
    bool ethercatOperational = false;
    bool hardwareReady = false;
    bool allMappedAxesEnabled = false;
    bool kinematicsReady = false;
    bool dynamicsReady = false;
    bool initialPoseReady = false;
    bool forceInputReady = false;
    CdprInitialPoseSource initialPoseSource = CdprInitialPoseSource::Preset;
    CdprForceInputSource forceInputSource = CdprForceInputSource::Simulated;
    CdprVector6 presetInitialPose {};
    CdprVector6 simulatedSensorWrench {};
    CdprVector6 platformWrench {};
    CdprPlatformState6 dynamicsState;
    QString dynamicsText = QStringLiteral("未配置");
    QString nokovText = QStringLiteral("未连接");
    QString traceFtText = QStringLiteral("Trace F/T对象类型待配置");
    bool nokovConnected = false;
    qint64 nokovFrameNumber = -1;
    qint64 nokovFrameAgeMs = -1;
    QVector<CdprMarkerView> markers;
    bool controlStartAvailable = false;
};

class CdprConfigurationFile
{
public:
    static bool load(const QString &path, CdprConfiguration &configuration,
                     QStringList &errors);
    // 写入运行记录配置快照。该快照与配置文件格式一致，可被load()直接读回。
    static QString serialize(const CdprConfiguration &configuration);
    static bool writeTemplate(const QString &path, QString &error);
    static QStringList validate(const CdprConfiguration &configuration);
    static QString identifier(const CdprConfiguration &configuration);
    static QString summary(const CdprConfiguration &configuration);
    static std::array<double, 8> calculateInitialCableLengths(
        const CdprConfiguration &configuration);
    static double maximumCableTravelM(
        const CdprConfiguration &configuration);
    static bool cableTravelWithinLimit(
        const CdprConfiguration &configuration,
        const std::array<double, 8> &runtimeInitialCableLengthsM,
        int cableIndex,
        double currentCableLengthM, double *travelFromInitialM = nullptr);
};

Q_DECLARE_METATYPE(CdprUiStatus)
Q_DECLARE_METATYPE(CdprForceControlRequest)
Q_DECLARE_METATYPE(CdprForceControlStatus)

#endif // CDPRCONFIGURATION_H
