#ifndef FORCEINTERACTIONRUNRECORDER_H
#define FORCEINTERACTIONRUNRECORDER_H

#include "forceinteractiontypes.h"

#include <array>
#include <atomic>
#include <vector>

#include <QMutex>
#include <QSemaphore>
#include <QString>
#include <QThread>
#include <QWaitCondition>

enum ForceInteractionRecordAvailability : quint32
{
    ForceRecordSensorWrench = 1u << 0,
    ForceRecordPlatformWrench = 1u << 1,
    ForceRecordDesiredState = 1u << 2,
    ForceRecordCableKinematics = 1u << 3,
    ForceRecordForwardKinematics = 1u << 4,
    ForceRecordAxisReference = 1u << 5,
    ForceRecordAxisCommand = 1u << 6,
    ForceRecordAxisTrace = 1u << 7,
    ForceRecordTiming = 1u << 8
};

struct ForceInteractionRunMetadata
{
    QString stage;
    QString sourceName;
    QString machineTemplateName;
    double controlPeriodS = 0.0;
    double plannedDurationS = 0.0;
};

// 固定字段覆盖阶段A～D。某阶段尚不存在的数据由 availabilityMask 明确标为
// 不可用，避免后续扩展时反复改变CSV列定义。
struct ForceInteractionRunRecord
{
    quint64 stepIndex = 0;
    double elapsedS = 0.0;
    ForceInteractionFrameStamp stamp;
    quint32 availabilityMask = 0;

    ForceInteractionVector6 sensorWrench{};
    ForceInteractionVector6 platformWrench{};
    ForceInteractionPlatformState desiredState;
    std::array<double, kForceInteractionCableCount> cableLengthMm{};
    std::array<double, kForceInteractionCableCount> relativeMotorThetaRad{};
    ForceInteractionVector6 forwardPoseMmRad{};

    double translationRoundTripErrorMm = 0.0;
    double orientationRoundTripErrorDeg = 0.0;
    double maximumCableResidualMm = 0.0;
    int newmarkIterations = 0;
    double newmarkResidual = 0.0;
    bool poseBoundsViolation = false;
    bool roundTripToleranceViolation = false;

    std::array<double, kForceInteractionCableCount> axisReferencePosition{};
    std::array<double, kForceInteractionCableCount> axisReferenceVelocity{};
    std::array<double, kForceInteractionCableCount> axisPidCorrectionVelocity{};
    std::array<double, kForceInteractionCableCount> axisCommandVelocity{};
    std::array<double, kForceInteractionCableCount> axisTracePosition{};
    std::array<double, kForceInteractionCableCount> axisTraceVelocity{};

    qint64 calculationDurationUs = 0;
    qint64 hardwareApiDurationUs = 0;
    qint64 fullCycleDurationUs = 0;
};

// 六维力交互专用的有界异步CSV记录器。控制/计算线程只做一次定长结构复制；
// 队列争用或写盘跟不上时丢弃记录并计数，绝不等待磁盘。
class ForceInteractionRunRecorder final : public QThread
{
public:
    explicit ForceInteractionRunRecorder(QObject* parent = nullptr);
    ~ForceInteractionRunRecorder() override;

    bool begin(const QString& directory,
               const ForceInteractionRunMetadata& metadata,
               QString* outputPath = nullptr,
               QString* errorMessage = nullptr);
    void tryAppend(const ForceInteractionRunRecord& record);
    void requestFinish();
    void finishAndWait();

    QString outputPath() const;
    QString writerError() const;
    quint64 acceptedCount() const;
    quint64 writtenCount() const;
    quint64 droppedCount() const;

protected:
    void run() override;

private:
    static constexpr std::size_t kMaximumQueuedRecords = 65536;

    QString filePath_;
    QString openError_;
    QString writerError_;
    ForceInteractionRunMetadata metadata_;
    QMutex queueMutex_;
    QWaitCondition queueReady_;
    QSemaphore ready_;
    std::vector<ForceInteractionRunRecord> queue_;
    std::size_t queueHead_ = 0;
    std::size_t queuedRecordCount_ = 0;
    std::atomic_bool stopRequested_{false};
    std::atomic_bool openSucceeded_{false};
    std::atomic<quint64> accepted_{0};
    std::atomic<quint64> written_{0};
    std::atomic<quint64> dropped_{0};
};

#endif // FORCEINTERACTIONRUNRECORDER_H
