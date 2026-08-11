#ifndef MOTIONCONTROLWORKER_H
#define MOTIONCONTROLWORKER_H

#include <QQueue>
#include <QSet>
#include <QObject>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QJsonObject>
#include <QJsonValue>
#include <QMap>

#include "common/ContiTypes.h"
#include "cdpr/CdprControlTypes.h"
#include "cdpr/CdprVirtualConsistencyAnalyzer.h"
#include "control/PositionVelocityPid.h"
#include "control/TraceDelayCalibration.h"
#include "hardware/MotionCardHardwareInterface.h"
#include "planner/QuinticTrajectory.h"
#include "telemetry/TelemetryRecorder.h"

class QTimer;

// 上层规划、运行状态机与反馈处理线程；不直接调用 LTDMC。
// 所有 SDK 调用以及运行中的定时补段均由 MotionCardHardwareInterface 的独占线程执行。
class MotionControlWorker : public QObject
{
    Q_OBJECT

public:
    explicit MotionControlWorker(QObject *parent = nullptr);

public slots:
    void initializeBoard(const ContiTestConfig &config);
    void closeBoard();
    void enableSelectedAxes(const ContiTestConfig &config);
    void disableSelectedAxes(const ContiTestConfig &config);
    void enableAllDetectedAxes();
    void disableAllDetectedAxes();
    void startTest(const ContiTestConfig &config);
    void stopTest(bool emergency);
    void refreshFeedback();
    void enableJogAxis(const SingleAxisJogConfig &config);
    void disableJogAxis(const SingleAxisJogConfig &config);
    void setJogAxisZero(const SingleAxisJogConfig &config);
    void startPointMove(const SingleAxisJogConfig &config);
    void stopPointMove(bool emergency);
    void startVelocityControl(const VelocityControlConfig &config);
    void stopVelocityControl(bool emergency);
    void resetVelocityController();
    void writeTorqueVelocityLimit(const TorqueTestConfig &config);
    void startTorqueTest(const TorqueTestConfig &config);
    void updateTorqueCommand(const TorqueTestConfig &config);
    void stopTorqueTest(bool emergency);
    void startTraceDelayCalibration(const TraceDelayCalibrationConfig &config);
    void stopTraceDelayCalibration(bool emergency);
    void resetTraceDelayCalibrationAxis(quint16 axis);
    void startOfflinePvt(const CdprOfflinePvtPlan &plan);
    void stopOfflinePvt(bool emergency);
    void startCdprVelocityControl(const CdprOfflinePvtPlan &plan,
                                  const CdprVelocityControlConfig &config);
    void stopCdprVelocityControl(bool emergency);
    void startTelemetryRecording();
    void stopTelemetryRecording();
    void refreshBusCycle();
    void shutdownHardware();

signals:
    void logMessage(const QString &message);
    void statusChanged(const ContiStatus &status);
    void velocityPlotSamplesReady(const QVector<VelocityPlotSample> &samples);
    void torquePlotSamplesReady(const QVector<TorquePlotSample> &samples);
    void offlinePvtStatusChanged(const CdprOfflinePvtStatus &status);
    void cdprVelocityControlStatusChanged(const CdprVelocityControlStatus &status);

private slots:
    void produceNextPoint();
    void monitorContinuousRun();
    void runVelocityControlCycle();
    void runTorqueTestCycle();
    void runTraceDelayCalibrationCycle();
    void monitorOfflinePvt();
    void runCdprVelocityControlCycle();

private:
    bool startAfterPreload();
    void submitGeneratedPoints();
    void enqueueTrajectoryPoint(const ContiPoint &point);
    bool generateAllTrajectoryPoints();
    bool hasExecutionDelayReady() const;
    double planTimeForMark(long currentMark) const;
    double referenceVectorSpeed(double planTimeS) const;
    double calculateRatioCommand(long currentMark, double bufferTimeS, qint64 elapsedMs);
    bool applyRatioCommand(qint64 elapsedMs, QString &errorMessage);
    bool traceReachedFinalTarget() const;
    void finishRun(const QString &message);
    void enterError(const QString &message);
    void resetRunTimingState();
    void publishStatus();
    bool configureBaseAxes(const ContiTestConfig &config);
    bool configureFeedbackTrace(const QVector<quint16> &axes, double degreesPerCardUnit,
                                QString &errorMessage,
                                TraceFeedbackProfile profile =
                                    TraceFeedbackProfile::FullPositionVelocity,
                                bool allowEmptyActiveRecording = false);
    bool pollTraceFeedback();
    void updateTraceVelocityDiagnostics(const QVector<TraceTelemetryFrame> &frames);
    void refreshAxisStates();
    bool bothAxesEnabled(const ContiTestConfig &config) const;
    bool waitForAxisStop(quint16 axis, int timeoutMs) const;
    bool waitForContiStop(int timeoutMs) const;
    bool safelyStopPointAxis(quint16 axis, const QString &reason);
    void safelyStopAllMotionForShutdown();
    void disableAllEnabledAxesForShutdown();
    bool validateVelocityControlConfig(const VelocityControlConfig &config,
                                       QString &errorMessage) const;
    void evaluateVelocityReference(double elapsedS,
                                   double &positionDegree,
                                   double &velocityDegreePerSecond) const;
    void finishVelocityControl(const QString &message, bool emergency = false);
    void appendVelocityPlotFrames(const QVector<TraceTelemetryFrame> &frames);
    void flushVelocityPlotSamples();
    bool validateTorqueTestConfig(const TorqueTestConfig &config,
                                  QString &errorMessage) const;
    int torquePercentToRaw(double torquePercent) const;
    void finishTorqueTest(const QString &message, bool emergency = false);
    void flushTorquePlotSamples();
    bool validateTraceDelayCalibrationConfig(const TraceDelayCalibrationConfig &config,
                                             QString &errorMessage) const;
    bool startNextTraceDelaySegment(QString &errorMessage);
    void analyzeTraceDelayCalibration();
    void finishTraceDelayCalibration(const QString &message,
                                     bool failed = false,
                                     bool emergency = false);
    void appendTraceDelayCalibrationFrames(const QVector<TraceTelemetryFrame> &frames);
    void resetTraceDelayHistory();
    void finishOfflinePvt(const QString &message,
                          CdprOfflinePvtRunState state,
                          bool emergency = false);
    bool validateCdprVelocityControl(const CdprOfflinePvtPlan &plan,
                                     const CdprVelocityControlConfig &config,
                                     QString &errorMessage) const;
    bool cdprVelocityReference(double elapsedS, int cable,
                               double &positionDegree,
                               double &velocityDegreePerSecond) const;
    void finishCdprVelocityControl(const QString &message, bool emergency);
    bool beginCdprRunRecording(const CdprOfflinePvtPlan &plan,
                               const QString &mode,
                               QString &errorMessage);
    bool exportCdprExpectedTrajectory(const CdprOfflinePvtPlan &plan,
                                      const QString &directory,
                                      QString &errorMessage) const;
    bool writeCdprRunContext(QString &errorMessage);
    void updateCdprRunContext(const QString &key, const QJsonValue &value);
    void startCdprVirtualConsistencyAnalysis(const QString &directory);
    void finishCdprRunRecording(const QString &eventText, bool &autoRecordingFlag);
    QString cdprVelocityPerformanceSummary(const QString &prefix) const;
    void applyTraceDelayCompensation(const QVector<TraceTelemetryFrame> &frames);
    void loadTraceDelayCalibrationResults();
    void saveTraceDelayCalibrationResults();
    void validateLoadedTraceDelayTiming();
    QString traceDelayCalibrationFilePath() const;

private:
    enum class TraceDelayPhase
    {
        Idle,
        Resting,
        Moving,
        Stopping
    };

    struct TraceCommandHistorySample
    {
        quint64 traceTimeUs = 0;
        double commandPositionDegree = 0.0;
    };

    // 控制线程只保存状态机与轨迹数据；SDK、Trace PDO 和控制卡状态均在
    // MotionCardHardwareInterface 的独占硬件线程内。
    MotionCardHardwareInterface card_;
    TelemetryRecorder telemetryRecorder_;
    QuinticTrajectory trajectory_;
    QQueue<ContiPoint> hostQueue_;
    QSet<quint16> enabledAxes_;
    QVector<quint16> detectedAxes_;
    QVector<quint16> traceAxes_;
    TraceFeedbackProfile traceFeedbackProfile_ =
        TraceFeedbackProfile::FullPositionVelocity;
    int traceSamplePeriodUs_ = 1000;
    QVector<AxisFeedback> latestAxisFeedback_;
    QTimer *producerTimer_ = nullptr;
    QTimer *monitorTimer_ = nullptr;
    QTimer *feedbackTimer_ = nullptr;
    QTimer *velocityControlTimer_ = nullptr;
    QTimer *torqueTestTimer_ = nullptr;
    QTimer *traceDelayCalibrationTimer_ = nullptr;
    QTimer *offlinePvtMonitorTimer_ = nullptr;
    QTimer *cdprVelocityControlTimer_ = nullptr;
    ContiTestConfig config_;
    ContiFeedStatus lastFeedStatus_;
    bool boardInitialized_ = false;
    bool ethercatOperational_ = false;
    quint16 controllerWorkMode_ = 0;
    quint32 ethercatMasterState_ = 0;
    short detectedBoardCount_ = 0;
    quint16 initializedCardNo_ = 0;
    int actualBusCycleUs_ = 0;
    bool listOpen_ = false;
    bool preparing_ = false;
    bool running_ = false;
    bool pointMoveActive_ = false;
    bool velocityControlActive_ = false;
    bool torqueTestActive_ = false;
    bool torqueMotionStarted_ = false;
    bool velocityMotionStarted_ = false;
    bool velocityReferenceInitialized_ = false;
    bool velocityAutoRecording_ = false;
    bool traceDelayCalibrationActive_ = false;
    bool traceDelayMotionStarted_ = false;
    bool traceDelayAutoRecording_ = false;
    bool offlinePvtActive_ = false;
    bool cdprVelocityControlActive_ = false;
    bool manualTelemetryRecording_ = false;
    quint64 velocityRunId_ = 0;
    VelocityControlConfig velocityConfig_;
    VelocityControlStatus velocityStatus_;
    PositionVelocityPid velocityPid_;
    double velocityStartPositionDegree_ = 0.0;
    double velocityFinalPositionDegree_ = 0.0;
    QElapsedTimer velocityRunClock_;
    QElapsedTimer velocityCycleClock_;
    QElapsedTimer velocityTraceFreshClock_;
    QElapsedTimer velocityAlignedErrorFreshClock_;
    QElapsedTimer velocityCompletionClock_;
    quint64 velocityLastTraceSequence_ = 0;
    quint64 velocityLastFeedbackTraceTimeUs_ = 0;
    double velocityFeedbackElapsedS_ = 0.0;
    double velocityFeedbackReferencePositionDegree_ = 0.0;
    double velocityFeedbackReferenceVelocityDegreePerSecond_ = 0.0;
    double velocityFeedbackPositionErrorDegree_ = 0.0;
    double velocityFeedbackDtSeconds_ = 0.001;
    bool velocityFeedbackReferenceValid_ = false;
    qint64 velocityLastDiagnosticMs_ = -1;
    QVector<VelocityPlotSample> pendingVelocityPlotSamples_;
    QElapsedTimer velocityPlotPublishClock_;
    quint64 velocityPlotTraceStartTimeUs_ = 0;
    bool velocityPlotTraceStartValid_ = false;
    QQueue<TraceCommandHistorySample> velocityPlotCommandHistory_;
    quint64 velocityPlotLastTraceSequence_ = 0;
    // 当前批次内按 Trace 时间戳对齐后的最大轨迹跟踪误差：
    // reference(t - delay) - traceActual(t)。只用于失控保护；
    // type05/type06 延迟对齐误差继续作为电机执行层诊断量。
    bool velocityBatchAlignedTrackingErrorValid_ = false;
    double velocityBatchPeakAlignedTrackingErrorDegree_ = 0.0;
    quint64 torqueRunId_ = 0;
    TorqueTestConfig torqueConfig_;
    TorqueTestStatus torqueStatus_;
    QElapsedTimer torqueRunClock_;
    QElapsedTimer torqueTraceFreshClock_;
    QElapsedTimer torquePlotPublishClock_;
    quint64 torqueLastTraceSequence_ = 0;
    qint64 torqueLastDiagnosticMs_ = -1;
    bool torquePostStartDiagnosticPending_ = false;
    QVector<TorquePlotSample> pendingTorquePlotSamples_;
    TraceDelayCalibrationConfig traceDelayConfig_;
    TraceDelayCalibrationStatus traceDelayStatus_;
    QVector<TraceDelayAxisResult> traceDelayAxisResults_;
    QVector<double> traceDelaySegmentTargets_;
    QVector<TraceDelaySegmentCapture> traceDelaySegments_;
    QVector<TraceTelemetryFrame> traceDelayCurrentSegmentFrames_;
    TraceDelayPhase traceDelayPhase_ = TraceDelayPhase::Idle;
    int traceDelayCurrentSegmentIndex_ = 0;
    QElapsedTimer traceDelayPhaseClock_;
    std::array<QQueue<TraceCommandHistorySample>, 8> traceCommandHistory_;
    quint64 lastTraceDelaySequence_ = 0;
    int savedCalibrationBusCycleUs_ = 0;
    int savedCalibrationTracePeriodUs_ = 0;
    CdprOfflinePvtPlan offlinePvtPlan_;
    CdprOfflinePvtStatus offlinePvtStatus_;
    QVector<quint16> offlinePvtAxes_;
    QElapsedTimer offlinePvtRunClock_;
    bool offlinePvtAutoRecording_ = false;
    bool offlinePvtTraceAvailable_ = false;
    bool offlinePvtTraceWarningReported_ = false;
    quint64 offlinePvtStartRequestPreTraceSequence_ = 0;
    CdprOfflinePvtPlan cdprVelocityPlan_;
    CdprVelocityControlConfig cdprVelocityConfig_;
    CdprVelocityControlStatus cdprVelocityStatus_;
    std::array<PositionVelocityPid, kCdprCableCount> cdprVelocityPids_;
    std::array<double, kCdprCableCount> cdprVelocityStartPositionDegree_ {};
    std::array<int, kCdprCableCount> cdprVelocityDirection_ {};
    std::array<bool, kCdprCableCount> cdprVelocityAxisStarted_ {};
    std::array<quint64, kCdprCableCount> cdprVelocityCommandStartTraceSequence_ {};
    std::array<quint64, kCdprCableCount> cdprVelocityLastTraceTimeUs_ {};
    QElapsedTimer cdprVelocityRunClock_;
    QElapsedTimer cdprVelocityCycleClock_;
    QElapsedTimer cdprVelocityTraceFreshClock_;
    QElapsedTimer cdprVelocityStatusPublishClock_;
    QElapsedTimer cdprVelocityPerformanceLogClock_;
    quint64 cdprVelocityLastTraceSequence_ = 0;
    // 八轴控制以 cdprVelocityRunClock_ 的主机单调时间推进。首帧序号只用于
    // 将 Trace 相对时间映射回该主机运行起点，以计算延迟对齐跟随误差。
    quint64 cdprVelocityStartTraceSequence_ = 0;
    bool cdprVelocityProtectionArmedLogged_ = false;
    quint64 cdprVelocitySchedulingSampleCount_ = 0;
    quint64 cdprVelocityRunId_ = 0;
    bool cdprVelocityAutoRecording_ = false;
    QString cdprRunRecordDirectory_;
    QJsonObject cdprRunContext_;
    QFutureWatcher<CdprVirtualConsistencyAnalysisResult> *cdprAnalysisWatcher_ = nullptr;
    quint16 pointMoveAxis_ = 0;
    bool pointMoveDiagnosticPending_ = false;
    double pointMoveRequestedTargetUnit_ = 0.0;
    double pointMoveCardTargetUnit_ = 0.0;
    bool pointMoveCardTargetValid_ = false;
    QVector<double> softwareZeroUnit_;
    QVector<bool> softwareZeroValid_;
    QSet<quint16> pendingJogAutoZeroAxes_;
    bool producerFinished_ = false;
    bool speedRatioPending_ = false;
    std::array<double, 2> lastQueuedTargetDegree_ {};
    bool lastQueuedTargetDegreeValid_ = false;
    int skippedDuplicatePointCount_ = 0;
    double firstEffectivePointTimeS_ = -1.0;
    QElapsedTimer contiRunElapsed_;
    QElapsedTimer completionStableClock_;
    qint64 lastContiDiagnosticMs_ = -1;
    int speedRatioNotReadyCount_ = 0;
    QMap<long, ContiPoint> pushedPointsByMark_;
    ContiPoint trajectoryStartPoint_;
    double lastPushedPlanTimeS_ = 0.0;
    double latestGeneratedPlanTimeS_ = 0.0;
    double currentPlanTimeS_ = 0.0;
    double expectedPlanTimeS_ = 0.0;
    double phaseErrorMs_ = 0.0;
    double bufferTimeMs_ = 0.0;
    double ratioRef_ = 0.0;
    double ratioPhase_ = 0.0;
    double ratioBuffer_ = 0.0;
    double ratioCommand_ = 1.0;
    double ratioApplied_ = 1.0;
    QElapsedTimer phaseClock_;
    bool phaseClockStarted_ = false;
    qint64 lastRatioControlMs_ = -1;
    qint64 lastRatioApiMs_ = -1;
    long lastRatioApiMark_ = -1;
    long nextMark_ = 1;
    long lastPushedMark_ = 0;
    int traceFramesRead_ = 0;
    quint64 latestTraceSequence_ = 0;
    quint64 latestTraceTimeUs_ = 0;
    TraceReadDiagnostics traceReadDiagnostics_;
    bool trajectoryComparisonActive_ = false;
    quint64 trajectoryTraceStartTimeUs_ = 0;
    bool traceVelocityAnchorValid_ = false;
    quint16 traceVelocityAxis_ = 0;
    quint64 traceVelocityAnchorTimeUs_ = 0;
    qint32 traceVelocityAnchorCommandPulse_ = 0;
    qint32 traceVelocityAnchorActualPulse_ = 0;
    bool traceVelocityValid_ = false;
    double traceCommandVelocityDegreePerSecond_ = 0.0;
    double traceActualVelocityDegreePerSecond_ = 0.0;
    bool vectorSpeedReadFailureLogged_ = false;
    QString traceStateText_ = QStringLiteral("Trace 未配置");
    QString stateText_ = QStringLiteral("未初始化");
};

#endif // MOTIONCONTROLWORKER_H
