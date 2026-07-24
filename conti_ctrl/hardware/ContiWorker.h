#ifndef CONTIWORKER_H
#define CONTIWORKER_H

#include <QQueue>
#include <QSet>
#include <QObject>
#include <QElapsedTimer>
#include <QMap>

#include "common/ContiTypes.h"
#include "control/PositionVelocityPid.h"
#include "control/TraceDelayCalibration.h"
#include "hardware/E5000HardwareInterface.h"
#include "planner/QuinticTrajectory.h"
#include "telemetry/TelemetryRecorder.h"

class QTimer;

// 上层规划、运行状态机与反馈处理线程；不直接调用 LTDMC。
// 所有 SDK 调用以及运行中的定时补段均由 E5000HardwareInterface 的独占线程执行。
class ContiWorker : public QObject
{
    Q_OBJECT

public:
    explicit ContiWorker(QObject *parent = nullptr);

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
    void startTraceDelayCalibration(const TraceDelayCalibrationConfig &config);
    void stopTraceDelayCalibration(bool emergency);
    void resetTraceDelayCalibrationAxis(quint16 axis);
    void startTelemetryRecording();
    void stopTelemetryRecording();
    void refreshBusCycle();
    void shutdownHardware();

signals:
    void logMessage(const QString &message);
    void statusChanged(const ContiStatus &status);
    void velocityPlotSamplesReady(const QVector<VelocityPlotSample> &samples);
    void traceDelayPlotSamplesReady(const QVector<TraceDelayPlotSample> &samples);

private slots:
    void produceNextPoint();
    void monitorContinuousRun();
    void runVelocityControlCycle();
    void runTraceDelayCalibrationCycle();

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
                                QString &errorMessage);
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
    bool validateTraceDelayCalibrationConfig(const TraceDelayCalibrationConfig &config,
                                             QString &errorMessage) const;
    bool startNextTraceDelaySegment(QString &errorMessage);
    void analyzeTraceDelayCalibration();
    void finishTraceDelayCalibration(const QString &message,
                                     bool failed = false,
                                     bool emergency = false);
    void appendTraceDelayCalibrationFrames(const QVector<TraceTelemetryFrame> &frames);
    void flushTraceDelayPlotSamples();
    void resetTraceDelayHistory();
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
    // E5000HardwareInterface 的独占硬件线程内。
    E5000HardwareInterface card_;
    TelemetryRecorder telemetryRecorder_;
    QuinticTrajectory trajectory_;
    QQueue<ContiPoint> hostQueue_;
    QSet<quint16> enabledAxes_;
    QVector<quint16> detectedAxes_;
    QVector<quint16> traceAxes_;
    QVector<AxisFeedback> latestAxisFeedback_;
    QTimer *producerTimer_ = nullptr;
    QTimer *monitorTimer_ = nullptr;
    QTimer *feedbackTimer_ = nullptr;
    QTimer *velocityControlTimer_ = nullptr;
    QTimer *traceDelayCalibrationTimer_ = nullptr;
    ContiTestConfig config_;
    ContiFeedStatus lastFeedStatus_;
    bool boardInitialized_ = false;
    short detectedBoardCount_ = 0;
    quint16 initializedCardNo_ = 0;
    int actualBusCycleUs_ = 0;
    bool listOpen_ = false;
    bool preparing_ = false;
    bool running_ = false;
    bool pointMoveActive_ = false;
    bool velocityControlActive_ = false;
    bool velocityMotionStarted_ = false;
    bool velocityReferenceInitialized_ = false;
    bool velocityAutoRecording_ = false;
    bool traceDelayCalibrationActive_ = false;
    bool traceDelayMotionStarted_ = false;
    bool traceDelayAutoRecording_ = false;
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
    QElapsedTimer velocityCompletionClock_;
    quint64 velocityLastTraceSequence_ = 0;
    qint64 velocityLastDiagnosticMs_ = -1;
    QVector<VelocityPlotSample> pendingVelocityPlotSamples_;
    QElapsedTimer velocityPlotPublishClock_;
    quint64 velocityPlotTraceStartTimeUs_ = 0;
    bool velocityPlotTraceStartValid_ = false;
    TraceDelayCalibrationConfig traceDelayConfig_;
    TraceDelayCalibrationStatus traceDelayStatus_;
    QVector<TraceDelayAxisResult> traceDelayAxisResults_;
    QVector<double> traceDelaySegmentTargets_;
    QVector<TraceDelaySegmentCapture> traceDelaySegments_;
    QVector<TraceTelemetryFrame> traceDelayCurrentSegmentFrames_;
    TraceDelayPhase traceDelayPhase_ = TraceDelayPhase::Idle;
    int traceDelayCurrentSegmentIndex_ = 0;
    quint64 traceDelayRunId_ = 0;
    QElapsedTimer traceDelayPhaseClock_;
    QElapsedTimer traceDelayPlotPublishClock_;
    quint64 traceDelayPlotStartTimeUs_ = 0;
    QVector<TraceDelayPlotSample> pendingTraceDelayPlotSamples_;
    std::array<QQueue<TraceCommandHistorySample>, 8> traceCommandHistory_;
    quint64 lastTraceDelaySequence_ = 0;
    int savedCalibrationBusCycleUs_ = 0;
    int savedCalibrationTracePeriodUs_ = 0;
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

#endif // CONTIWORKER_H
