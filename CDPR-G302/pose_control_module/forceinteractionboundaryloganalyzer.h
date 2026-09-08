#ifndef FORCEINTERACTIONBOUNDARYLOGANALYZER_H
#define FORCEINTERACTIONBOUNDARYLOGANALYZER_H

#include <QtGlobal>
#include <QString>
#include <QThread>

struct ForceInteractionBoundaryLogAnalysisResult
{
    bool passed = false;
    int sourceSchemaVersion = 0;
    quint64 dataRows = 0;
    quint64 replayedRows = 0;
    quint64 malformedRows = 0;
    quint64 mismatchRows = 0;
    quint64 actionMismatchRows = 0;
    quint64 pointMismatchRows = 0;
    quint64 traceInvalidRows = 0;
    quint64 traceNonMonotonicRows = 0;
    quint64 interactionRows = 0;
    quint64 brakingRows = 0;
    quint64 recorderAcceptedRows = 0;
    quint64 recorderWrittenRows = 0;
    quint64 recorderDroppedRows = 0;
    bool terminalSummaryPresent = false;
    bool terminalSummaryValid = false;
    int terminalState = -1;
    int terminalControlledStopCause = 0;
    bool terminalExperimentValid = false;
    quint64 terminalFinalStepCount = 0;
    quint64 terminalFinalCommandCount = 0;
    quint64 terminalMissedCycleCount = 0;
    double terminalElapsedS = 0.0;
    double terminalMinimumWorkspaceClearanceMm = 0.0;
    QString terminalReason;
    QString terminalSafetyReason;
    double maximumClearanceDifferenceMm = 0.0;
    double maximumTriggerDistanceDifferenceMm = 0.0;
    double maximumPointDifferenceMm = 0.0;
    QString csvPath;
    QString reportPath;
    QString errorMessage;
    QString summary;
};

// 会话结束后从CSV重新解析期望状态和冻结的安全配置，再独立调用统一边界模块。
// 本类不接触硬件，也不在控制周期内运行。
class ForceInteractionBoundaryLogAnalyzer
{
public:
    static ForceInteractionBoundaryLogAnalysisResult analyze(
            const QString& csvPath);
};

class ForceInteractionBoundaryLogAnalysisWorker final : public QThread
{
    Q_OBJECT

public:
    explicit ForceInteractionBoundaryLogAnalysisWorker(
            const QString& csvPath,
            QObject* parent = nullptr);

    ForceInteractionBoundaryLogAnalysisResult result() const;

protected:
    void run() override;

private:
    QString csvPath_;
    ForceInteractionBoundaryLogAnalysisResult result_;
};

#endif // FORCEINTERACTIONBOUNDARYLOGANALYZER_H
