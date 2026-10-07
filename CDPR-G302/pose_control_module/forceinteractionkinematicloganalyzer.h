#ifndef FORCEINTERACTIONKINEMATICLOGANALYZER_H
#define FORCEINTERACTIONKINEMATICLOGANALYZER_H

#include "compensatedcablekinematics.h"
#include "physicalworkspaceboundary.h"
#include "forceinteractiontensionshadow.h"

#include <array>
#include <limits>
#include <vector>

#include <QString>
#include <QThread>

struct ForceInteractionKinematicLogAnalysisRequest
{
    QString csvPath;
    CompensatedCableKinematics::Configuration kinematics;
    std::array<double, 8> motorUnitPerRadian{};
    // 运行记录中的 safety-relative Trace 位置以整机安全零点为基准；
    // 正运动学所需的是相对本次六维力会话起点的电机位移。
    std::array<double, 8> actualStartSafetyRelativePosition{};
    std::vector<double> referenceCableLengthMm;
    std::vector<double> initialPoseMmRad;
    PhysicalWorkspaceBoundaryConfig physicalWorkspace;
    // 2026-10-05: M1 uses the same frozen run context and never reads live UI.
    ForceInteractionTensionShadowConfig tensionShadow;
    bool tensionShadowEnabled = false;
};

struct ForceInteractionKinematicLogAnalysisResult
{
    bool completed = false;
    bool staticTensionMode = false;
    quint64 dataRows = 0;
    quint64 validTraceRows = 0;
    quint64 solvedRows = 0;
    quint64 failedRows = 0;
    quint64 nonMonotonicTraceRows = 0;
    double translationRmsMm = 0.0;
    double translationMaximumMm = 0.0;
    double orientationRmsDeg = 0.0;
    double orientationMaximumDeg = 0.0;
    double cableResidualRmsMm = 0.0;
    double cableResidualMaximumMm = 0.0;
    quint64 shadowEvaluatedRows = 0;
    quint64 shadowValidRows = 0;
    quint64 shadowInfeasibleRows = 0;
    qint64 shadowMaximumCalculationUs = 0;
    double shadowMinimumTensionMarginN =
            std::numeric_limits<double>::infinity();
    double shadowMaximumWrenchResidual = 0.0;
    quint64 staticTensionRows = 0;
    quint64 staticTensionCommittedRows = 0;
    quint64 staticTensionBatchFailureRows = 0;
    quint64 staticTensionErrorMismatchRows = 0;
    quint64 staticTensionQuantizationViolationRows = 0;
    quint64 staticTensionSlewViolationRows = 0;
    double staticTensionMaximumAbsErrorN = 0.0;
    double staticTensionMaximumQuantumResidualNm = 0.0;
    double staticTensionMaximumSlewExcessNm = 0.0;
    qint64 traceHostAnchorOffsetUs = 0;
    QString csvPath;
    QString resultCsvPath;
    QString errorMessage;
    QString summary;
};

class ForceInteractionKinematicLogAnalyzer
{
public:
    static ForceInteractionKinematicLogAnalysisResult analyze(
            const ForceInteractionKinematicLogAnalysisRequest& request);
};

class ForceInteractionKinematicLogAnalysisWorker final : public QThread
{
    Q_OBJECT
public:
    explicit ForceInteractionKinematicLogAnalysisWorker(
            const ForceInteractionKinematicLogAnalysisRequest& request,
            QObject* parent = nullptr);
    ForceInteractionKinematicLogAnalysisResult result() const;

protected:
    void run() override;

private:
    ForceInteractionKinematicLogAnalysisRequest request_;
    ForceInteractionKinematicLogAnalysisResult result_;
};

#endif // FORCEINTERACTIONKINEMATICLOGANALYZER_H
