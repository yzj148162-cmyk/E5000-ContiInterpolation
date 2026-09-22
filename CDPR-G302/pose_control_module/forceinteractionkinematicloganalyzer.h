#ifndef FORCEINTERACTIONKINEMATICLOGANALYZER_H
#define FORCEINTERACTIONKINEMATICLOGANALYZER_H

#include "compensatedcablekinematics.h"
#include "physicalworkspaceboundary.h"

#include <array>
#include <vector>

#include <QString>
#include <QThread>

struct ForceInteractionKinematicLogAnalysisRequest
{
    QString csvPath;
    CompensatedCableKinematics::Configuration kinematics;
    std::array<double, 8> motorUnitPerRadian{};
    std::vector<double> referenceCableLengthMm;
    std::vector<double> initialPoseMmRad;
    PhysicalWorkspaceBoundaryConfig physicalWorkspace;
};

struct ForceInteractionKinematicLogAnalysisResult
{
    bool completed = false;
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
