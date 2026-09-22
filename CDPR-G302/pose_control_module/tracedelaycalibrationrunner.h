#ifndef TRACEDELAYCALIBRATIONRUNNER_H
#define TRACEDELAYCALIBRATIONRUNNER_H

#include "tracedelaycalibration.h"
#include "hardwareinterface.h"

#include <QMutex>
#include <future>
#include <vector>

class TraceDelayCalibrationRunner
{
public:
    explicit TraceDelayCalibrationRunner(HardwareInterface* hardware = nullptr);

    bool start(const TraceDelayCalibrationConfig& config,
               QString* errorMessage = nullptr);
    void tick(qint64 nowUs, int traceSamplePeriodUs);
    void stop(bool emergency, const QString& reason);
    bool isActive() const;
    TraceDelayCalibrationStatus status() const;
    std::array<TraceDelayAxisResult, 8> resultsForProfile(
            const QString& profileKey,
            double axisEquivalentPulsePerUnit,
            int traceSamplePeriodUs) const;
    bool recalculateLast(QString* errorMessage = nullptr);

private:
    struct FinalizationResult {
        TraceDelayFitResult fit;
        std::vector<TraceDelayCalibrationSegment> segments;
        QString rawDataFile;
        QString errorMessage;
        qint64 fitDurationUs = 0;
        qint64 csvWriteDurationUs = 0;
        qint64 persistenceDurationUs = 0;
    };

    bool configureCurrentAxis(QString* errorMessage);
    bool beginCurrentSegment(QString* errorMessage);
    void drainSamples();
    void completeCurrentAxis();
    void collectCurrentAxisFinalization();
    void finish(bool fault, const QString& message);
    bool writeRawCsv(QString* errorMessage = nullptr);
    void loadResults();
    void saveResults() const;
    QString persistencePath() const;

    HardwareInterface* hardware_ = nullptr;
    mutable QMutex mutex_;
    TraceDelayCalibrationConfig config_;
    TraceDelayCalibrationStatus status_;
    std::vector<int> axes_;
    std::array<double, 6> targets_{};
    std::vector<TraceDelayCalibrationSegment> segments_;
    std::vector<TraceDelayCalibrationSegment> lastSegments_;
    TraceDelayCalibrationConfig lastConfig_;
    int traceSamplePeriodUs_ = 0;
    int lastTraceSamplePeriodUs_ = 0;
    qint64 phaseStartUs_ = 0;
    qint64 axisStartUs_ = 0;
    std::array<double, 8> axisStartPosition_{};
    std::array<std::array<TraceDelayAxisResult, 8>, 2> storedResults_{};
    std::array<QString, 2> storedProfileKeys_{{QStringLiteral("g302_original"),
                                               QStringLiteral("generic_incremental_8_axis")}};
    std::array<double, 2> storedEquivalent_{{0.0, 0.0}};
    std::array<int, 2> storedTracePeriodUs_{{0, 0}};
    std::future<FinalizationResult> finalizationFuture_;
    bool finalizationPending_ = false;
    QString finalizationWarnings_;
};

#endif
