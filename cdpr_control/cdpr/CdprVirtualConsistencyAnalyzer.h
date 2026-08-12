#ifndef CDPRVIRTUALCONSISTENCYANALYZER_H
#define CDPRVIRTUALCONSISTENCYANALYZER_H

#include <QString>

// 只读分析已落盘的CDPR运行记录。该类不会访问控制卡，也不会被控制周期调用。
struct CdprVirtualConsistencyAnalysisResult
{
    bool success = false;
    QString outputDirectory;
    QString summary;
    QString errorText;
    quint64 inputFrameCount = 0;
    quint64 eligibleFrameCount = 0;
    quint64 analyzedFrameCount = 0;
    quint64 engineeringResidualFrameCount = 0;
    quint64 strictResidualFrameCount = 0;
    quint64 rejectedFrameCount = 0;
    quint64 sequenceGapCount = 0;
    quint64 startTraceSequence = 0;
    double globalDelayMs = 0.0;
    double maximumSolvedCableResidualUm = 0.0;
    double maximumTranslationErrorMm = 0.0;
    double rmsTranslationErrorMm = 0.0;
    double maximumOrientationErrorDegree = 0.0;
};

class CdprVirtualConsistencyAnalyzer
{
public:
    // runDirectory必须包含metadata.json、configuration_snapshot.json、
    // run_context.json和trace_position.bin。期望轨迹可位于旧运行目录内，
    // 也可由run_context引用经过SHA-256校验的预生成缓存。
    static CdprVirtualConsistencyAnalysisResult analyze(
        const QString &runDirectory);
};

#endif // CDPRVIRTUALCONSISTENCYANALYZER_H
