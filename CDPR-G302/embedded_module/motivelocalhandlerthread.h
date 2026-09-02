/*
 * 文件总览：
 * - MotiveLocalHandlerThread 管理 Nokov 采集定时器，周期读取刚体/标记点并输出平台位姿。
 * - 同时支持姿态采集工作流：累积多帧有效结果求平均，用于初始位姿或标定确认。
 * - 类名保留历史 Motive 命名，当前实际使用 NokovMinimalClient。
 */

#ifndef MOTIVELOCALHANDLERTHREAD_H
#define MOTIVELOCALHANDLERTHREAD_H

#include <QObject>
#include <QTimer>
#include <QString>
#include <QtGlobal>

#include <vector>
#include <string>

#include "NokovMinimalClient.h"
#include "nokovposecalculator.h"

#pragma execution_character_set("utf-8")

class MotiveLocalHandlerThread : public QObject
{
    Q_OBJECT

public:
    static constexpr int DEFAULT_CAPTURE_SAMPLE_COUNT = 30;

    // 创建动捕处理对象，默认不连接 Nokov。
    MotiveLocalHandlerThread();
    // 创建并配置采样周期和 Nokov 服务 IP。
    MotiveLocalHandlerThread(double _ctrlCycleMs, QString nokovIP = "10.1.1.198");
    // 停止定时器并释放 Nokov 客户端。
    ~MotiveLocalHandlerThread();

    // 启动周期采样定时器。
    void startTimer();
    // 停止周期采样定时器。
    void stopTimer();

    double ctrlCycleMs = 0.0;

    // 返回最近一次有效刚体位姿矩阵。
    std::vector<std::vector<double>> getRigidPose();
    // 根据当前刚体位姿计算绳索起点坐标，保留给旧流程使用。
    std::vector<std::vector<double>> calCableStartPos();
    // 判断当前缓存中是否有有效刚体。
    bool hasCurrentRigidBody() const;
    // 判断最近 maxAgeMs 内是否收到有效刚体。
    bool hasRecentRigidBody(int maxAgeMs) const;
    // 判断姿态采集工作流是否已经得到有效结果。
    bool hasCapturedRigidBody() const;
    // 返回上一帧参与位姿计算的标记点数量。
    int lastMarkerCount() const;

    bool isInit = false;
    bool extraInfo = false;
    bool detailInfo = false;

public slots:
    // 开始多帧姿态采集，累计 sampleCount 帧后输出平均位姿。
    void beginPoseCapture(int sampleCount = DEFAULT_CAPTURE_SAMPLE_COUNT);

private:
    static constexpr int CAPTURE_TIMEOUT_MS = 5000;

    QTimer* timer = nullptr;
    bool isFirstLoop = true;
    bool isFirst = true;

    // 定时器入口：拉取 Nokov 数据并触发处理。
    void threadLoop();
    // 将当前帧标记点转换为刚体位姿，并处理采集状态机。
    void dataProcessor();

    // Nokov
    NokovMinimalClient* m_client = nullptr;
    bool m_isConnected = false;
    bool m_isPolling = false;
    qint64 m_lastPollTime = 0;
    int m_frameCount = 0;
    double m_averageInterval = 0.0;
    bool m_lastRigidBodyValid = false;
    int m_lastMarkerCount = 0;
    qint64 m_lastValidRigidBodyTimestampMs = -1;
    NokovPoseCalculator m_poseCalculator;
    bool m_captureActive = false;
    int m_captureSampleTarget = DEFAULT_CAPTURE_SAMPLE_COUNT;
    int m_captureSampleCount = 0;
    qint64 m_captureStartTimestampMs = -1;
    QVector<MarkerPoint> m_captureMarkerSums;

    std::vector<std::vector<double>> rigidPose;
    std::vector<std::vector<double>> tempRigidPose;

    // 从 Nokov 缓存中取出本项目刚体位姿计算所需的标记点。
    QVector<MarkerPoint> currentRigidMarkers();
    // 重置采集状态，可选择清空已捕获位姿。
    void resetCaptureState(bool clearPose);
    // 采集失败时统一发信号并记录原因。
    void failPoseCapture(const QString& reason);
    // 累加一帧采集结果，供 finishPoseCapture 求平均。
    void accumulatePoseSample(const NokovPoseCalculator::Result& poseResult);
    // 结束采集并输出平均刚体位姿。
    void finishPoseCapture();

signals:
    void displayInfoSignal(std::string info, std::string type);
    void dataUpdateSignal(std::vector<std::vector<double>> rigidPose);
    void poseCaptureCompleted(std::vector<std::vector<double>> rigidPose, int sampleCount);
    void poseCaptureFailed(std::string reason, int markerCount);
};

#endif // MOTIVELOCALHANDLERTHREAD_H
