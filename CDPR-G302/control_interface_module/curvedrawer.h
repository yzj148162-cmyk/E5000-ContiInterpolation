/*
 * 文件总览：
 * - CurveDrawer 封装 QCustomPlot 曲线初始化和数据刷新，负责电机、末端、绳力、绳速和绳长等曲线显示。
 * - 上层只发 update 信号，本类负责通道命名、坐标轴缩放和定时刷新。
 */

#ifndef CURVEDRAWER_H
#define CURVEDRAWER_H

#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVector>

#include "qcustomplot.h"

class QWidget;

class CurveDrawer : public QObject
{
    Q_OBJECT

public:
    // 创建曲线绘制器并初始化刷新定时器、信号槽连接。
    explicit CurveDrawer(QObject *parent = nullptr);
    ~CurveDrawer() override = default;

    // 设置电机控制曲线和绳索类曲线的通道数量，通常在轴数变化后调用。
    void setChannelCounts(int motorControlChannelCount, int cableChannelCount);
    // 开启或暂停定时重绘，避免未运行状态下重复刷新 UI。
    void setRefreshEnabled(bool enabled);
    void setMotorControlHybridMode(bool enabled, const QVector<int>& forceGraphIndexes);
    // 绑定各个 QCustomPlot 控件并完成坐标轴、图例和曲线通道初始化。
    void initPlot(QCustomPlot *motorControlPlotWidget,
                  QCustomPlot *endEffectorPosPlotWidget,
                  QCustomPlot *endEffectorVelPlotWidget,
                  QCustomPlot *endEffectorAccPlotWidget,
                  QCustomPlot *cableTensionPlotWidget,
                  QCustomPlot *cableSpeedPlotWidget,
                  QCustomPlot *cableLengthPlotWidget,
                  QCustomPlot *actualEndEffectorPosPlotWidget = nullptr,
                  QCustomPlot *actualEndEffectorVelPlotWidget = nullptr,
                  QCustomPlot *actualEndEffectorAccPlotWidget = nullptr);

signals:
    void updateMotorControlPlotSignal(double time, QVector<double> values);
    void updateEndEffectorPosPlotSignal(double time, QVector<double> position, QVector<double> orientation);
    void updateEndEffectorVelPlotSignal(double time, QVector<double> velocity, QVector<double> angularVelocity);
    void updateEndEffectorAccPlotSignal(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    void updateActualEndEffectorPosPlotSignal(double time, QVector<double> position, QVector<double> orientation);
    void updateActualEndEffectorVelPlotSignal(double time, QVector<double> velocity, QVector<double> angularVelocity);
    void updateActualEndEffectorAccPlotSignal(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    void updateCableTensionPlotSignal(double time, QVector<double> tensions);
    void updateCableSpeedPlotSignal(double time, QVector<double> speeds);
    void updateCableLengthPlotSignal(double time, QVector<double> lengths);
    void clearAllDataSignal();
    void plotRefreshTick(int intervalMs);

private slots:
    // 接收电机控制量采样并追加到对应曲线。
    void handleUpdateMotorControlPlot(double time, QVector<double> values);
    // 接收末端位置/姿态采样并追加到位姿曲线。
    void handleUpdateEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation);
    // 接收末端速度/角速度采样并追加到速度曲线。
    void handleUpdateEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity);
    // 接收末端加速度/角加速度采样并追加到加速度曲线。
    void handleUpdateEndEffectorAccPlot(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    void handleUpdateActualEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation);
    void handleUpdateActualEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity);
    void handleUpdateActualEndEffectorAccPlot(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    // 接收绳索张力采样并追加到张力曲线。
    void handleUpdateCableTensionPlot(double time, QVector<double> tensions);
    // 接收绳索速度采样并追加到速度曲线。
    void handleUpdateCableSpeedPlot(double time, QVector<double> speeds);
    // 接收绳长采样并追加到绳长曲线。
    void handleUpdateCableLengthPlot(double time, QVector<double> lengths);
    // 清空所有曲线数据，通常在新会话或重新仿真前调用。
    void handleClearAllData();

private:
    struct PlotRefreshState {
        bool dirty = false;
        bool forceRescale = true;
        qint64 lastRescaleMs = 0;
        qint64 lastTrimMs = 0;
    };

    void handlePlotTimerTick();
    PlotRefreshState* refreshStateForPlot(QCustomPlot *plot);
    void refreshDirtyPlot(QCustomPlot *plot, PlotRefreshState& state);
    void updatePlotAfterAppend(QCustomPlot *plot, double time, double minSample, double maxSample);
    void updateDualAxisPlotAfterAppend(QCustomPlot *plot,
                                       double time,
                                       double leftMinSample,
                                       double leftMaxSample,
                                       double rightMinSample,
                                       double rightMaxSample);
    void trimPlotDataIfDue(QCustomPlot *plot, PlotRefreshState& state, double time, qint64 nowMs);
    void maybeRescalePlotYAxis(QCustomPlot *plot,
                               PlotRefreshState& state,
                               double minSample,
                               double maxSample,
                               qint64 nowMs);
    void maybeRescaleDualYAxis(QCustomPlot *plot,
                               PlotRefreshState& state,
                               double leftMinSample,
                               double leftMaxSample,
                               double rightMinSample,
                               double rightMaxSample,
                               qint64 nowMs);
    bool isPlotVisibleForRefresh(QCustomPlot *plot) const;
    bool sampleOutsideAxisRange(QCPAxis *axis, double minSample, double maxSample) const;
    bool isDualYAxisPosePlot(QCustomPlot *plot) const;
    bool isMotorControlForceGraph(int graphIndex) const;

    // 建立本对象内部信号到槽的连接，并启动刷新定时器。
    void initConnections();
    void applyMotorControlPlotMode(bool clearData);
    // 按通道数量初始化单个图表的曲线、图例和坐标轴标签。
    void setupPlot(QCustomPlot *plot,
                   int graphCount,
                   const QString &yLabel,
                   const QStringList &graphNames,
                   bool dualYAxis = false,
                   const QString &rightYLabel = QString());
    void rescaleAxisForGraphRange(QCustomPlot *plot, QCPAxis *axis, int firstGraph, int graphCount) const;
    void rescaleAxisForMotorControlHybridGraphs(QCPAxis *axis, bool forceGraphs) const;
    // 根据当前可见数据自适应 Y 轴范围。
    void rescalePlotYAxis(QCustomPlot *plot) const;

    // 生成“电机1/绳索1”这类带编号的曲线名称列表。
    static QStringList buildAxisNames(const QString &prefix, int count, int startIndex = 1);
    // 生成末端位姿六自由度曲线名称。
    static QStringList buildPoseAxisNames();

    QCustomPlot *motorControlPlot = nullptr;
    QCustomPlot *endEffectorPosPlot = nullptr;
    QCustomPlot *endEffectorVelPlot = nullptr;
    QCustomPlot *endEffectorAccPlot = nullptr;
    QCustomPlot *actualEndEffectorPosPlot = nullptr;
    QCustomPlot *actualEndEffectorVelPlot = nullptr;
    QCustomPlot *actualEndEffectorAccPlot = nullptr;
    QCustomPlot *cableTensionPlot = nullptr;
    QCustomPlot *cableSpeedPlot = nullptr;
    QCustomPlot *cableLengthPlot = nullptr;

    PlotRefreshState motorControlPlotState;
    PlotRefreshState endEffectorPosPlotState;
    PlotRefreshState endEffectorVelPlotState;
    PlotRefreshState endEffectorAccPlotState;
    PlotRefreshState actualEndEffectorPosPlotState;
    PlotRefreshState actualEndEffectorVelPlotState;
    PlotRefreshState actualEndEffectorAccPlotState;
    PlotRefreshState cableTensionPlotState;
    PlotRefreshState cableSpeedPlotState;
    PlotRefreshState cableLengthPlotState;

    QTimer *plotTimer = nullptr;
    int motorControlChannelCount = 8;
    int cableChannelCount = 8;
    double timeWindow = 10.0;
    bool motorControlHybridMode = false;
    QVector<int> motorControlHybridForceGraphIndexes;
};

#endif // CURVEDRAWER_H
