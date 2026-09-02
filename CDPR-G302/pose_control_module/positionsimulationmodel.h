/*
 * 文件总览：
 * - PositionSimulationModel 负责把末端平台位姿轨迹转换为各根绳长和电机角度轨迹，是位置/PVT 执行前的离线仿真环节。
 * - 支持在线生成五次轨迹，也支持读取外部离线轨迹；固定锚点的 G302 版本保留旧接口但不再计算锚点座运动。
 * - 绞盘螺旋卷绕补偿和绳索弹性补偿在这里合入，最终输出给 MainWindow/PvtExecutionWorker。
 */

#ifndef POSITIONSIMULATIONMODEL_H
#define POSITIONSIMULATIONMODEL_H

#include <QMutex>
#include <QString>
#include <QThread>
#include <QVector>
#include <QVector3D>

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "winchcompensation.h"

# pragma execution_character_set("utf-8")

class PositionSimulationModel : public QThread
{
    Q_OBJECT
public:
    struct TrajectoryPointTimingSample {
        int pointIndex = 0;
        double trajectoryTimeSec = 0.0;
        qint64 cableLengthCalculationUs = 0;
    };

    // 创建空仿真模型，参数稍后由 setTraj 或完整构造函数配置。
    PositionSimulationModel();
    // 创建完整仿真模型，绑定结构尺寸、绳索几何、轨迹来源和补偿参数。
    PositionSimulationModel(double _ctrlCycleMs, double _frameL, double _frameW,
                   std::vector<double> _anchorMotorCof,
                   std::vector<double> _cableMotorCof,
                   std::vector<std::vector<std::vector<double>>> _endCableContactPos,
                   std::vector<std::vector<double>>* _anchorCableCoorHome,
                   std::vector<double> _anchorStepTimeMaxDis,
                   bool _useOfflineTraj,
                   double _pulleyRadius,
                   std::vector<WinchCompensation::AxisConfig> _winchCompensationConfig = {},
                   std::vector<std::vector<double>> _winchReferencePose = {},
                   std::vector<std::vector<double>> _cableForceTraj = {},
                   bool _applyRopeElasticCompensation = false,
                   std::vector<double> _ropeElasticFixedLengthL0Mm = {});
    // 停止线程并释放仿真模型资源。
    ~PositionSimulationModel();

    // 请求仿真线程退出。
    void stopThread();

    // 返回仿真得到的电机角度轨迹。
    std::vector<std::vector<double>> getCableMotorThetaTraj();
    // 返回仿真得到的绳长轨迹。
    std::vector<std::vector<double>> getCableLengthTraj();
    // 返回绳长/电机轨迹对应的时间戳。
    std::vector<double> getCableLengthTimeStamps();
    // 判断当前轨迹仿真是否已经完整结束。
    std::vector<TrajectoryPointTimingSample> getTrajectoryPointTimingSamples() const;
    bool isTrajectorySimulationComplete() const;
    // 返回锚点时间戳轨迹；固定锚点版本主要用于兼容旧接口。
    std::vector<std::vector<double>> getAnchorTimeStampTraj();
    // 返回末端锚点坐标轨迹。
    std::vector<std::vector<double>> getEndAnchorCoor();
    // 计算单根绳索从动平台连接点到锚点的长度，支持滑轮半径修正。
    double cableLengthCalculate(
        const std::vector<double>& a,   // 动平台连接点
        const std::vector<double>& b,   // 锚点
        double r                       // 滑轮半径
    ) const;

    // 设置在线规划轨迹的起止位姿、速度、加速度和采样周期。
    void setTraj(std::vector<std::vector<double>> p0, std::vector<std::vector<double>> v0, std::vector<std::vector<double>> a0,
                 std::vector<std::vector<double>> pt, std::vector<std::vector<double>> vt, std::vector<std::vector<double>> at,
                 double t, double dt);


    // 设置外部文件或上层规划生成的离线末端轨迹。
    void setOfflineEndTraj(std::vector<std::vector<std::vector<std::vector<double>>>> _offlineTraj);
    // 控制仿真是否按真实时间回放，关闭后可尽快跑完离线计算。
    void setRealtimePlaybackEnabled(bool enabled);
    // 请求跳过3D动画展示；剩余轨迹仍会快速计算完成，用于后续PVT下发。
    void requestSkipVisualizationPlayback();
    void setSimulationPathSignature(const QString& signature,
                                    int segmentIndex = 0,
                                    int segmentCount = 1);
    QString simulationPathSignature() const;
    int simulationPathSegmentIndex() const;
    int simulationPathSegmentCount() const;

    // G302 固定锚点版本：保留旧接口供 MainWindow/力控辅助调用，内部不再计算锚点座运动。
    // 兼容旧接口：由锚点位移计算锚点坐标；固定锚点版本直接返回基准坐标。
    std::vector<std::vector<double>> anchorMoveDis2AnchorCableCoor(std::vector<double> anchorMoveDis);
    // 兼容旧流程的几何计算辅助函数，返回当前姿态下的绳长相关结果。
    std::vector<double> gcHelper(std::vector<std::vector<double>> _curPose);

    // 兼容 MainWindow 现有赋值/开关；固定锚点版本中不产生动态锚点行为。
    std::string frameCalMethod = "fixed";
    // 设置是否使用固定锚点模型；G302 当前版本通常保持启用。
    void setStaticAnchor(bool enabled);

signals:
    void displayInfoSignal(std::string info, std::string type);
    void update3DViewerSignal(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                              QVector<QVector3D> cableEndPos);
    void poseCtrlStartSignal();
    void poseCtrlEndSignal();
    void trajectorySimulationFinished(QString pathSignature,
                                      int segmentIndex,
                                      int segmentCount,
                                      bool complete);


private:
    // QThread 入口：校验输入后执行 poseCtrl。
    void run() override;
    // 主仿真循环：逐轨迹点计算平台位姿、绳长和电机角度。
    void poseCtrl();
    // 校验轨迹、几何参数和补偿配置是否足以开始仿真。
    bool validateInput();
    // 统计所有末端上的接触点总数。
    int totalContactPointNum() const;
    // 返回指定轨迹点的时间戳。
    double pointTime(int pointIndex) const;
    // 将局部接触点按当前欧拉角旋转到平台坐标。
    std::vector<double> rotateContactPoint(const std::vector<double>& localPoint, double rx, double ry, double rz) const;
    // SO(3) 轨迹的主路径：直接使用平台到全局的旋转矩阵，避免从显示欧拉角重建姿态。
    std::vector<double> rotateContactPoint(const std::vector<double>& localPoint,
                                           const Eigen::Matrix3d& rotationGlobalFromBody) const;
    // 从在线或离线轨迹中取出某一采样点的各末端位姿。
    std::vector<std::vector<double>> trajectoryPoseAtPoint(int pointIndex) const;
    // 规范化卷绕补偿参考位姿，保证轴数量和末端数量一致。
    std::vector<std::vector<double>> normalizedWinchReferencePose() const;
    // 计算给定位姿矩阵对应的全部绳长，供仿真和正运动学辅助复用。
    std::vector<double> cableLengthsForPoseMatrix(const std::vector<std::vector<double>>& poseByEnd) const;

    QMutex mLock;
    bool isRunning = false;

    double ctrlCycleMs = 0.0;
    double trajTime = 0.0;
    double trajStepTime = 0.0;
    bool useOfflineTraj = false;
    bool isStaticAnchor = true;
    bool realtimePlaybackEnabled = true;
    bool visualizationPlaybackSkipRequested = false;

    int endNum = 0;
    int anchorNum = 0;
    double frameL = 0.0;
    double frameW = 0.0;

    std::vector<double> cableMotorCof;
    std::vector<WinchCompensation::AxisConfig> winchCompensationConfig;
    std::vector<std::vector<double>> winchReferencePose;
    std::vector<std::vector<double>> cableForceTraj;
    bool applyRopeElasticCompensation = false;
    std::vector<double> ropeElasticFixedLengthL0Mm;
    std::vector<std::vector<std::vector<double>>> endCableContactPos;
    std::vector<std::vector<double>>* anchorCableCoorHome = nullptr;
    double pulleyRadius = 0.0;

    std::vector<std::vector<double>> trajStartPose;
    std::vector<std::vector<double>> trajStartVel;
    std::vector<std::vector<double>> trajStartAcc;
    std::vector<std::vector<double>> trajEndPose;
    std::vector<std::vector<double>> trajEndVel;
    std::vector<std::vector<double>> trajEndAcc;

    // 第一层：末端；第二层：位置/速度/加速度/时间；第三层：变量；第四层：轨迹点。
    std::vector<std::vector<std::vector<std::vector<double>>>> traj;
    std::vector<std::vector<std::vector<std::vector<double>>>> offlineTraj;

    std::vector<std::vector<double>> anchorTimeStampTraj;
    std::vector<std::vector<std::vector<double>>> trajAnchorCoorVec;
    std::vector<std::vector<double>> trajCableLenVec;
    std::vector<std::vector<double>> trajCableMotorThetaVec;
    std::vector<TrajectoryPointTimingSample> trajectoryPointTimingSamples;
    bool trajectorySimulationComplete = false;
    QString simulationPathSignatureText;
    int simulationPathSegmentIndexValue = 0;
    int simulationPathSegmentCountValue = 1;
    
};

#endif // POSITIONSIMULATIONMODEL_H
