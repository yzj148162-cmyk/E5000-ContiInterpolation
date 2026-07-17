#ifndef POSITIONSIMULATIONMODEL_H
#define POSITIONSIMULATIONMODEL_H

#include <QMutex>
#include <QThread>
#include <QVector>
#include <QVector3D>

#include <string>
#include <vector>

# pragma execution_character_set("utf-8")

class PositionSimulationModel : public QThread
{
    Q_OBJECT
public:
    PositionSimulationModel();
    PositionSimulationModel(double _ctrlCycleMs, double _frameL, double _frameW,
                   std::vector<double> _anchorMotorCof,
                   std::vector<double> _cableMotorCof,
                   std::vector<std::vector<std::vector<double>>> _endCableContactPos,
                   std::vector<std::vector<double>>* _anchorCableCoorHome,
                   std::vector<double> _anchorStepTimeMaxDis,
                   bool _useOfflineTraj,
                   double _pulleyRadius);
    ~PositionSimulationModel();

    void stopThread();

    std::vector<std::vector<double>> getCableMotorThetaTraj();
    std::vector<std::vector<double>> getCableLengthTraj();
    std::vector<double> getCableLengthTimeStamps();
    bool isTrajectorySimulationComplete() const;
    std::vector<std::vector<double>> getAnchorTimeStampTraj();
    std::vector<std::vector<double>> getEndAnchorCoor();
    double cableLengthCalculate(
        const std::vector<double>& a,   // 动平台连接点
        const std::vector<double>& b,   // 锚点
        double r                       // 滑轮半径
    );

    void setTraj(std::vector<std::vector<double>> p0, std::vector<std::vector<double>> v0, std::vector<std::vector<double>> a0,
                 std::vector<std::vector<double>> pt, std::vector<std::vector<double>> vt, std::vector<std::vector<double>> at,
                 double t, double dt);


    void setOfflineEndTraj(std::vector<std::vector<std::vector<std::vector<double>>>> _offlineTraj);

    // G302 固定锚点版本：保留旧接口供 MainWindow/力控辅助调用，内部不再计算锚点座运动。
    std::vector<std::vector<double>> anchorMoveDis2AnchorCableCoor(std::vector<double> anchorMoveDis);
    std::vector<double> gcHelper(std::vector<std::vector<double>> _curPose);

    // 兼容 MainWindow 现有赋值/开关；固定锚点版本中不产生动态锚点行为。
    std::string frameCalMethod = "fixed";
    void setStaticAnchor(bool enabled);

signals:
    void displayInfoSignal(std::string info, std::string type);
    void update3DViewerSignal(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                              QVector<QVector3D> cableEndPos);
    void poseCtrlStartSignal();
    void poseCtrlEndSignal();


private:
    void run() override;
    void poseCtrl();
    bool validateInput();
    int totalContactPointNum() const;
    double pointTime(int pointIndex) const;
    std::vector<double> rotateContactPoint(const std::vector<double>& localPoint, double rx, double ry, double rz) const;

    QMutex mLock;
    bool isRunning = false;

    double ctrlCycleMs = 0.0;
    double trajTime = 0.0;
    double trajStepTime = 0.0;
    bool useOfflineTraj = false;
    bool isStaticAnchor = true;

    int endNum = 0;
    int anchorNum = 0;
    double frameL = 0.0;
    double frameW = 0.0;

    std::vector<double> cableMotorCof;
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
    bool trajectorySimulationComplete = false;
    
};

#endif // POSITIONSIMULATIONMODEL_H
