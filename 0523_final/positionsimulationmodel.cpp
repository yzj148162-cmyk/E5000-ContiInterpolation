#include "positionsimulationmodel.h"
#include "MatrixFun.h"
#include "trajectoryplanner.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

bool isSixDimTrajPara(const std::vector<std::vector<double>>& data)
{
    return !data.empty() && data[0].size() == 6;
}

} // namespace

PositionSimulationModel::PositionSimulationModel()
{
}

PositionSimulationModel::PositionSimulationModel(double _ctrlCycleMs, double _frameL, double _frameW,
                               std::vector<double> _anchorMotorCof,
                               std::vector<double> _cableMotorCof,
                               std::vector<std::vector<std::vector<double>>> _endCableContactPos,
                               std::vector<std::vector<double>>* _anchorCableCoorHome,
                               std::vector<double> _anchorStepTimeMaxDis,
                               bool _useOfflineTraj,
                               double _pulleyRadius)
    : ctrlCycleMs(_ctrlCycleMs),
      useOfflineTraj(_useOfflineTraj),
      endNum(static_cast<int>(_endCableContactPos.size())),
      anchorNum(_anchorCableCoorHome ? static_cast<int>(_anchorCableCoorHome->size()) : 0),
      frameL(_frameL),
      frameW(_frameW),
      cableMotorCof(std::move(_cableMotorCof)),
      endCableContactPos(std::move(_endCableContactPos)),
      anchorCableCoorHome(_anchorCableCoorHome),
      pulleyRadius(_pulleyRadius)
{
    (void)_anchorMotorCof;
    (void)_anchorStepTimeMaxDis;
}

PositionSimulationModel::~PositionSimulationModel()
{
    stopThread();
    if (QThread::isRunning() && !wait(500)) {
        terminate();
        wait(500);
    }
}

void PositionSimulationModel::stopThread()
{
    QMutexLocker locker(&mLock);
    isRunning = false;
}

void PositionSimulationModel::run()
{
    {
        QMutexLocker locker(&mLock);
        isRunning = true;
    }

    qDebug() << "Pose ctrl thread created.";
    emit poseCtrlStartSignal();
    poseCtrl();
    qDebug() << "============POSE CTRL COMPLETE============";
    emit poseCtrlEndSignal();

    {
        QMutexLocker locker(&mLock);
        isRunning = false;
    }
}

std::vector<std::vector<double>> PositionSimulationModel::getCableMotorThetaTraj()
{
    return trajCableMotorThetaVec;
}

std::vector<std::vector<double>> PositionSimulationModel::getCableLengthTraj()
{
    return trajCableLenVec;
}

std::vector<double> PositionSimulationModel::getCableLengthTimeStamps()
{
    if (anchorTimeStampTraj.empty()) {
        return {};
    }
    return anchorTimeStampTraj.front();
}

bool PositionSimulationModel::isTrajectorySimulationComplete() const
{
    return trajectorySimulationComplete;
}

std::vector<std::vector<double>> PositionSimulationModel::getAnchorTimeStampTraj()
{
    return anchorTimeStampTraj;
}

std::vector<std::vector<double>> PositionSimulationModel::getEndAnchorCoor()
{
    if (trajAnchorCoorVec.empty()) {
        return {};
    }
    return trajAnchorCoorVec.back();
}

void PositionSimulationModel::setOfflineEndTraj(std::vector<std::vector<std::vector<std::vector<double>>>> _offlineTraj)
{
    offlineTraj = std::move(_offlineTraj);
}

void PositionSimulationModel::setTraj(std::vector<std::vector<double>> p0, std::vector<std::vector<double>> v0, std::vector<std::vector<double>> a0,
                             std::vector<std::vector<double>> pt, std::vector<std::vector<double>> vt, std::vector<std::vector<double>> at,
                             double t, double dt)
{
    if (!isSixDimTrajPara(p0) || !isSixDimTrajPara(v0) || !isSixDimTrajPara(a0) ||
        !isSixDimTrajPara(pt) || !isSixDimTrajPara(vt) || !isSixDimTrajPara(at)) {
        emit displayInfoSignal("错误：位置模式，输入的轨迹参数都应当包含位置和姿态", "error");
        return;
    }

    trajStartPose = std::move(p0);
    trajStartVel = std::move(v0);
    trajStartAcc = std::move(a0);
    trajEndPose = std::move(pt);
    trajEndVel = std::move(vt);
    trajEndAcc = std::move(at);
    trajTime = t;
    trajStepTime = dt;
}

std::vector<std::vector<double>> PositionSimulationModel::anchorMoveDis2AnchorCableCoor(std::vector<double> anchorMoveDis)
{
    (void)anchorMoveDis;
    if (!anchorCableCoorHome) {
        return {};
    }
    return *anchorCableCoorHome;
}

std::vector<double> PositionSimulationModel::gcHelper(std::vector<std::vector<double>> _curPose)
{
    (void)_curPose;
    return std::vector<double>(anchorNum, 0.0);
}

void PositionSimulationModel::setStaticAnchor(bool enabled)
{
    isStaticAnchor = enabled;
}

bool PositionSimulationModel::validateInput()
{
    if (!anchorCableCoorHome || anchorCableCoorHome->empty()) {
        emit displayInfoSignal("错误：位置模式，固定锚点坐标为空", "error");
        return false;
    }
    for (const auto& anchor : *anchorCableCoorHome) {
        if (anchor.size() != 3) {
            emit displayInfoSignal("错误：位置模式，固定锚点坐标应当包含XYZ三个元素", "error");
            return false;
        }
    }

    if (endCableContactPos.empty()) {
        emit displayInfoSignal("错误：位置模式，末端绳索接点为空", "error");
        return false;
    }
    for (const auto& endContacts : endCableContactPos) {
        if (endContacts.empty()) {
            emit displayInfoSignal("错误：位置模式，末端绳索接点为空", "error");
            return false;
        }
        for (const auto& contact : endContacts) {
            if (contact.size() != 3) {
                emit displayInfoSignal("错误：位置模式，末端绳索接点位置坐标应当包含XYZ三个元素", "error");
                return false;
            }
        }
    }

    if (totalContactPointNum() != static_cast<int>(anchorCableCoorHome->size())) {
        emit displayInfoSignal("错误：位置模式，固定锚点数量与末端绳索接点数量不一致", "error");
        return false;
    }

    if (cableMotorCof.size() != anchorCableCoorHome->size()) {
        emit displayInfoSignal("错误：位置模式，电机换算系数数量与固定锚点数量不一致", "error");
        return false;
    }

    if (!useOfflineTraj && (std::abs(trajTime) < 1e-9 || std::abs(trajStepTime) < 1e-9)) {
        emit displayInfoSignal("错误：位置模式，请先输入轨迹运行时间及其步长", "error");
        return false;
    }

    return true;
}

int PositionSimulationModel::totalContactPointNum() const
{
    int count = 0;
    for (const auto& endContacts : endCableContactPos) {
        count += static_cast<int>(endContacts.size());
    }
    return count;
}

double PositionSimulationModel::pointTime(int pointIndex) const
{
    if (!traj.empty() && traj[0].size() > 3 && !traj[0][3].empty() &&
        pointIndex >= 0 && pointIndex < static_cast<int>(traj[0][3][0].size())) {
        return traj[0][3][0][pointIndex];
    }
    return pointIndex * trajStepTime;
}

std::vector<double> PositionSimulationModel::rotateContactPoint(const std::vector<double>& localPoint, double rx, double ry, double rz) const
{
    const double cx = std::cos(rx);
    const double sx = std::sin(rx);
    const double cy = std::cos(ry);
    const double sy = std::sin(ry);
    const double cz = std::cos(rz);
    const double sz = std::sin(rz);

    const double x1 = localPoint[0];
    const double y1 = cx * localPoint[1] - sx * localPoint[2];
    const double z1 = sx * localPoint[1] + cx * localPoint[2];

    const double x2 = cy * x1 + sy * z1;
    const double y2 = y1;
    const double z2 = -sy * x1 + cy * z1;

    return {
        cz * x2 - sz * y2,
        sz * x2 + cz * y2,
        z2
    };
}

void PositionSimulationModel::poseCtrl()
{
    trajectorySimulationComplete = false;

    if (!validateInput()) {
        return;
    }

    endNum = static_cast<int>(endCableContactPos.size());
    anchorNum = static_cast<int>(anchorCableCoorHome->size());

    traj.clear();
    traj.resize(endNum);
    if (useOfflineTraj) {
        if (offlineTraj.size() < static_cast<size_t>(endNum)) {
            emit displayInfoSignal("错误：位置模式，离线轨迹末端数量不足", "error");
            return;
        }
        for (int endIndex = 0; endIndex < endNum; ++endIndex) {
            if (offlineTraj[endIndex].empty() || offlineTraj[endIndex][0].size() < 6) {
                emit displayInfoSignal("错误：位置模式，离线轨迹应至少包含6维位姿", "error");
                return;
            }
            traj[endIndex] = offlineTraj[endIndex];
            if (traj[endIndex].size() < 4) {
                traj[endIndex].resize(4);
            }
        }
    } else {
        if (trajStartPose.size() < static_cast<size_t>(endNum) ||
            trajStartVel.size() < static_cast<size_t>(endNum) ||
            trajStartAcc.size() < static_cast<size_t>(endNum) ||
            trajEndPose.size() < static_cast<size_t>(endNum) ||
            trajEndVel.size() < static_cast<size_t>(endNum) ||
            trajEndAcc.size() < static_cast<size_t>(endNum)) {
            emit displayInfoSignal("错误：位置模式，轨迹输入末端数量不足", "error");
            return;
        }

        for (int endIndex = 0; endIndex < endNum; ++endIndex) {
            traj[endIndex] = TrajectoryPlanner::quintic(trajStartPose[endIndex], trajStartVel[endIndex], trajStartAcc[endIndex],
                                                        trajEndPose[endIndex], trajEndVel[endIndex], trajEndAcc[endIndex],
                                                        trajTime, trajStepTime);
            if (traj[endIndex].empty()) {
                emit displayInfoSignal("错误：位置模式，轨迹插值失败", "error");
                return;
            }
        }
    }

    if (traj.empty() || traj[0].empty() || traj[0][0].size() < 6 || traj[0][0][0].empty()) {
        emit displayInfoSignal("错误：位置模式，轨迹为空", "error");
        return;
    }

    const int trajPointNum = static_cast<int>(traj[0][0][0].size());
    for (int endIndex = 0; endIndex < endNum; ++endIndex) {
        if (traj[endIndex].empty() || traj[endIndex][0].size() < 6) {
            emit displayInfoSignal("错误：位置模式，轨迹位姿维度不足", "error");
            return;
        }
        for (int dim = 0; dim < 6; ++dim) {
            if (traj[endIndex][0][dim].size() != static_cast<size_t>(trajPointNum)) {
                emit displayInfoSignal("错误：位置模式，各维度轨迹点数量不一致", "error");
                return;
            }
        }
    }

    trajAnchorCoorVec.assign(trajPointNum, *anchorCableCoorHome);
    trajCableLenVec.assign(trajPointNum, std::vector<double>(anchorNum, 0.0));
    trajCableMotorThetaVec.assign(anchorNum, std::vector<double>(trajPointNum, 0.0));
    anchorTimeStampTraj.assign(anchorNum, std::vector<double>(trajPointNum, 0.0));

    QElapsedTimer playbackTimer;
    playbackTimer.start();
    const double playbackStartTime = pointTime(0);
    auto shouldContinue = [this]() {
        QMutexLocker locker(&mLock);
        return isRunning;
    };

    std::vector<double> homeCableLen(anchorNum, 0.0);
    for (int pointIndex = 0; pointIndex < trajPointNum; ++pointIndex) {
        if (!shouldContinue()) {
            return;
        }

        const qint64 targetElapsedMs = std::max<qint64>(
            0,
            static_cast<qint64>(std::llround((pointTime(pointIndex) - playbackStartTime) * 1000.0)));
        while (playbackTimer.elapsed() < targetElapsedMs) {
            if (!shouldContinue()) {
                return;
            }
            const qint64 remainingMs = targetElapsedMs - playbackTimer.elapsed();
            QThread::msleep(static_cast<unsigned long>(std::max<qint64>(1, std::min<qint64>(remainingMs, 5))));
        }

        QVector<QVector3D> targetPos(1);
        QVector<QVector3D> trajPos(endNum);
        QVector<QVector3D> anchorPos;
        QVector<QVector3D> cableEndPos;

        int anchorIndex = 0;
        for (int endIndex = 0; endIndex < endNum; ++endIndex) {
            const double px = traj[endIndex][0][0][pointIndex];
            const double py = traj[endIndex][0][1][pointIndex];
            const double pz = traj[endIndex][0][2][pointIndex];
            const double rx = traj[endIndex][0][3][pointIndex];
            const double ry = traj[endIndex][0][4][pointIndex];
            const double rz = traj[endIndex][0][5][pointIndex];
            trajPos[endIndex] = QVector3D(static_cast<float>(px / 1000.0),
                                          static_cast<float>(py / 1000.0),
                                          static_cast<float>(pz / 1000.0));

            if (endIndex == 0) {
                targetPos[0] = trajPos[endIndex];
            }

            for (const auto& contactPoint : endCableContactPos[endIndex]) {
                const std::vector<double> rotated = rotateContactPoint(contactPoint, rx, ry, rz);
                const std::vector<double> globalContactPoint = {
                    px + rotated[0],
                    py + rotated[1],
                    pz + rotated[2]
                };

                //加上了滑轮运动学
                //const double cableLen = distance3(anchorCableCoorHome->at(anchorIndex), globalContactPoint);
                const double cableLen = cableLengthCalculate(globalContactPoint, anchorCableCoorHome->at(anchorIndex), pulleyRadius);
                trajCableLenVec[pointIndex][anchorIndex] = cableLen;
                if (pointIndex == 0) {
                    homeCableLen[anchorIndex] = cableLen;
                    trajCableMotorThetaVec[anchorIndex][pointIndex] = 0.0;
                } else {
                    trajCableMotorThetaVec[anchorIndex][pointIndex] =
                        (homeCableLen[anchorIndex] - cableLen) * cableMotorCof[anchorIndex];
                }
                anchorTimeStampTraj[anchorIndex][pointIndex] = pointTime(pointIndex);

                anchorPos.push_back(QVector3D(static_cast<float>(anchorCableCoorHome->at(anchorIndex)[0] / 1000.0),
                                              static_cast<float>(anchorCableCoorHome->at(anchorIndex)[1] / 1000.0),
                                              static_cast<float>(anchorCableCoorHome->at(anchorIndex)[2] / 1000.0)));
                cableEndPos.push_back(QVector3D(static_cast<float>(globalContactPoint[0] / 1000.0),
                                                static_cast<float>(globalContactPoint[1] / 1000.0),
                                                static_cast<float>(globalContactPoint[2] / 1000.0)));
                ++anchorIndex;
            }
        }

        emit update3DViewerSignal(targetPos, trajPos, anchorPos, cableEndPos);
    }

    trajectorySimulationComplete = true;

    (void)isStaticAnchor;
    (void)frameL;
    (void)frameW;
}

double PositionSimulationModel::cableLengthCalculate(
    const std::vector<double>& a,   // 动平台连接点
    const std::vector<double>& b,   // 锚点
    double r                       // 滑轮半径
)
{
    return MatrixFun::cableLengthCalculate(a, b, r).idealLength;
#if 0
    // ===== 参数检查 =====
    if (a.size() < 3 || b.size() < 3 || r <= 0.0) {
        return 0.0;
    }

    // ===== 计算 ialpha =====
    double dx = a[0] - b[0];
    double dz = a[2] - b[2];

    double ialpha = atan2(dz, std::abs(dx));  // 比 MATLAB 更安全

    // ===== 圆心 O =====
    double Ox = b[0] + (dx >= 0 ? 1.0 : -1.0) * r * cos(ialpha);
    double Oy = b[1];
    double Oz = b[2] + r * sin(ialpha);

    std::vector<double> O = {Ox, Oy, Oz};

    // ===== l1 =====
    double l1 = sqrt(
        pow(O[0] - a[0], 2) +
        pow(O[1] - a[1], 2) +
        pow(O[2] - a[2], 2)
    );

    // ===== l2 =====
    double temp = l1 * l1 - r * r;
    if (temp < 0) temp = 0; // 防止数值误差
    double l2 = sqrt(temp);

    // ===== theta1 =====
    double theta1 = atan2(l2, r);

    // ===== theta2 =====
    double planarDist = sqrt(
        pow(a[0] - O[0], 2) +
        pow(a[2] - O[2], 2)
    );

    double theta2 = atan2((a[1] - O[1]), planarDist);

    // ===== theta3 =====
    double signY = (b[1] >= 0) ? 1.0 : -1.0;
    double theta3 = M_PI - (theta1 + signY * theta2);

    // ===== 圆弧长度 =====
    double c = theta3 * r;

    // ===== 总长度 =====
    return l2 + c;
#endif
}
