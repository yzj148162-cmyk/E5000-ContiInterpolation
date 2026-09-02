/*
 * 文件总览：
 * - PositionSimulationModel 的实现文件，在线程中遍历轨迹点，计算接点世界坐标、绳长、补偿后的电机角度和时间戳。
 * - run/poseCtrl 是主执行路径；validateInput 先做维度与参数检查，避免无效轨迹进入硬件执行阶段。
 * - 轨迹数据维度约定为“末端 -> 位置/速度/加速度/时间 -> 变量 -> 点”，阅读索引时需要保持该层级。
 */

#include "positionsimulationmodel.h"
#include "compensatedcablekinematics.h"
#include "MatrixFun.h"
#include "ropeelasticcompensation.h"
#include "trajectoryplanner.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr double kViewerFrameIntervalSec = 1.0 / 60.0;

bool isSixDimTrajPara(const std::vector<std::vector<double>>& data)
{
    return !data.empty() && data[0].size() == 6;
}

bool hasFinitePose6(const std::vector<double>& pose)
{
    if (pose.size() < 6) {
        return false;
    }
    for (int index = 0; index < 6; ++index) {
        if (!std::isfinite(pose[index])) {
            return false;
        }
    }
    return true;
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
                               double _pulleyRadius,
                               std::vector<WinchCompensation::AxisConfig> _winchCompensationConfig,
                               std::vector<std::vector<double>> _winchReferencePose,
                               std::vector<std::vector<double>> _cableForceTraj,
                               bool _applyRopeElasticCompensation,
                               std::vector<double> _ropeElasticFixedLengthL0Mm)
    : ctrlCycleMs(_ctrlCycleMs),
      useOfflineTraj(_useOfflineTraj),
      endNum(static_cast<int>(_endCableContactPos.size())),
      anchorNum(_anchorCableCoorHome ? static_cast<int>(_anchorCableCoorHome->size()) : 0),
      frameL(_frameL),
      frameW(_frameW),
      cableMotorCof(std::move(_cableMotorCof)),
      winchCompensationConfig(std::move(_winchCompensationConfig)),
      winchReferencePose(std::move(_winchReferencePose)),
      cableForceTraj(std::move(_cableForceTraj)),
      applyRopeElasticCompensation(_applyRopeElasticCompensation),
      ropeElasticFixedLengthL0Mm(std::move(_ropeElasticFixedLengthL0Mm)),
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
    // QThread 入口只负责切换运行标志和发送开始/结束信号，实际轨迹计算集中在 poseCtrl 中。
    {
        QMutexLocker locker(&mLock);
        isRunning = true;
    }

    qDebug() << "Pose ctrl thread created.";
    emit poseCtrlStartSignal();
    poseCtrl();
    qDebug() << "============POSE CTRL COMPLETE============";
    emit trajectorySimulationFinished(simulationPathSignatureText,
                                      simulationPathSegmentIndexValue,
                                      simulationPathSegmentCountValue,
                                      trajectorySimulationComplete);
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

std::vector<PositionSimulationModel::TrajectoryPointTimingSample>
PositionSimulationModel::getTrajectoryPointTimingSamples() const
{
    return trajectoryPointTimingSamples;
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

void PositionSimulationModel::setRealtimePlaybackEnabled(bool enabled)
{
    QMutexLocker locker(&mLock);
    realtimePlaybackEnabled = enabled;
}

void PositionSimulationModel::requestSkipVisualizationPlayback()
{
    QMutexLocker locker(&mLock);
    visualizationPlaybackSkipRequested = true;
    realtimePlaybackEnabled = false;
}

void PositionSimulationModel::setSimulationPathSignature(const QString& signature,
                                                         int segmentIndex,
                                                         int segmentCount)
{
    simulationPathSignatureText = signature;
    simulationPathSegmentIndexValue = std::max(0, segmentIndex);
    simulationPathSegmentCountValue = std::max(1, segmentCount);
}

QString PositionSimulationModel::simulationPathSignature() const
{
    return simulationPathSignatureText;
}

int PositionSimulationModel::simulationPathSegmentIndex() const
{
    return simulationPathSegmentIndexValue;
}

int PositionSimulationModel::simulationPathSegmentCount() const
{
    return simulationPathSegmentCountValue;
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
    // 这里做纯数据合法性检查，不做轨迹计算；任何维度不一致都会在进入硬件前被拦截。
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

std::vector<double> PositionSimulationModel::rotateContactPoint(
        const std::vector<double>& localPoint,
        const Eigen::Matrix3d& rotationGlobalFromBody) const
{
    if(localPoint.size() < 3){
        return {};
    }
    const Eigen::Vector3d rotated = rotationGlobalFromBody *
            Eigen::Vector3d(localPoint[0], localPoint[1], localPoint[2]);
    return {rotated(0), rotated(1), rotated(2)};
}

std::vector<std::vector<double>> PositionSimulationModel::trajectoryPoseAtPoint(int pointIndex) const
{
    std::vector<std::vector<double>> poseByEnd(endNum, std::vector<double>(6, 0.0));
    if (pointIndex < 0) {
        return {};
    }

    for (int endIndex = 0; endIndex < endNum; ++endIndex) {
        if (endIndex >= static_cast<int>(traj.size()) ||
                traj[endIndex].empty() ||
                traj[endIndex][0].size() < 6) {
            return {};
        }
        for (int dim = 0; dim < 6; ++dim) {
            if (pointIndex >= static_cast<int>(traj[endIndex][0][dim].size())) {
                return {};
            }
            poseByEnd[endIndex][dim] = traj[endIndex][0][dim][pointIndex];
        }
    }
    return poseByEnd;
}

std::vector<std::vector<double>> PositionSimulationModel::normalizedWinchReferencePose() const
{
    std::vector<std::vector<double>> referencePose(endNum, std::vector<double>(6, 0.0));
    for (int endIndex = 0; endIndex < endNum; ++endIndex) {
        if (endIndex < static_cast<int>(winchReferencePose.size()) &&
                hasFinitePose6(winchReferencePose[endIndex])) {
            for (int dim = 0; dim < 6; ++dim) {
                referencePose[endIndex][dim] = winchReferencePose[endIndex][dim];
            }
        }
    }
    return referencePose;
}

std::vector<double> PositionSimulationModel::cableLengthsForPoseMatrix(
        const std::vector<std::vector<double>>& poseByEnd) const
{
    if (!anchorCableCoorHome || static_cast<int>(poseByEnd.size()) < endNum) {
        return {};
    }

    std::vector<double> lengths;
    lengths.reserve(anchorNum);
    int anchorIndex = 0;
    for (int endIndex = 0; endIndex < endNum; ++endIndex) {
        if (!hasFinitePose6(poseByEnd[endIndex])) {
            return {};
        }
        const double px = poseByEnd[endIndex][0];
        const double py = poseByEnd[endIndex][1];
        const double pz = poseByEnd[endIndex][2];
        const double rx = poseByEnd[endIndex][3];
        const double ry = poseByEnd[endIndex][4];
        const double rz = poseByEnd[endIndex][5];

        for (const auto& contactPoint : endCableContactPos[endIndex]) {
            if (anchorIndex >= static_cast<int>(anchorCableCoorHome->size())) {
                return {};
            }
            const std::vector<double> rotated = rotateContactPoint(contactPoint, rx, ry, rz);
            const std::vector<double> globalContactPoint = {
                px + rotated[0],
                py + rotated[1],
                pz + rotated[2]
            };
            lengths.push_back(cableLengthCalculate(globalContactPoint,
                                                   anchorCableCoorHome->at(anchorIndex),
                                                   pulleyRadius));
            ++anchorIndex;
        }
    }

    if (static_cast<int>(lengths.size()) != anchorNum) {
        return {};
    }
    return lengths;
}

void PositionSimulationModel::poseCtrl()
{
    // 位置仿真主流程：构建末端轨迹 -> 逐点算绳长 -> 合入绞盘/弹性补偿 -> 输出电机角度轨迹。
    trajectorySimulationComplete = false;
    trajectoryPointTimingSamples.clear();

    if (!validateInput()) {
        return;
    }

    endNum = static_cast<int>(endCableContactPos.size());
    anchorNum = static_cast<int>(anchorCableCoorHome->size());

    traj.clear();
    traj.resize(endNum);
    if (useOfflineTraj) {
        // 外部文件轨迹已包含完整位姿点，本阶段只校验结构并拷贝到统一轨迹容器。
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
        // UI 起终点轨迹统一生成五次多项式，后续处理不再区分来源。
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
    std::vector<bool> endUsesSO3Attitude(endNum, false);
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

        const bool hasSO3Layer =
                traj[endIndex].size() > TrajectoryPlanner::kSO3RotationLayer &&
                !traj[endIndex][TrajectoryPlanner::kSO3RotationLayer].empty();
        if(hasSO3Layer && !TrajectoryPlanner::hasSO3AttitudeData(traj[endIndex])){
            emit displayInfoSignal("错误：位置模式，SO(3)姿态数据不完整", "error");
            return;
        }
        endUsesSO3Attitude[endIndex] = hasSO3Layer;
    }

    trajAnchorCoorVec.assign(trajPointNum, *anchorCableCoorHome);
    trajCableLenVec.assign(trajPointNum, std::vector<double>(anchorNum, 0.0));
    trajCableMotorThetaVec.assign(anchorNum, std::vector<double>(trajPointNum, 0.0));
    anchorTimeStampTraj.assign(anchorNum, std::vector<double>(trajPointNum, 0.0));
    trajectoryPointTimingSamples.clear();
    trajectoryPointTimingSamples.reserve(trajPointNum);

    QElapsedTimer playbackTimer;
    playbackTimer.start();
    const double playbackStartTime = pointTime(0);
    auto playbackState = [this](bool* realtimeEnabled, bool* skipRequested) {
        QMutexLocker locker(&mLock);
        if (realtimeEnabled) {
            *realtimeEnabled = realtimePlaybackEnabled;
        }
        if (skipRequested) {
            *skipRequested = visualizationPlaybackSkipRequested;
        }
        return isRunning;
    };

    // 补偿运动学显式保存绞盘连续解。轨迹仿真和末端遥控共用这一套
    // “绳长 -> 补偿后电机角度”实现，避免两条执行路径的公式逐渐漂移。
    RopeElasticCompensation::Config ropeElasticConfig =
            RopeElasticCompensation::defaultConfig();
    if (static_cast<int>(ropeElasticFixedLengthL0Mm.size()) == RopeElasticCompensation::kCableCount) {
        bool fixedLengthValid = true;
        for (double lengthMm : ropeElasticFixedLengthL0Mm) {
            if (!std::isfinite(lengthMm) || lengthMm < 0.0) {
                fixedLengthValid = false;
                break;
            }
        }
        if (fixedLengthValid) {
            for (int cableIndex = 0; cableIndex < RopeElasticCompensation::kCableCount; ++cableIndex) {
                ropeElasticConfig.fixedLengthL0Mm[cableIndex] =
                        ropeElasticFixedLengthL0Mm[cableIndex];
            }
        }
    }
    ropeElasticConfig.enabled =
            applyRopeElasticCompensation && ropeElasticConfig.enabled;
    auto cableForceForAnchorPoint = [this](int anchorIndex, int pointIndex) {
        if (anchorIndex >= 0 &&
                anchorIndex < static_cast<int>(cableForceTraj.size()) &&
                pointIndex >= 0 &&
                pointIndex < static_cast<int>(cableForceTraj[anchorIndex].size())) {
            return cableForceTraj[anchorIndex][pointIndex];
        }
        return 0.0;
    };
    const std::vector<std::vector<double>> startPose = trajectoryPoseAtPoint(0);
    std::vector<double> initialCableTension(anchorNum, 0.0);
    for(int anchorIndex = 0; anchorIndex < anchorNum; ++anchorIndex){
        initialCableTension[anchorIndex] =
                cableForceForAnchorPoint(anchorIndex, 0);
    }
    CompensatedCableKinematics::Configuration kinematicsConfig;
    kinematicsConfig.cableMotorScaleRadPerMm = cableMotorCof;
    kinematicsConfig.winchConfig = winchCompensationConfig;
    kinematicsConfig.winchReferencePose = winchReferencePose;
    kinematicsConfig.endCableContactPos = endCableContactPos;
    kinematicsConfig.anchorCableCoordinate = *anchorCableCoorHome;
    kinematicsConfig.pulleyRadiusMm = pulleyRadius;
    kinematicsConfig.ropeElasticConfig = ropeElasticConfig;
    CompensatedCableKinematics compensatedKinematics;
    QString kinematicsError;
    if(!compensatedKinematics.initialize(kinematicsConfig,
                                         startPose,
                                         initialCableTension,
                                         &kinematicsError)){
        emit displayInfoSignal(
                    QStringLiteral("错误：位置模式补偿运动学初始化失败：%1")
                    .arg(kinematicsError).toStdString(),
                    "error");
        return;
    }
    CompensatedCableKinematics::State compensatedKinematicsState =
            compensatedKinematics.initialState();

    double lastViewerFrameTime = -std::numeric_limits<double>::infinity();
    for (int pointIndex = 0; pointIndex < trajPointNum; ++pointIndex) {
        bool realtimeEnabled = true;
        bool skipRequested = false;
        if (!playbackState(&realtimeEnabled, &skipRequested)) {
            return;
        }

        const double currentPointTime = pointTime(pointIndex);
        const qint64 targetElapsedMs = std::max<qint64>(
            0,
            static_cast<qint64>(std::llround((currentPointTime - playbackStartTime) * 1000.0)));
        while (realtimeEnabled && !skipRequested && playbackTimer.elapsed() < targetElapsedMs) {
            if (!playbackState(&realtimeEnabled, &skipRequested)) {
                return;
            }
            const qint64 remainingMs = targetElapsedMs - playbackTimer.elapsed();
            QThread::msleep(static_cast<unsigned long>(std::max<qint64>(1, std::min<qint64>(remainingMs, 5))));
        }

        const bool emitViewerFrame =
                (!skipRequested && pointIndex == 0) ||
                pointIndex == trajPointNum - 1 ||
                (!skipRequested && currentPointTime - lastViewerFrameTime >= kViewerFrameIntervalSec);
        QVector<QVector3D> targetPos;
        QVector<QVector3D> trajPos;
        QVector<QVector3D> anchorPos;
        QVector<QVector3D> cableEndPos;
        qint64 cableLengthCalculationUs = 0;
        if (emitViewerFrame) {
            targetPos.resize(1);
            trajPos.resize(endNum);
            anchorPos.reserve(anchorNum);
            cableEndPos.reserve(anchorNum);
        }

        int anchorIndex = 0;
        for (int endIndex = 0; endIndex < endNum; ++endIndex) {
            const double px = traj[endIndex][0][0][pointIndex];
            const double py = traj[endIndex][0][1][pointIndex];
            const double pz = traj[endIndex][0][2][pointIndex];
            const double rx = traj[endIndex][0][3][pointIndex];
            const double ry = traj[endIndex][0][4][pointIndex];
            const double rz = traj[endIndex][0][5][pointIndex];
            TrajectoryPlanner::SO3AttitudeSample so3Attitude;
            if(endUsesSO3Attitude[endIndex] &&
                    !TrajectoryPlanner::readSO3AttitudeSample(traj[endIndex], pointIndex, so3Attitude)){
                emit displayInfoSignal("错误：位置模式，读取SO(3)姿态轨迹点失败", "error");
                return;
            }
            if (emitViewerFrame) {
                trajPos[endIndex] = QVector3D(static_cast<float>(px / 1000.0),
                                              static_cast<float>(py / 1000.0),
                                              static_cast<float>(pz / 1000.0));

                if (endIndex == 0) {
                    targetPos[0] = trajPos[endIndex];
                }
            }

            for (const auto& contactPoint : endCableContactPos[endIndex]) {
                const std::vector<double> rotated = endUsesSO3Attitude[endIndex] ?
                            rotateContactPoint(contactPoint, so3Attitude.rotationGlobalFromBody) :
                            rotateContactPoint(contactPoint, rx, ry, rz);
                if(rotated.size() < 3){
                    emit displayInfoSignal("错误：位置模式，接点旋转计算失败", "error");
                    return;
                }
                const std::vector<double> globalContactPoint = {
                    px + rotated[0],
                    py + rotated[1],
                    pz + rotated[2]
                };

                //加上了滑轮运动学
                //const double cableLen = distance3(anchorCableCoorHome->at(anchorIndex), globalContactPoint);
                QElapsedTimer cableLengthTimer;
                cableLengthTimer.start();
                const double cableLen = cableLengthCalculate(globalContactPoint, anchorCableCoorHome->at(anchorIndex), pulleyRadius);
                cableLengthCalculationUs += cableLengthTimer.nsecsElapsed() / 1000;
                trajCableLenVec[pointIndex][anchorIndex] = cableLen;
                anchorTimeStampTraj[anchorIndex][pointIndex] = currentPointTime;

                if (emitViewerFrame) {
                    anchorPos.push_back(QVector3D(static_cast<float>(anchorCableCoorHome->at(anchorIndex)[0] / 1000.0),
                                                  static_cast<float>(anchorCableCoorHome->at(anchorIndex)[1] / 1000.0),
                                                  static_cast<float>(anchorCableCoorHome->at(anchorIndex)[2] / 1000.0)));
                    cableEndPos.push_back(QVector3D(static_cast<float>(globalContactPoint[0] / 1000.0),
                                                    static_cast<float>(globalContactPoint[1] / 1000.0),
                                                    static_cast<float>(globalContactPoint[2] / 1000.0)));
                }
                ++anchorIndex;
            }
        }

        std::vector<double> pointCableTension(anchorNum, 0.0);
        for(int cableIndex = 0; cableIndex < anchorNum; ++cableIndex){
            pointCableTension[cableIndex] =
                    cableForceForAnchorPoint(cableIndex, pointIndex);
        }
        const CompensatedCableKinematics::Evaluation motorEvaluation =
                compensatedKinematics.evaluateCableLengths(
                    trajCableLenVec[pointIndex],
                    compensatedKinematicsState,
                    pointCableTension);
        if(!motorEvaluation.valid ||
                static_cast<int>(motorEvaluation.relativeMotorThetaRad.size()) != anchorNum){
            emit displayInfoSignal(
                        QStringLiteral("错误：位置模式轨迹点%1补偿运动学失败：%2")
                        .arg(pointIndex)
                        .arg(motorEvaluation.errorMessage).toStdString(),
                        "error");
            return;
        }
        compensatedKinematicsState = motorEvaluation.nextState;
        for(int cableIndex = 0; cableIndex < anchorNum; ++cableIndex){
            trajCableMotorThetaVec[cableIndex][pointIndex] =
                    motorEvaluation.relativeMotorThetaRad[cableIndex];
        }

        if (emitViewerFrame) {
            emit update3DViewerSignal(targetPos, trajPos, anchorPos, cableEndPos);
            lastViewerFrameTime = currentPointTime;
        }
        TrajectoryPointTimingSample timingSample;
        timingSample.pointIndex = pointIndex;
        timingSample.trajectoryTimeSec = currentPointTime;
        timingSample.cableLengthCalculationUs = cableLengthCalculationUs;
        trajectoryPointTimingSamples.push_back(timingSample);
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
) const
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
