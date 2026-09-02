/*
 * 文件总览：
 * - TrajectoryPlanner 的实现文件，生成位置/速度/加速度/时间四类轨迹并按多末端结构组织。
 * - 文件轨迹入口会解析外部点列、分段标记和时间戳，供实时轨迹或暂停恢复流程复用。
 * - 过渡轨迹函数用于把当前实测位置平滑接回原轨迹，减少急停/恢复时的位置突变。
 */

#include "trajectoryplanner.h"

#include <QFile>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <complex>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

namespace {

// 校验离散点轨迹的长度、维度和严格递增时间戳。
bool validatePointTrajectory(const std::vector<std::vector<double>>& positionTraj,
                             const std::vector<double>& timeStamp,
                             QString& errorMessage)
{
    // 至少要有两个点，且时间与轨迹长度一致
    if (positionTraj.size() < 2 || timeStamp.size() < 2 || positionTraj.size() != timeStamp.size()) {
        errorMessage = QStringLiteral("轨迹点或时间戳数量不足");
        return false;
    }

    // 获取维度（例如 xyz 或 xyz+rpy）
    const size_t dim = positionTraj.front().size();
    if (dim == 0) {
        errorMessage = QStringLiteral("轨迹维度为空");
        return false;
    }

    // 检查每个点维度一致，且时间严格递增
    for (size_t i = 0; i < positionTraj.size(); ++i) {
        if (positionTraj[i].size() != dim) {
            errorMessage = QStringLiteral("轨迹各点维度不一致");
            return false;
        }
        if (i > 0 && timeStamp[i] <= timeStamp[i - 1]) {
            errorMessage = QStringLiteral("轨迹时间戳必须严格递增");
            return false;
        }
    }

    return true;
}

// 将时间裁剪到合法区间，避免插值越界。
double clampTime(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

// 将一行空格分隔文本解析为 double 数组，并可校验列数。
bool parseDoubleLine(const QString& line,
                     int expectedCount,
                     std::vector<double>& values)
{
    const QStringList tokens = line.simplified().split(' ', Qt::SkipEmptyParts);
    if (expectedCount > 0 && tokens.size() != expectedCount) {
        return false;
    }

    values.clear();
    values.reserve(tokens.size());
    for (const QString& token : tokens) {
        bool ok = false;
        const double value = token.toDouble(&ok);
        if (!ok || !std::isfinite(value)) {
            return false;
        }
        values.push_back(value);
    }
    return true;
}

// 将分段标记行解析为 0/1 标记数组。
bool parseSegmentFlagLine(const QString& line,
                          int expectedCount,
                          std::vector<int>& flags)
{
    std::vector<double> values;
    if (!parseDoubleLine(line, expectedCount, values)) {
        return false;
    }

    flags.clear();
    flags.reserve(values.size());
    for (double value : values) {
        if (std::abs(value) <= 1e-9) {
            flags.push_back(0);
        } else if (std::abs(value - 1.0) <= 1e-9) {
            flags.push_back(1);
        } else {
            return false;
        }
    }
    return true;
}

// 按点数和采样周期生成均匀时间轴。
std::vector<double> buildUniformTimeAxis(int pointNum, double stepTime)
{
    std::vector<double> timeStamp;
    if (pointNum <= 0 || stepTime <= 0.0) {
        return timeStamp;
    }

    timeStamp.reserve(pointNum);
    for (int index = 0; index < pointNum; ++index) {
        timeStamp.push_back(index * stepTime);
    }
    return timeStamp;
}

// 用前向/后向/中心差分估计离散序列导数。
std::vector<double> estimateTrajectoryDerivative(const std::vector<double>& samples,
                                                 const std::vector<double>& timeStamp)
{
    std::vector<double> derivative(samples.size(), 0.0);
    if (samples.size() != timeStamp.size() || samples.size() < 2) {
        return derivative;
    }

    for (size_t index = 0; index < samples.size(); ++index) {
        if (index == 0) {
            const double dt = timeStamp[1] - timeStamp[0];
            derivative[index] = dt > 1e-9 ? (samples[1] - samples[0]) / dt : 0.0;
        } else if (index + 1 == samples.size()) {
            const double dt = timeStamp[index] - timeStamp[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index] - samples[index - 1]) / dt : 0.0;
        } else {
            const double dt = timeStamp[index + 1] - timeStamp[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index + 1] - samples[index - 1]) / dt : 0.0;
        }
    }

    return derivative;
}

constexpr double kSO3SmallAngle = 1e-8;
constexpr double kSO3GimbalLockCosine = 1e-10;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double clampUnit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

bool isFiniteEuler(const std::vector<double>& euler)
{
    return euler.size() == 3 &&
            std::isfinite(euler[0]) &&
            std::isfinite(euler[1]) &&
            std::isfinite(euler[2]);
}

Eigen::Matrix3d eulerZYXToRotation(const Eigen::Vector3d& euler)
{
    const double roll = euler(0);
    const double pitch = euler(1);
    const double yaw = euler(2);
    const double cRoll = std::cos(roll);
    const double sRoll = std::sin(roll);
    const double cPitch = std::cos(pitch);
    const double sPitch = std::sin(pitch);
    const double cYaw = std::cos(yaw);
    const double sYaw = std::sin(yaw);

    Eigen::Matrix3d rotation;
    rotation << cYaw * cPitch, cYaw * sPitch * sRoll - sYaw * cRoll,
            cYaw * sPitch * cRoll + sYaw * sRoll,
            sYaw * cPitch, sYaw * sPitch * sRoll + cYaw * cRoll,
            sYaw * sPitch * cRoll - cYaw * sRoll,
            -sPitch, cPitch * sRoll, cPitch * cRoll;
    return rotation;
}

Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& rotation)
{
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (U * V.transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    return U * correction * V.transpose();
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& rotationVector)
{
    const double angle = rotationVector.norm();
    Eigen::Matrix3d skew;
    skew << 0.0, -rotationVector(2), rotationVector(1),
            rotationVector(2), 0.0, -rotationVector(0),
            -rotationVector(1), rotationVector(0), 0.0;

    double firstCoefficient = 0.0;
    double secondCoefficient = 0.0;
    if(angle < kSO3SmallAngle){
        const double angle2 = angle * angle;
        firstCoefficient = 1.0 - angle2 / 6.0 + angle2 * angle2 / 120.0;
        secondCoefficient = 0.5 - angle2 / 24.0 + angle2 * angle2 / 720.0;
    }
    else{
        firstCoefficient = std::sin(angle) / angle;
        secondCoefficient = (1.0 - std::cos(angle)) / (angle * angle);
    }
    return Eigen::Matrix3d::Identity() + firstCoefficient * skew +
            secondCoefficient * skew * skew;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& inputRotation)
{
    const Eigen::Matrix3d rotation = projectToSO3(inputRotation);
    const double cosine = clampUnit((rotation.trace() - 1.0) * 0.5);
    const double angle = std::acos(cosine);
    const Eigen::Vector3d skewVector(
                rotation(2, 1) - rotation(1, 2),
                rotation(0, 2) - rotation(2, 0),
                rotation(1, 0) - rotation(0, 1));

    if(angle < kSO3SmallAngle){
        return 0.5 * skewVector;
    }
    if(kPi - angle < 1e-6){
        // 与 MATLAB plan_trj_SO3.m 的 so3Log 保持相同的取轴逻辑：
        // [V,D] = eig(R)，取最接近特征值 1 的特征向量并取实部。
        // 不再人为翻转轴符号；在 pi 旋转处 u 和 -u 都合法，应遵从
        // eig 求解器返回的分支，才能尽量贴近 MATLAB 的中间旋转方向。
        Eigen::EigenSolver<Eigen::Matrix3d> eigensolver(rotation);
        const Eigen::Vector3cd eigenvalues = eigensolver.eigenvalues();
        Eigen::Index axisIndex = 0;
        double closestDistance = std::hypot(eigenvalues(0).real() - 1.0,
                                            eigenvalues(0).imag());
        for(Eigen::Index index = 1; index < eigenvalues.size(); ++index){
            const double distance = std::hypot(eigenvalues(index).real() - 1.0,
                                               eigenvalues(index).imag());
            if(distance < closestDistance){
                closestDistance = distance;
                axisIndex = index;
            }
        }
        Eigen::Vector3d axis = eigensolver.eigenvectors().col(axisIndex).real();
        if(axis.norm() < kSO3SmallAngle){
            axis = Eigen::Vector3d::UnitX();
        }
        else{
            axis.normalize();
        }
        return angle * axis;
    }
    return angle / (2.0 * std::sin(angle)) * skewVector;
}

Eigen::Vector3d nearestTwoPi(Eigen::Vector3d value,
                             const Eigen::Vector3d& reference)
{
    for(int index = 0; index < 3; ++index){
        value(index) += kTwoPi * std::round((reference(index) - value(index)) / kTwoPi);
    }
    return value;
}

Eigen::Vector3d rotationToEulerZYXNear(const Eigen::Matrix3d& rotation,
                                       const Eigen::Vector3d& reference)
{
    const double pitch = std::asin(clampUnit(-rotation(2, 0)));
    const double cosinePitch = std::cos(pitch);
    if(std::abs(cosinePitch) > kSO3GimbalLockCosine){
        const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
        const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        const Eigen::Vector3d candidate1 =
                nearestTwoPi(Eigen::Vector3d(roll, pitch, yaw), reference);
        const Eigen::Vector3d candidate2 = nearestTwoPi(
                    Eigen::Vector3d(roll + kPi, kPi - pitch, yaw + kPi),
                    reference);
        return (candidate2 - reference).norm() < (candidate1 - reference).norm() ?
                    candidate2 : candidate1;
    }

    const double roll = reference(0);
    double yaw = 0.0;
    if(pitch > 0.0){
        yaw = roll + std::atan2(-rotation(0, 1), rotation(1, 1));
    }
    else{
        yaw = std::atan2(-rotation(0, 1), rotation(1, 1)) - roll;
    }
    return nearestTwoPi(Eigen::Vector3d(roll, pitch, yaw), reference);
}

void setSO3PlanningError(QString* errorMessage, const QString& message)
{
    if(errorMessage){
        *errorMessage = message;
    }
}

bool prepareSO3Trajectory(TrajectoryPlanner::TrajectoryBlock& trajectory,
                          std::vector<double>& timeAxis,
                          QString* errorMessage,
                          const QString& context)
{
    if(trajectory.size() < TrajectoryPlanner::kTrajectoryTimeLayer + 1 ||
            trajectory[TrajectoryPlanner::kTrajectoryPoseLayer].size() < 6 ||
            trajectory[TrajectoryPlanner::kTrajectoryVelocityLayer].size() < 6 ||
            trajectory[TrajectoryPlanner::kTrajectoryAccelerationLayer].size() < 6 ||
            trajectory[TrajectoryPlanner::kTrajectoryTimeLayer].empty()){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：轨迹数据不完整").arg(context));
        return false;
    }

    timeAxis = trajectory[TrajectoryPlanner::kTrajectoryTimeLayer][0];
    const int pointCount = static_cast<int>(timeAxis.size());
    if(pointCount < 2){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：时间轴至少需要两个点").arg(context));
        return false;
    }
    for(int dimension = 0; dimension < 6; ++dimension){
        if(static_cast<int>(trajectory[TrajectoryPlanner::kTrajectoryPoseLayer][dimension].size()) != pointCount ||
                static_cast<int>(trajectory[TrajectoryPlanner::kTrajectoryVelocityLayer][dimension].size()) != pointCount ||
                static_cast<int>(trajectory[TrajectoryPlanner::kTrajectoryAccelerationLayer][dimension].size()) != pointCount){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("%1：轨迹各维度点数不一致").arg(context));
            return false;
        }
    }
    for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
        if(!std::isfinite(timeAxis[pointIndex]) ||
                (pointIndex > 0 && timeAxis[pointIndex] <= timeAxis[pointIndex - 1])){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("%1：时间轴必须为有限且严格递增的真实时间戳").arg(context));
            return false;
        }
    }

    trajectory.resize(TrajectoryPlanner::kSO3AlphaGlobalLayer + 1);
    trajectory[TrajectoryPlanner::kSO3RotationLayer].assign(9, std::vector<double>(pointCount, 0.0));
    trajectory[TrajectoryPlanner::kSO3OmegaBodyLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[TrajectoryPlanner::kSO3AlphaBodyLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[TrajectoryPlanner::kSO3OmegaGlobalLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[TrajectoryPlanner::kSO3AlphaGlobalLayer].assign(3, std::vector<double>(pointCount, 0.0));
    return true;
}

void writeSO3TrajectorySample(TrajectoryPlanner::TrajectoryBlock& trajectory,
                              int pointIndex,
                              const Eigen::Matrix3d& rotation,
                              const Eigen::Vector3d& displayEuler,
                              const Eigen::Vector3d& omegaBody,
                              const Eigen::Vector3d& alphaBody)
{
    const Eigen::Vector3d omegaGlobal = rotation * omegaBody;
    const Eigen::Vector3d alphaGlobal = rotation * alphaBody;
    for(int row = 0; row < 3; ++row){
        for(int column = 0; column < 3; ++column){
            trajectory[TrajectoryPlanner::kSO3RotationLayer][3 * row + column][pointIndex] =
                    rotation(row, column);
        }
        trajectory[TrajectoryPlanner::kSO3OmegaBodyLayer][row][pointIndex] = omegaBody(row);
        trajectory[TrajectoryPlanner::kSO3AlphaBodyLayer][row][pointIndex] = alphaBody(row);
        trajectory[TrajectoryPlanner::kSO3OmegaGlobalLayer][row][pointIndex] = omegaGlobal(row);
        trajectory[TrajectoryPlanner::kSO3AlphaGlobalLayer][row][pointIndex] = alphaGlobal(row);

        // 与 MATLAB 的 SO(3) 分支保持一致：遗留字段使用全局角速度/角加速度。
        trajectory[TrajectoryPlanner::kTrajectoryVelocityLayer][row + 3][pointIndex] = omegaGlobal(row);
        trajectory[TrajectoryPlanner::kTrajectoryAccelerationLayer][row + 3][pointIndex] = alphaGlobal(row);
        trajectory[TrajectoryPlanner::kTrajectoryPoseLayer][row + 3][pointIndex] = displayEuler(row);
    }
}

bool buildSO3EndpointData(const std::vector<double>& startEuler,
                          const std::vector<double>& endEuler,
                          Eigen::Vector3d& startEulerVector,
                          Eigen::Matrix3d& startRotation,
                          Eigen::Vector3d& rotationVector,
                          QString* errorMessage,
                          const QString& context)
{
    if(!isFiniteEuler(startEuler) || !isFiniteEuler(endEuler)){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：起点或终点欧拉角无效").arg(context));
        return false;
    }
    startEulerVector = Eigen::Vector3d(startEuler[0], startEuler[1], startEuler[2]);
    const Eigen::Vector3d endEulerVector(endEuler[0], endEuler[1], endEuler[2]);
    startRotation = eulerZYXToRotation(startEulerVector);
    rotationVector = so3Log(startRotation.transpose() * eulerZYXToRotation(endEulerVector));
    if(rotationVector.norm() < kSO3SmallAngle &&
            (endEulerVector - startEulerVector).norm() > 1e-6){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：SO(3)不支持用等价欧拉角表达多圈旋转或分支切换；请改用欧拉角模式")
                            .arg(context));
        return false;
    }
    return true;
}

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d skew;
    skew << 0.0, -vector(2), vector(1),
            vector(2), 0.0, -vector(0),
            -vector(1), vector(0), 0.0;
    return skew;
}

Eigen::Vector3d eulerZYXRatesToBodyOmega(const Eigen::Vector3d& euler,
                                          const Eigen::Vector3d& eulerRate)
{
    const double roll = euler(0);
    const double pitch = euler(1);
    Eigen::Vector3d omegaBody;
    omegaBody(0) = eulerRate(0) - std::sin(pitch) * eulerRate(2);
    omegaBody(1) = std::cos(roll) * eulerRate(1) +
            std::sin(roll) * std::cos(pitch) * eulerRate(2);
    omegaBody(2) = -std::sin(roll) * eulerRate(1) +
            std::cos(roll) * std::cos(pitch) * eulerRate(2);
    return omegaBody;
}

Eigen::Vector3d eulerZYXAccelerationsToBodyAlpha(const Eigen::Vector3d& euler,
                                                  const Eigen::Vector3d& eulerRate,
                                                  const Eigen::Vector3d& eulerAcceleration)
{
    const double roll = euler(0);
    const double pitch = euler(1);
    const double rollRate = eulerRate(0);
    const double pitchRate = eulerRate(1);

    Eigen::Matrix3d jacobian;
    jacobian << 1.0, 0.0, -std::sin(pitch),
            0.0, std::cos(roll), std::sin(roll) * std::cos(pitch),
            0.0, -std::sin(roll), std::cos(roll) * std::cos(pitch);
    Eigen::Matrix3d jacobianDot;
    jacobianDot << 0.0, 0.0, -std::cos(pitch) * pitchRate,
            0.0, -std::sin(roll) * rollRate,
            std::cos(roll) * rollRate * std::cos(pitch) -
                    std::sin(roll) * std::sin(pitch) * pitchRate,
            0.0, -std::cos(roll) * rollRate,
            -std::sin(roll) * rollRate * std::cos(pitch) -
                    std::cos(roll) * std::sin(pitch) * pitchRate;
    return jacobian * eulerAcceleration + jacobianDot * eulerRate;
}

Eigen::Matrix3d so3RightJacobian(const Eigen::Vector3d& rotationVector)
{
    const double angle = rotationVector.norm();
    const Eigen::Matrix3d skew = skewMatrix(rotationVector);
    if(angle < kSO3SmallAngle){
        return Eigen::Matrix3d::Identity() - 0.5 * skew +
                (1.0 / 6.0) * skew * skew;
    }
    const double coefficientA = (1.0 - std::cos(angle)) / (angle * angle);
    const double coefficientB = (angle - std::sin(angle)) / (angle * angle * angle);
    return Eigen::Matrix3d::Identity() - coefficientA * skew +
            coefficientB * skew * skew;
}

Eigen::Matrix3d so3RightJacobianInverse(const Eigen::Vector3d& rotationVector)
{
    const double angle = rotationVector.norm();
    const Eigen::Matrix3d skew = skewMatrix(rotationVector);
    if(angle < kSO3SmallAngle){
        return Eigen::Matrix3d::Identity() + 0.5 * skew +
                (1.0 / 12.0) * skew * skew;
    }
    const double coefficient = 1.0 / (angle * angle) -
            1.0 / (2.0 * angle * std::tan(0.5 * angle));
    return Eigen::Matrix3d::Identity() + 0.5 * skew + coefficient * skew * skew;
}

Eigen::Matrix3d so3RightJacobianDerivative(const Eigen::Vector3d& rotationVector,
                                            const Eigen::Vector3d& rotationVectorRate)
{
    const double angle = rotationVector.norm();
    const Eigen::Matrix3d skew = skewMatrix(rotationVector);
    const Eigen::Matrix3d skewRate = skewMatrix(rotationVectorRate);
    if(angle < kSO3SmallAngle){
        return -0.5 * skewRate + (1.0 / 6.0) *
                (skewRate * skew + skew * skewRate);
    }
    const double angleRate = rotationVector.dot(rotationVectorRate) / angle;
    const double coefficientA = (1.0 - std::cos(angle)) / (angle * angle);
    const double coefficientB = (angle - std::sin(angle)) / (angle * angle * angle);
    const double coefficientADot =
            (angle * std::sin(angle) - 2.0 * (1.0 - std::cos(angle))) /
            (angle * angle * angle) * angleRate;
    const double coefficientBDot =
            (angle * (1.0 - std::cos(angle)) - 3.0 * (angle - std::sin(angle))) /
            (angle * angle * angle * angle) * angleRate;
    return -coefficientADot * skew - coefficientA * skewRate +
            coefficientBDot * skew * skew + coefficientB *
            (skewRate * skew + skew * skewRate);
}

struct SegmentPoseDefinition
{
    std::vector<double> startPose;
    std::vector<double> endPose;
    double duration = 0.0;
    double stepTime = 0.0;
};

std::vector<double> segmentFilePoseToPlannerPose(std::vector<double> pose,
                                                 double rxOffsetRad)
{
    if (pose.size() >= 4) {
        pose[3] += rxOffsetRad;
    }
    return pose;
}

bool isSegmentPoseFileHeader(const QStringList& data)
{
    if (data.isEmpty()) {
        return false;
    }

    QString header = data.first().trimmed().toLower();
    header.replace(QLatin1Char('-'), QLatin1Char('_'));
    return header == QStringLiteral("pose_segments_v1") ||
            header == QStringLiteral("line_pose_segments_v1") ||
            header == QStringLiteral("segment_pose_trajectory");
}

bool parsePositiveIntLine(const QString& line, int& value)
{
    bool ok = false;
    const int parsed = line.simplified().toInt(&ok);
    if (!ok || parsed <= 0) {
        return false;
    }

    value = parsed;
    return true;
}

bool parseSingleDoubleLine(const QString& line, double& value)
{
    std::vector<double> values;
    if (!parseDoubleLine(line, 1, values)) {
        return false;
    }

    value = values.front();
    return true;
}

bool parseDurationStepLine(const QString& line,
                           double& duration,
                           double& stepTime)
{
    std::vector<double> values;
    if (!parseDoubleLine(line, 2, values)) {
        return false;
    }

    duration = values[0];
    stepTime = values[1];
    return true;
}

bool parseSegmentPoseRow(const QString& line, SegmentPoseDefinition& segment)
{
    std::vector<double> values;
    if (!parseDoubleLine(line, 14, values)) {
        return false;
    }

    segment.startPose.assign(values.begin(), values.begin() + 6);
    segment.endPose.assign(values.begin() + 6, values.begin() + 12);
    segment.duration = values[12];
    segment.stepTime = values[13];
    return true;
}

bool validateSegmentPoseDefinition(const SegmentPoseDefinition& segment,
                                   int segmentNumber,
                                   QString& errorMessage)
{
    if (segment.startPose.size() != 6 || segment.endPose.size() != 6) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段起点或终点位姿维度不是 6").arg(segmentNumber);
        return false;
    }

    for (double value : segment.startPose) {
        if (!std::isfinite(value)) {
            errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段起点位姿包含无效数值").arg(segmentNumber);
            return false;
        }
    }
    for (double value : segment.endPose) {
        if (!std::isfinite(value)) {
            errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段终点位姿包含无效数值").arg(segmentNumber);
            return false;
        }
    }
    if (!std::isfinite(segment.duration) || segment.duration <= 1e-9) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段时长必须大于 0").arg(segmentNumber);
        return false;
    }
    if (!std::isfinite(segment.stepTime) || segment.stepTime <= 1e-9) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段时间步长必须大于 0").arg(segmentNumber);
        return false;
    }
    if (segment.stepTime > segment.duration + 1e-9) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段时间步长不能大于该段时长").arg(segmentNumber);
        return false;
    }

    return true;
}

bool loadSegmentPoseFile(const QStringList& data,
                         int expectedEndNum,
                         double positionModeUiRxOffsetRad,
                         TrajectoryPlanner::FileTrajectory& out,
                         QString& errorMessage)
{
    out = TrajectoryPlanner::FileTrajectory{};
    out.preserveImportedTimeStep = true;

    if (expectedEndNum > 0 && expectedEndNum != 1) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹文件只支持单末端位姿，当前界面末端数为 %1").arg(expectedEndNum);
        return false;
    }
    if (data.size() < 2) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹文件缺少段数或分段数据");
        return false;
    }

    int segmentCount = 0;
    int segmentLineBase = 1;
    const bool hasExplicitSegmentCount = parsePositiveIntLine(data.at(1), segmentCount);
    if (hasExplicitSegmentCount) {
        segmentLineBase = 2;
    } else {
        segmentCount = data.size() - 1;
    }

    if (segmentCount <= 0) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹文件至少应包含 1 段轨迹");
        return false;
    }

    const int remainingLineCount = data.size() - segmentLineBase;
    std::vector<SegmentPoseDefinition> segments;
    segments.reserve(segmentCount);

    if (!hasExplicitSegmentCount || remainingLineCount == segmentCount) {
        if (remainingLineCount != segmentCount) {
            errorMessage = QStringLiteral("错误：分段位姿轨迹文件段数与数据行数不一致");
            return false;
        }
        for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            SegmentPoseDefinition segment;
            if (!parseSegmentPoseRow(data.at(segmentLineBase + segmentIndex), segment)) {
                errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段格式不合法，应为 14 个数：起点6维 终点6维 时长(s) 步长(s)").arg(segmentIndex + 1);
                return false;
            }
            segments.push_back(segment);
        }
    } else if (remainingLineCount == segmentCount * 3) {
        for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            SegmentPoseDefinition segment;
            const int lineIndex = segmentLineBase + segmentIndex * 3;
            if (!parseDoubleLine(data.at(lineIndex), 6, segment.startPose) ||
                    !parseDoubleLine(data.at(lineIndex + 1), 6, segment.endPose) ||
                    !parseDurationStepLine(data.at(lineIndex + 2), segment.duration, segment.stepTime)) {
                errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段格式不合法，应为起点6维、终点6维、时长和步长").arg(segmentIndex + 1);
                return false;
            }
            segments.push_back(segment);
        }
    } else if (remainingLineCount == segmentCount * 4) {
        for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            SegmentPoseDefinition segment;
            const int lineIndex = segmentLineBase + segmentIndex * 4;
            if (!parseDoubleLine(data.at(lineIndex), 6, segment.startPose) ||
                    !parseDoubleLine(data.at(lineIndex + 1), 6, segment.endPose) ||
                    !parseSingleDoubleLine(data.at(lineIndex + 2), segment.duration) ||
                    !parseSingleDoubleLine(data.at(lineIndex + 3), segment.stepTime)) {
                errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段格式不合法，应为起点6维、终点6维、时长、步长").arg(segmentIndex + 1);
                return false;
            }
            segments.push_back(segment);
        }
    } else {
        errorMessage = QStringLiteral("错误：分段位姿轨迹文件格式不合法；支持每段 1 行、3 行或 4 行写法");
        return false;
    }

    out.endNum = 1;
    out.offlineTraj.resize(1);
    out.offlineTraj[0].resize(4);
    for (int stateIndex = 0; stateIndex < 4; ++stateIndex) {
        out.offlineTraj[0][stateIndex].resize(6);
    }

    std::vector<double> previousEndPose;
    for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        const SegmentPoseDefinition& segment = segments[segmentIndex];
        const int segmentNumber = segmentIndex + 1;
        if (!validateSegmentPoseDefinition(segment, segmentNumber, errorMessage)) {
            return false;
        }

        const std::vector<double> segmentStartPose =
                segmentFilePoseToPlannerPose(segment.startPose,
                                             positionModeUiRxOffsetRad);
        const std::vector<double> segmentEndPose =
                segmentFilePoseToPlannerPose(segment.endPose,
                                             positionModeUiRxOffsetRad);

        if (!previousEndPose.empty()) {
            for (int dim = 0; dim < 6; ++dim) {
                if (std::fabs(segmentStartPose[dim] - previousEndPose[dim]) > 1e-6) {
                    errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段起点与上一段终点不连续，维度 %2 不一致")
                            .arg(segmentNumber)
                            .arg(dim + 1);
                    return false;
                }
            }
        }

        const TrajectoryPlanner::TrajectoryBlock block =
                TrajectoryPlanner::quintic(segmentStartPose,
                                           {0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0},
                                           segmentEndPose,
                                           {0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0},
                                           segment.duration,
                                           segment.stepTime);
        if (block.size() < 4 || block[0].size() < 6 || block[3].empty() || block[3][0].size() < 2) {
            errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段直线轨迹规划失败").arg(segmentNumber);
            return false;
        }

        const int blockPointCount = static_cast<int>(block[3][0].size());
        for (int stateIndex = 0; stateIndex < 4; ++stateIndex) {
            if (block[stateIndex].size() < 6) {
                errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段轨迹维度不足").arg(segmentNumber);
                return false;
            }
            for (int dim = 0; dim < 6; ++dim) {
                if (static_cast<int>(block[stateIndex][dim].size()) != blockPointCount) {
                    errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段轨迹点数不一致").arg(segmentNumber);
                    return false;
                }
            }
        }

        const int segmentStartIndex = out.pointNum == 0 ? 0 : out.pointNum - 1;
        const int appendStartIndex = out.pointNum == 0 ? 0 : 1;
        const double timeOffset = out.duration;
        for (int pointIndex = appendStartIndex; pointIndex < blockPointCount; ++pointIndex) {
            for (int stateIndex = 0; stateIndex < 4; ++stateIndex) {
                for (int dim = 0; dim < 6; ++dim) {
                    const double value = stateIndex == 3 ?
                                timeOffset + block[3][dim][pointIndex] :
                                block[stateIndex][dim][pointIndex];
                    out.offlineTraj[0][stateIndex][dim].push_back(value);
                }
            }
        }

        out.pointNum = static_cast<int>(out.offlineTraj[0][0][0].size());
        const int segmentEndIndex = out.pointNum - 1;
        if (segmentEndIndex <= segmentStartIndex) {
            errorMessage = QStringLiteral("错误：分段位姿轨迹第 %1 段没有生成有效轨迹点").arg(segmentNumber);
            return false;
        }

        out.segmentRanges.push_back(std::make_pair(segmentStartIndex, segmentEndIndex));
        out.segmentFlags.resize(out.pointNum, 0);
        out.segmentFlags[segmentEndIndex] = 1;
        out.duration = out.offlineTraj[0][3][0].back();
        previousEndPose = segmentEndPose;
    }

    if (out.pointNum < 2 || out.segmentRanges.empty()) {
        errorMessage = QStringLiteral("错误：分段位姿轨迹文件未生成有效轨迹");
        return false;
    }

    out.firstEndStartPose.resize(6);
    for (int dim = 0; dim < 6; ++dim) {
        out.firstEndStartPose[dim] = out.offlineTraj[0][0][dim][0];
    }

    return true;
}

} // namespace


// ========================= 五次多项式轨迹 =========================
TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::quintic(
    const std::vector<double>& startPose,
    const std::vector<double>& startVel,
    const std::vector<double>& startAcc,
    const std::vector<double>& endPose,
    const std::vector<double>& endVel,
    const std::vector<double>& endAcc,
    double duration,
    double stepTime)
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    // 参数合法性检查
    if (startPose.size() != startVel.size() ||
        startPose.size() != startAcc.size() ||
        startPose.size() != endPose.size() ||
        startPose.size() != endVel.size() ||
        startPose.size() != endAcc.size() ||
        std::abs(duration) < 1e-9 ||
        std::abs(stepTime) < 1e-9) {
        return {};
    }

    // 分配维度
    for (auto& block : result) {
        block.resize(startPose.size());
    }

    // 五次多项式系数 c3 c4 c5
    std::vector<double> c3;
    std::vector<double> c4;
    std::vector<double> c5;
    c3.reserve(startPose.size());
    c4.reserve(startPose.size());
    c5.reserve(startPose.size());

    // 逐维计算多项式系数
    for (size_t i = 0; i < startPose.size(); ++i) {
        c3.push_back((20.0 * (endPose[i] - startPose[i]) - (8.0 * endVel[i] + 12.0 * startVel[i]) * duration +
                      (endAcc[i] - 3.0 * startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 3.0)));

        c4.push_back((-30.0 * (endPose[i] - startPose[i]) + (14.0 * endVel[i] + 16.0 * startVel[i]) * duration -
                      (2.0 * endAcc[i] - 3.0 * startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 4.0)));

        c5.push_back((12.0 * (endPose[i] - startPose[i]) - (6.0 * endVel[i] + 6.0 * startVel[i]) * duration +
                      (endAcc[i] - startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 5.0)));
    }

    double currentTime = 0.0;

    // 离散步数
    const int maxStep = static_cast<int>(std::floor(duration / stepTime + 1e-9));

    // 轨迹生成
    for (int step = 0; step <= maxStep; ++step) {
        for (size_t dim = 0; dim < startPose.size(); ++dim) {

            // 时间
            result[3][dim].push_back(currentTime);

            // 位置
            result[0][dim].push_back(startPose[dim] + startVel[dim] * currentTime + startAcc[dim] * std::pow(currentTime, 2.0) +
                                     c3[dim] * std::pow(currentTime, 3.0) +
                                     c4[dim] * std::pow(currentTime, 4.0) +
                                     c5[dim] * std::pow(currentTime, 5.0));

            // 速度
            result[1][dim].push_back(startVel[dim] + 2.0 * startAcc[dim] * currentTime +
                                     3.0 * c3[dim] * std::pow(currentTime, 2.0) +
                                     4.0 * c4[dim] * std::pow(currentTime, 3.0) +
                                     5.0 * c5[dim] * std::pow(currentTime, 4.0));

            // 加速度
            result[2][dim].push_back(2.0 * startAcc[dim] +
                                     6.0 * c3[dim] * currentTime +
                                     12.0 * c4[dim] * std::pow(currentTime, 2.0) +
                                     20.0 * c5[dim] * std::pow(currentTime, 3.0));
        }

        currentTime += stepTime;
    }

    // 末端补点（保证精确到终点）
    if (result[3][0].empty() || std::abs(result[3][0].back() - duration) > 1e-9) {
        for (size_t dim = 0; dim < startPose.size(); ++dim) {
            result[3][dim].push_back(duration);
            result[0][dim].push_back(endPose[dim]);
            result[1][dim].push_back(endVel[dim]);
            result[2][dim].push_back(endAcc[dim]);
        }
    }

    return result;
}

std::vector<double> TrajectoryPlanner::quinticParameterProfile(double startValue,
                                                               double endValue,
                                                               double duration,
                                                               double stepTime)
{
    TrajectoryBlock block = quintic({startValue},
                                    {0.0},
                                    {0.0},
                                    {endValue},
                                    {0.0},
                                    {0.0},
                                    duration,
                                    stepTime);
    if(block.empty() ||
            block.front().empty() ||
            block.front().front().empty()){
        return {};
    }
    return block.front().front();
}

bool TrajectoryPlanner::applySO3QuinticAttitude(
        TrajectoryBlock& trajectory,
        const std::vector<double>& startEuler,
        const std::vector<double>& endEuler,
        QString* errorMessage)
{
    if(!isFiniteEuler(startEuler) || !isFiniteEuler(endEuler)){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("SO(3)姿态规划失败：起点或终点欧拉角无效"));
        return false;
    }
    if(trajectory.size() < kTrajectoryTimeLayer + 1 ||
            trajectory[kTrajectoryPoseLayer].size() < 6 ||
            trajectory[kTrajectoryVelocityLayer].size() < 6 ||
            trajectory[kTrajectoryAccelerationLayer].size() < 6 ||
            trajectory[kTrajectoryTimeLayer].empty()){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("SO(3)姿态规划失败：直线五次轨迹数据不完整"));
        return false;
    }

    const std::vector<double>& timeAxis = trajectory[kTrajectoryTimeLayer][0];
    const int pointCount = static_cast<int>(timeAxis.size());
    if(pointCount < 2){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("SO(3)姿态规划失败：时间轴至少需要两个点"));
        return false;
    }
    for(int dimension = 0; dimension < 6; ++dimension){
        if(static_cast<int>(trajectory[kTrajectoryPoseLayer][dimension].size()) != pointCount ||
                static_cast<int>(trajectory[kTrajectoryVelocityLayer][dimension].size()) != pointCount ||
                static_cast<int>(trajectory[kTrajectoryAccelerationLayer][dimension].size()) != pointCount){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("SO(3)姿态规划失败：轨迹各维度点数不一致"));
            return false;
        }
    }
    for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
        if(!std::isfinite(timeAxis[pointIndex]) ||
                (pointIndex > 0 && timeAxis[pointIndex] <= timeAxis[pointIndex - 1])){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("SO(3)姿态规划失败：时间轴必须为有限且严格递增的真实时间戳"));
            return false;
        }
    }

    const double startTime = timeAxis.front();
    const double duration = timeAxis.back() - startTime;
    if(!std::isfinite(duration) || duration <= 0.0){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("SO(3)姿态规划失败：轨迹总时长必须大于0"));
        return false;
    }

    const Eigen::Vector3d startEulerVector(startEuler[0], startEuler[1], startEuler[2]);
    const Eigen::Vector3d endEulerVector(endEuler[0], endEuler[1], endEuler[2]);
    const Eigen::Matrix3d startRotation = eulerZYXToRotation(startEulerVector);
    const Eigen::Matrix3d endRotation = eulerZYXToRotation(endEulerVector);
    const Eigen::Vector3d rotationVector = so3Log(startRotation.transpose() * endRotation);
    if(rotationVector.norm() < kSO3SmallAngle &&
            (endEulerVector - startEulerVector).norm() > 1e-6){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("SO(3)姿态规划不支持用等价欧拉角表达多圈旋转或分支切换；请改用欧拉角模式"));
        return false;
    }

    trajectory.resize(kSO3AlphaGlobalLayer + 1);
    trajectory[kSO3RotationLayer].assign(9, std::vector<double>(pointCount, 0.0));
    trajectory[kSO3OmegaBodyLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[kSO3AlphaBodyLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[kSO3OmegaGlobalLayer].assign(3, std::vector<double>(pointCount, 0.0));
    trajectory[kSO3AlphaGlobalLayer].assign(3, std::vector<double>(pointCount, 0.0));

    Eigen::Vector3d previousDisplayEuler = startEulerVector;
    for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
        const double tau = (timeAxis[pointIndex] - startTime) / duration;
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double s = 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2;
        const double sDot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau3 * tau) / duration;
        const double sDDot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) /
                (duration * duration);

        const Eigen::Matrix3d rotation = startRotation * so3Exp(rotationVector * s);
        const Eigen::Vector3d omegaBody = rotationVector * sDot;
        const Eigen::Vector3d alphaBody = rotationVector * sDDot;
        const Eigen::Vector3d omegaGlobal = rotation * omegaBody;
        const Eigen::Vector3d alphaGlobal = rotation * alphaBody;
        const Eigen::Vector3d displayEuler = pointIndex == 0 ?
                    startEulerVector : rotationToEulerZYXNear(rotation, previousDisplayEuler);
        previousDisplayEuler = displayEuler;

        for(int row = 0; row < 3; ++row){
            for(int column = 0; column < 3; ++column){
                trajectory[kSO3RotationLayer][3 * row + column][pointIndex] = rotation(row, column);
            }
            trajectory[kSO3OmegaBodyLayer][row][pointIndex] = omegaBody(row);
            trajectory[kSO3AlphaBodyLayer][row][pointIndex] = alphaBody(row);
            trajectory[kSO3OmegaGlobalLayer][row][pointIndex] = omegaGlobal(row);
            trajectory[kSO3AlphaGlobalLayer][row][pointIndex] = alphaGlobal(row);

            // 与 MATLAB 输出保持一致：遗留速度/加速度字段在 SO(3) 模式下
            // 记录全局系角速度/角加速度，而不是欧拉角导数。
            trajectory[kTrajectoryVelocityLayer][row + 3][pointIndex] = omegaGlobal(row);
            trajectory[kTrajectoryAccelerationLayer][row + 3][pointIndex] = alphaGlobal(row);
            trajectory[kTrajectoryPoseLayer][row + 3][pointIndex] = displayEuler(row);
        }
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool TrajectoryPlanner::applySO3ConstantAttitude(
        TrajectoryBlock& trajectory,
        const std::vector<double>& euler,
        QString* errorMessage)
{
    const QString context = QStringLiteral("SO(3)恒定姿态规划失败");
    if(!isFiniteEuler(euler)){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：欧拉角无效").arg(context));
        return false;
    }
    std::vector<double> timeAxis;
    if(!prepareSO3Trajectory(trajectory, timeAxis, errorMessage, context)){
        return false;
    }

    const Eigen::Vector3d displayEuler(euler[0], euler[1], euler[2]);
    const Eigen::Matrix3d rotation = eulerZYXToRotation(displayEuler);
    for(int pointIndex = 0; pointIndex < static_cast<int>(timeAxis.size()); ++pointIndex){
        writeSO3TrajectorySample(trajectory, pointIndex, rotation, displayEuler,
                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool TrajectoryPlanner::applySO3EndpointProgressAttitude(
        TrajectoryBlock& trajectory,
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        QString* errorMessage)
{
    const QString context = QStringLiteral("SO(3)端点进度姿态规划失败");
    if(startPose.size() < 6 || endPose.size() < 6){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：起点或终点位姿维度不足").arg(context));
        return false;
    }

    std::vector<double> startEuler(startPose.begin() + 3, startPose.begin() + 6);
    std::vector<double> endEuler(endPose.begin() + 3, endPose.begin() + 6);
    Eigen::Vector3d startEulerVector;
    Eigen::Matrix3d startRotation;
    Eigen::Vector3d rotationVector;
    if(!buildSO3EndpointData(startEuler, endEuler, startEulerVector, startRotation,
                             rotationVector, errorMessage, context)){
        return false;
    }

    Eigen::Matrix<double, 6, 1> direction;
    double distanceSquared = 0.0;
    for(int dimension = 0; dimension < 6; ++dimension){
        if(!std::isfinite(startPose[dimension]) || !std::isfinite(endPose[dimension])){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("%1：起点或终点位姿包含非有限值").arg(context));
            return false;
        }
        direction(dimension) = endPose[dimension] - startPose[dimension];
        distanceSquared += direction(dimension) * direction(dimension);
    }
    if(distanceSquared < 1e-12){
        return applySO3ConstantAttitude(trajectory, startEuler, errorMessage);
    }

    std::vector<double> timeAxis;
    if(!prepareSO3Trajectory(trajectory, timeAxis, errorMessage, context)){
        return false;
    }

    Eigen::Vector3d previousDisplayEuler = startEulerVector;
    for(int pointIndex = 0; pointIndex < static_cast<int>(timeAxis.size()); ++pointIndex){
        Eigen::Matrix<double, 6, 1> legacyPose;
        Eigen::Matrix<double, 6, 1> legacyVelocity;
        Eigen::Matrix<double, 6, 1> legacyAcceleration;
        for(int dimension = 0; dimension < 6; ++dimension){
            legacyPose(dimension) = trajectory[kTrajectoryPoseLayer][dimension][pointIndex];
            legacyVelocity(dimension) = trajectory[kTrajectoryVelocityLayer][dimension][pointIndex];
            legacyAcceleration(dimension) = trajectory[kTrajectoryAccelerationLayer][dimension][pointIndex];
            if(!std::isfinite(legacyPose(dimension)) ||
                    !std::isfinite(legacyVelocity(dimension)) ||
                    !std::isfinite(legacyAcceleration(dimension))){
                setSO3PlanningError(errorMessage,
                                    QStringLiteral("%1：原轨迹包含非有限值").arg(context));
                return false;
            }
        }
        Eigen::Matrix<double, 6, 1> start;
        for(int dimension = 0; dimension < 6; ++dimension){
            start(dimension) = startPose[dimension];
        }
        const double progress = direction.dot(legacyPose - start) / distanceSquared;
        const double progressVelocity = direction.dot(legacyVelocity) / distanceSquared;
        const double progressAcceleration = direction.dot(legacyAcceleration) / distanceSquared;
        const Eigen::Matrix3d rotation = startRotation * so3Exp(rotationVector * progress);
        const Eigen::Vector3d omegaBody = rotationVector * progressVelocity;
        const Eigen::Vector3d alphaBody = rotationVector * progressAcceleration;
        const Eigen::Vector3d displayEuler =
                pointIndex == 0 && std::abs(progress) < 1e-9 ?
                    startEulerVector : rotationToEulerZYXNear(rotation, previousDisplayEuler);
        previousDisplayEuler = displayEuler;
        writeSO3TrajectorySample(trajectory, pointIndex, rotation, displayEuler,
                                 omegaBody, alphaBody);
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool TrajectoryPlanner::applySO3CubicHermiteAttitude(
        TrajectoryBlock& trajectory,
        const std::vector<double>& startEuler,
        const std::vector<double>& endEuler,
        QString* errorMessage)
{
    const QString context = QStringLiteral("SO(3)三次Hermite姿态规划失败");
    Eigen::Vector3d startEulerVector;
    Eigen::Matrix3d startRotation;
    Eigen::Vector3d finalRotationVector;
    if(!buildSO3EndpointData(startEuler, endEuler, startEulerVector, startRotation,
                             finalRotationVector, errorMessage, context)){
        return false;
    }

    std::vector<double> timeAxis;
    if(!prepareSO3Trajectory(trajectory, timeAxis, errorMessage, context)){
        return false;
    }
    const int pointCount = static_cast<int>(timeAxis.size());
    const double startTime = timeAxis.front();
    const double duration = timeAxis.back() - startTime;
    if(duration <= 0.0 || !std::isfinite(duration)){
        setSO3PlanningError(errorMessage,
                            QStringLiteral("%1：轨迹总时长必须大于0").arg(context));
        return false;
    }

    Eigen::Vector3d endEulerVector(endEuler[0], endEuler[1], endEuler[2]);
    Eigen::Vector3d startEulerVelocity;
    Eigen::Vector3d endEulerVelocity;
    for(int row = 0; row < 3; ++row){
        startEulerVelocity(row) = trajectory[kTrajectoryVelocityLayer][row + 3][0];
        endEulerVelocity(row) = trajectory[kTrajectoryVelocityLayer][row + 3][pointCount - 1];
        if(!std::isfinite(startEulerVelocity(row)) || !std::isfinite(endEulerVelocity(row))){
            setSO3PlanningError(errorMessage,
                                QStringLiteral("%1：起止欧拉角速度无效").arg(context));
            return false;
        }
    }
    const Eigen::Vector3d startBodyOmega =
            eulerZYXRatesToBodyOmega(startEulerVector, startEulerVelocity);
    const Eigen::Vector3d endBodyOmega =
            eulerZYXRatesToBodyOmega(endEulerVector, endEulerVelocity);
    const Eigen::Vector3d rotationVectorStartRate = startBodyOmega;
    const Eigen::Vector3d rotationVectorEndRate =
            so3RightJacobianInverse(finalRotationVector) * endBodyOmega;

    Eigen::Vector3d previousDisplayEuler = startEulerVector;
    for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
        const double tau = (timeAxis[pointIndex] - startTime) / duration;
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double h10 = tau3 - 2.0 * tau2 + tau;
        const double h01 = -2.0 * tau3 + 3.0 * tau2;
        const double h11 = tau3 - tau2;
        const double h10Dot = 3.0 * tau2 - 4.0 * tau + 1.0;
        const double h01Dot = -6.0 * tau2 + 6.0 * tau;
        const double h11Dot = 3.0 * tau2 - 2.0 * tau;
        const double h10DDot = 6.0 * tau - 4.0;
        const double h01DDot = -12.0 * tau + 6.0;
        const double h11DDot = 6.0 * tau - 2.0;

        const Eigen::Vector3d rotationVector =
                h10 * duration * rotationVectorStartRate +
                h01 * finalRotationVector +
                h11 * duration * rotationVectorEndRate;
        const Eigen::Vector3d rotationVectorRate =
                h10Dot * rotationVectorStartRate +
                h01Dot * finalRotationVector / duration +
                h11Dot * rotationVectorEndRate;
        const Eigen::Vector3d rotationVectorAcceleration =
                h10DDot * rotationVectorStartRate / duration +
                h01DDot * finalRotationVector / (duration * duration) +
                h11DDot * rotationVectorEndRate / duration;
        const Eigen::Matrix3d rotation = startRotation * so3Exp(rotationVector);
        const Eigen::Matrix3d rightJacobian = so3RightJacobian(rotationVector);
        const Eigen::Vector3d omegaBody = rightJacobian * rotationVectorRate;
        const Eigen::Vector3d alphaBody =
                so3RightJacobianDerivative(rotationVector, rotationVectorRate) *
                rotationVectorRate + rightJacobian * rotationVectorAcceleration;
        const Eigen::Vector3d displayEuler = pointIndex == 0 ?
                    startEulerVector : rotationToEulerZYXNear(rotation, previousDisplayEuler);
        previousDisplayEuler = displayEuler;
        writeSO3TrajectorySample(trajectory, pointIndex, rotation, displayEuler,
                                 omegaBody, alphaBody);
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool TrajectoryPlanner::applySO3EulerWaveformAttitude(TrajectoryBlock& trajectory,
                                                       QString* errorMessage)
{
    const QString context = QStringLiteral("SO(3)欧拉波形兼容规划失败");
    std::vector<double> timeAxis;
    if(!prepareSO3Trajectory(trajectory, timeAxis, errorMessage, context)){
        return false;
    }

    for(int pointIndex = 0; pointIndex < static_cast<int>(timeAxis.size()); ++pointIndex){
        Eigen::Vector3d euler;
        Eigen::Vector3d eulerRate;
        Eigen::Vector3d eulerAcceleration;
        for(int row = 0; row < 3; ++row){
            euler(row) = trajectory[kTrajectoryPoseLayer][row + 3][pointIndex];
            eulerRate(row) = trajectory[kTrajectoryVelocityLayer][row + 3][pointIndex];
            eulerAcceleration(row) = trajectory[kTrajectoryAccelerationLayer][row + 3][pointIndex];
            if(!std::isfinite(euler(row)) || !std::isfinite(eulerRate(row)) ||
                    !std::isfinite(eulerAcceleration(row))){
                setSO3PlanningError(errorMessage,
                                    QStringLiteral("%1：欧拉角波形包含非有限值").arg(context));
                return false;
            }
        }
        const Eigen::Matrix3d rotation = eulerZYXToRotation(euler);
        const Eigen::Vector3d omegaBody = eulerZYXRatesToBodyOmega(euler, eulerRate);
        const Eigen::Vector3d alphaBody =
                eulerZYXAccelerationsToBodyAlpha(euler, eulerRate, eulerAcceleration);
        writeSO3TrajectorySample(trajectory, pointIndex, rotation, euler,
                                 omegaBody, alphaBody);
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool TrajectoryPlanner::hasSO3AttitudeData(const TrajectoryBlock& trajectory)
{
    if(trajectory.size() <= kSO3AlphaGlobalLayer ||
            trajectory[kTrajectoryPoseLayer].empty() ||
            trajectory[kTrajectoryPoseLayer][0].empty()){
        return false;
    }
    const std::size_t pointCount = trajectory[kTrajectoryPoseLayer][0].size();
    const auto hasLayerShape = [pointCount](const std::vector<std::vector<double>>& layer,
                                            std::size_t dimensionCount) {
        if(layer.size() != dimensionCount){
            return false;
        }
        for(const std::vector<double>& row : layer){
            if(row.size() != pointCount){
                return false;
            }
        }
        return true;
    };
    return hasLayerShape(trajectory[kSO3RotationLayer], 9) &&
            hasLayerShape(trajectory[kSO3OmegaBodyLayer], 3) &&
            hasLayerShape(trajectory[kSO3AlphaBodyLayer], 3) &&
            hasLayerShape(trajectory[kSO3OmegaGlobalLayer], 3) &&
            hasLayerShape(trajectory[kSO3AlphaGlobalLayer], 3);
}

bool TrajectoryPlanner::readSO3AttitudeSample(const TrajectoryBlock& trajectory,
                                               int pointIndex,
                                               SO3AttitudeSample& sample)
{
    if(!hasSO3AttitudeData(trajectory) || pointIndex < 0 ||
            pointIndex >= static_cast<int>(trajectory[kTrajectoryPoseLayer][0].size())){
        return false;
    }

    for(int row = 0; row < 3; ++row){
        for(int column = 0; column < 3; ++column){
            const double value = trajectory[kSO3RotationLayer][3 * row + column][pointIndex];
            if(!std::isfinite(value)){
                return false;
            }
            sample.rotationGlobalFromBody(row, column) = value;
        }
        const double omegaBody = trajectory[kSO3OmegaBodyLayer][row][pointIndex];
        const double alphaBody = trajectory[kSO3AlphaBodyLayer][row][pointIndex];
        const double omegaGlobal = trajectory[kSO3OmegaGlobalLayer][row][pointIndex];
        const double alphaGlobal = trajectory[kSO3AlphaGlobalLayer][row][pointIndex];
        if(!std::isfinite(omegaBody) || !std::isfinite(alphaBody) ||
                !std::isfinite(omegaGlobal) || !std::isfinite(alphaGlobal)){
            return false;
        }
        sample.omegaBody(row) = omegaBody;
        sample.alphaBody(row) = alphaBody;
        sample.omegaGlobal(row) = omegaGlobal;
        sample.alphaGlobal(row) = alphaGlobal;
    }
    return sample.rotationGlobalFromBody.allFinite() &&
            std::abs(sample.rotationGlobalFromBody.determinant() - 1.0) < 1e-6 &&
            (sample.rotationGlobalFromBody.transpose() * sample.rotationGlobalFromBody -
             Eigen::Matrix3d::Identity()).norm() < 1e-6;
}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::circular(
        const std::vector<double>& startPose,        
        const std::vector<double>& center, // [x,y,z]
        const double radius,
        const double duration,
        const int direction,
        double stepTime,
        double startLambda,
        double endLambda)
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    // ===== 参数检查 =====
    if (startPose.size() != 6 ||
        center.size() != 3 ||
        radius <= 0 ||
        duration <= 0 ||
        std::abs(stepTime) < 1e-9 ||
        !std::isfinite(startLambda) ||
        !std::isfinite(endLambda) ||
        endLambda < startLambda)
    {
        return {};
    }

    // 分配维度（6维：xyz + rpy）
    for (auto& block : result) {
        block.resize(6);
    }

    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    // ===== 初始角度（由 startPose 决定）=====
    double dx0 = startPose[0] - center[0];
    double dy0 = startPose[1] - center[1];
    double theta0 = atan2(dy0, dx0);

    const TrajectoryBlock lambdaBlock = quintic({startLambda},
                                                {0.0},
                                                {0.0},
                                                {endLambda},
                                                {0.0},
                                                {0.0},
                                                duration,
                                                stepTime);
    if(lambdaBlock.empty() ||
            lambdaBlock.size() < 4 ||
            lambdaBlock[0].empty() ||
            lambdaBlock[1].empty() ||
            lambdaBlock[2].empty() ||
            lambdaBlock[3].empty() ||
            lambdaBlock[0][0].empty() ||
            lambdaBlock[0][0].size() != lambdaBlock[1][0].size() ||
            lambdaBlock[0][0].size() != lambdaBlock[2][0].size() ||
            lambdaBlock[0][0].size() != lambdaBlock[3][0].size()){
        return {};
    }

    const std::vector<double>& lambda = lambdaBlock[0][0];
    const std::vector<double>& lambdaVel = lambdaBlock[1][0];
    const std::vector<double>& lambdaAcc = lambdaBlock[2][0];
    const std::vector<double>& timeAxis = lambdaBlock[3][0];

    for (size_t step = 0; step < lambda.size(); ++step)
    {
        double theta = theta0 + direction * kTwoPi * lambda[step];
        double thetaVel = direction * kTwoPi * lambdaVel[step];
        double thetaAcc = direction * kTwoPi * lambdaAcc[step];

        double cos_t = cos(theta);
        double sin_t = sin(theta);

        // ===== 位置 =====
        double x = center[0] + radius * cos_t;
        double y = center[1] + radius * sin_t;
        double z = startPose[2]; // 保持高度

        // ===== 速度 =====
        double vx = -radius * sin_t * thetaVel;
        double vy =  radius * cos_t * thetaVel;
        double vz = 0.0;

        // ===== 加速度 =====
        double ax = -radius * cos_t * thetaVel * thetaVel -
                radius * sin_t * thetaAcc;
        double ay = -radius * sin_t * thetaVel * thetaVel +
                radius * cos_t * thetaAcc;
        double az = 0.0;

        // ===== 写入 =====
        // 位置
        result[0][0].push_back(x);
        result[0][1].push_back(y);
        result[0][2].push_back(z);

        // 姿态（保持不变）
        result[0][3].push_back(startPose[3]);
        result[0][4].push_back(startPose[4]);
        result[0][5].push_back(startPose[5]);

        // 速度
        result[1][0].push_back(vx);
        result[1][1].push_back(vy);
        result[1][2].push_back(vz);

        result[1][3].push_back(0.0);
        result[1][4].push_back(0.0);
        result[1][5].push_back(0.0);

        // 加速度
        result[2][0].push_back(ax);
        result[2][1].push_back(ay);
        result[2][2].push_back(az);

        result[2][3].push_back(0.0);
        result[2][4].push_back(0.0);
        result[2][5].push_back(0.0);

        // 时间（每个维度都一样）
        for (int d = 0; d < 6; ++d) {
            result[3][d].push_back(timeAxis[step]);
        }

    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::scurve(
    const std::vector<double>& startPose,
    const std::vector<double>& endPose,
    const double vmax,
    const double acc,
    const double dec,
    double stepTime)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != endPose.size() ||
        vmax <= 0 || acc <= 0 || dec <= 0 ||
        stepTime <= 1e-9) {
        return {};
    }

    int dim = startPose.size();

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 计算方向 & 距离 =====
    std::vector<double> dir(dim);
    double dist = 0.0;

    for (int i = 0; i < dim; ++i) {
        dir[i] = endPose[i] - startPose[i];
        dist += dir[i] * dir[i];
    }

    dist = std::sqrt(dist);
    if (dist < 1e-9) return {};

    for (int i = 0; i < dim; ++i) {
        dir[i] /= dist;
    }

    // ===== 时间参数 =====
    double ta = vmax / acc;
    double td = vmax / dec;

    double da = 0.5 * acc * ta * ta;
    double dd = 0.5 * dec * td * td;

    double tc = 0.0;
    double profileVmax = vmax;

    // ===== 判断是否能达到 vmax =====
    if (dist > da + dd) {
        tc = (dist - da - dd) / profileVmax;
    } else {
        // Triangular velocity profile: no cruise phase.
        profileVmax = std::sqrt((2.0 * dist * acc * dec) / (acc + dec));
        ta = profileVmax / acc;
        td = profileVmax / dec;
        da = 0.5 * acc * ta * ta;
        dd = 0.5 * dec * td * td;
        tc = 0.0;
    }

    double totalTime = ta + tc + td;

    // ===== 轨迹生成 =====
    double time = 0.0;

    while (time <= totalTime + 1e-9) {

        double s = 0.0;
        double v = 0.0;
        double a = 0.0;

        if (time < ta) {
            // 加速段
            a = acc;
            v = acc * time;
            s = 0.5 * acc * time * time;
        }
        else if (time < ta + tc) {
            // 匀速段
            a = 0.0;
            v = profileVmax;
            s = da + profileVmax * (time - ta);
        }
        else {
            // 减速段
            double t2 = time - ta - tc;
            a = -dec;
            v = profileVmax - dec * t2;
            s = da + profileVmax * tc + profileVmax * t2 - 0.5 * dec * t2 * t2;
        }

        // ===== 写入各维 =====
        for (int i = 0; i < dim; ++i) {
            result[0][i].push_back(startPose[i] + dir[i] * s);
            result[1][i].push_back(dir[i] * v);
            result[2][i].push_back(dir[i] * a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    // ===== 末点修正 =====
    for (int i = 0; i < dim; ++i) {
        result[0][i].back() = endPose[i];
        result[1][i].back() = 0.0;
        result[2][i].back() = 0.0;
        result[3][i].back() = totalTime;
    }

    return result;


}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::eightShape(
        const std::vector<double>& startPose,
        const std::vector<double>& normal,
        const double R,
        const double range,
        double duration,
        double stepTime,
        double startLambda,
        double endLambda)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != 6 ||
        normal.size() != 3 ||
        R <= 0 ||
        duration <= 1e-9 ||
        stepTime <= 1e-9 ||
        !std::isfinite(startLambda) ||
        !std::isfinite(endLambda) ||
        endLambda < startLambda)
    {
        return {};
    }

    int dim = 6;

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 初始位姿 =====
    std::vector<double> p0(3);
    std::vector<double> eul0(3);

    for (int i = 0; i < 3; ++i) {
        p0[i] = startPose[i];
        eul0[i] = startPose[i + 3];
    }

    // ===== 法向量单位化 =====
    double norm = std::sqrt(normal[0]*normal[0] +
                            normal[1]*normal[1] +
                            normal[2]*normal[2]);

    if (norm < 1e-9) return {};

    std::vector<double> n_vec = {
        normal[0]/norm,
        normal[1]/norm,
        normal[2]/norm
    };

    // ===== 构造平面基 u v =====
    std::vector<double> tmp =
        (std::fabs(n_vec[0]) < 0.9) ?
        std::vector<double>{1,0,0} :
        std::vector<double>{0,1,0};

    // u = n × tmp
    std::vector<double> u = {
        n_vec[1]*tmp[2] - n_vec[2]*tmp[1],
        n_vec[2]*tmp[0] - n_vec[0]*tmp[2],
        n_vec[0]*tmp[1] - n_vec[1]*tmp[0]
    };

    double u_norm = std::sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    for (auto &v : u) v /= u_norm;

    // v = n × u
    std::vector<double> v = {
        n_vec[1]*u[2] - n_vec[2]*u[1],
        n_vec[2]*u[0] - n_vec[0]*u[2],
        n_vec[0]*u[1] - n_vec[1]*u[0]
    };

    // ===== 轨迹生成 =====
    const TrajectoryBlock lambdaBlock = quintic({startLambda},
                                                {0.0},
                                                {0.0},
                                                {endLambda},
                                                {0.0},
                                                {0.0},
                                                duration,
                                                stepTime);
    if(lambdaBlock.empty() ||
            lambdaBlock.size() < 4 ||
            lambdaBlock[0].empty() ||
            lambdaBlock[1].empty() ||
            lambdaBlock[2].empty() ||
            lambdaBlock[3].empty() ||
            lambdaBlock[0][0].empty() ||
            lambdaBlock[0][0].size() != lambdaBlock[1][0].size() ||
            lambdaBlock[0][0].size() != lambdaBlock[2][0].size() ||
            lambdaBlock[0][0].size() != lambdaBlock[3][0].size()){
        return {};
    }

    const std::vector<double>& lambda = lambdaBlock[0][0];
    const std::vector<double>& lambdaVel = lambdaBlock[1][0];
    const std::vector<double>& lambdaAcc = lambdaBlock[2][0];
    const std::vector<double>& timeAxis = lambdaBlock[3][0];

    for (size_t pointIndex = 0; pointIndex < lambda.size(); ++pointIndex)
    {
        double theta   = 2*M_PI*lambda[pointIndex];
        double dtheta  = 2*M_PI*lambdaVel[pointIndex];
        double ddtheta = 2*M_PI*lambdaAcc[pointIndex];

        // ===== 五次时间参数 =====

        // ===== 8字形 =====
        double x  = R*sin(theta);
        double y  = R*sin(theta)*cos(theta);

        double dx = R*cos(theta)*dtheta;
        double dy = R*cos(2*theta)*dtheta;

        double ddx = -R*sin(theta)*dtheta*dtheta + R*cos(theta)*ddtheta;
        double ddy = -2*R*sin(2*theta)*dtheta*dtheta + R*cos(2*theta)*ddtheta;

        // ===== 空间映射 =====
        std::vector<double> pos(3), vel(3), acc(3);

        for (int i = 0; i < 3; ++i) {
            pos[i] = p0[i] + x*u[i] + y*v[i];
            vel[i] = dx*u[i] + dy*v[i];
            acc[i] = ddx*u[i] + ddy*v[i];
        }

        // ===== 姿态 =====
        double roll  = eul0[0] + range*sin(theta);
        double pitch = eul0[1] + range*cos(theta);
        double yaw   = 0;

        double droll  = range*cos(theta)*dtheta;
        double dpitch = -range*sin(theta)*dtheta;
        double dyaw   = 0;

        double ddroll  = -range*sin(theta)*dtheta*dtheta + range*cos(theta)*ddtheta;
        double ddpitch = -range*cos(theta)*dtheta*dtheta - range*sin(theta)*ddtheta;
        double ddyaw   = 0;

        // ===== 写入 =====
        for (int i = 0; i < 3; ++i) {
            result[0][i].push_back(pos[i]);
            result[1][i].push_back(vel[i]);
            result[2][i].push_back(acc[i]);
            result[3][i].push_back(timeAxis[pointIndex]);
        }

        result[0][3].push_back(roll);
        result[0][4].push_back(pitch);
        result[0][5].push_back(yaw);

        result[1][3].push_back(droll);
        result[1][4].push_back(dpitch);
        result[1][5].push_back(dyaw);

        result[2][3].push_back(ddroll);
        result[2][4].push_back(ddpitch);
        result[2][5].push_back(ddyaw);

        result[3][3].push_back(timeAxis[pointIndex]);
        result[3][4].push_back(timeAxis[pointIndex]);
        result[3][5].push_back(timeAxis[pointIndex]);

    }

    return result;
}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::sineshape(
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double A,
        const double w,
        const double phi,
        double duration,
        double stepTime)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != endPose.size() ||
        w <= 0 ||
        duration <= 1e-9 ||
        stepTime <= 1e-9)
    {
        return {};
    }

    int dim = startPose.size();

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 方向单位向量 =====
    std::vector<double> dir(dim);
    double dist = 0.0;

    for (int i = 0; i < dim; ++i) {
        dir[i] = endPose[i] - startPose[i];
        dist += dir[i] * dir[i];
    }

    dist = std::sqrt(dist);
    if (dist < 1e-9) return {};

    for (int i = 0; i < dim; ++i) {
        dir[i] /= dist;
    }

    // ===== 状态变量 =====
    double time = 0.0;
    double v = 0.0;
    double s = 0.0;

    // ===== 主循环 =====
    while (time <= duration + 1e-9)
    {
        double a = A * std::sin(w * time + phi);

        // 数值积分（欧拉）
        v += a * stepTime;
        s += v * stepTime;

        for (int i = 0; i < dim; ++i) {
            result[0][i].push_back(startPose[i] + dir[i] * s);
            result[1][i].push_back(dir[i] * v);
            result[2][i].push_back(dir[i] * a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::stepAccel(
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& dir,    // 运动方向（单位向量）

        const double a_before,            // 阶跃前加速度
        const double a_after,             // 阶跃后加速度

        const double t_step,              // 阶跃发生时刻
        double stepTime)            // 采样时间);
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    int dim = startPose.size();

    // ===== 参数检查 =====
    if (dim < 3 ||
        dir.size() != 3 ||
        stepTime <= 1e-9 ||
        t_step < 0 ||
        std::abs(a_after) <= 1e-9)
    {
        return {};
    }
    const double dirNorm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (dirNorm <= 1e-9) {
        return {};
    }

    // ===== 分配空间 =====
    for (auto& block : result)
        block.resize(dim);

    // ===== 时间推进 =====
    double time = 0.0;

    double v_scalar = 0.0; // 标量速度（沿dir）
    double s_scalar = 0.0; // 标量位移
    double duration = a_before * t_step / a_after + t_step;


    while (time <= duration + 1e-9)
    {
        // ===== 判断当前加速度 =====
        double a_scalar = (time < t_step) ? a_before : a_after;

        // ===== 写入各维 =====
        for (int i = 0; i < dim; ++i)
        {
            double pos = startPose[i];
            double vel = 0.0;
            double acc = 0.0;
            if (i < 3) {
                pos += dir[i] * s_scalar;
                vel = dir[i] * v_scalar;
                acc = dir[i] * a_scalar;
            }

            result[0][i].push_back(pos);
            result[1][i].push_back(vel);
            result[2][i].push_back(acc);
            result[3][i].push_back(time);
        }

        if (time + stepTime > duration + 1e-9) {
            break;
        }

        // Record the current sample before integration so the first point
        // exactly matches startPose.
        v_scalar += a_scalar * stepTime;
        s_scalar += v_scalar * stepTime;
        time += stepTime;
    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::cubicline(
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& startVel,     // 初始速度
        const std::vector<double>& endPose,     // 初始位置
        const std::vector<double>& endVel,     // 初始速度
        double duration,            // 总时长
        double stepTime)            // 采样时间);
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    int dim = startPose.size();

    // ===== 参数检查 =====
    if (dim == 0 ||
        startVel.size() != dim ||
        endPose.size() != dim ||
        endVel.size() != dim ||
        duration <= 1e-9 ||
        stepTime <= 1e-9)
    {
        return {};
    }

    // ===== 分配空间 =====
    for (auto& block : result)
        block.resize(dim);

    // ===== 系数 =====
    std::vector<double> a0 = startPose;
    std::vector<double> a1 = startVel;
    std::vector<double> a2(dim);
    std::vector<double> a3(dim);

    for (int i = 0; i < dim; ++i)
    {
        a2[i] = (3*(endPose[i] - startPose[i]) - (2*startVel[i] + endVel[i]) * duration)
                / (duration * duration);

        a3[i] = (2*(startPose[i] - endPose[i]) + (startVel[i] + endVel[i]) * duration)
                / (duration * duration * duration);
    }

    // ===== 轨迹生成 =====
    double time = 0.0;
    const int maxStep = static_cast<int>(std::floor(duration / stepTime + 1e-9));

    for (int step = 0; step <= maxStep; ++step)
    {
        for (int i = 0; i < dim; ++i)
        {
            double t = time;

            double p = a0[i] + a1[i]*t + a2[i]*t*t + a3[i]*t*t*t;
            double v = a1[i] + 2*a2[i]*t + 3*a3[i]*t*t;
            double a = 2*a2[i] + 6*a3[i]*t;

            result[0][i].push_back(p);
            result[1][i].push_back(v);
            result[2][i].push_back(a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    // ===== 终点补偿（防止浮点误差）=====
    if (result[3][0].empty() || std::abs(result[3][0].back() - duration) > 1e-9)
    {
        for (int i = 0; i < dim; ++i)
        {
            result[3][i].push_back(duration);
            result[0][i].push_back(endPose[i]);
            result[1][i].push_back(endVel[i]);

            // 加速度用公式算（更一致）
            double a_end = 2*a2[i] + 6*a3[i]*duration;
            result[2][i].push_back(a_end);
        }
    }

    return result;
}

// ========================= 多末端轨迹拼接 =========================
TrajectoryPlanner::MultiEndTrajectory
TrajectoryPlanner::buildLineTrajectory(const std::vector<EndpointRequest>& requests, const TrajectoryPlanner::TrajType& type)
{
    MultiEndTrajectory result;
    result.reserve(requests.size());
    switch (type)
    {
    case TrajectoryPlanner::TrajType::Quintic:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = quintic(
            request.q_param.startPose,
            request.q_param.startVel,
            request.q_param.startAcc,
            request.q_param.endPose,
            request.q_param.endVel,
            request.q_param.endAcc,
            request.q_param.duration,
            request.q_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Circular:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = circular(
            request.cir_param.startPose,
            request.cir_param.center,
            request.cir_param.radius,
            request.cir_param.duration,
            request.cir_param.direction,
            request.cir_param.stepTime,
            request.cir_param.startLambda,
            request.cir_param.endLambda);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::SCurve:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = scurve(
            request.s_param.startPose,
            request.s_param.endPose,
            request.s_param.vmax,
            request.s_param.acc,
            request.s_param.dec,
            request.s_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::EightShape:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = eightShape(
            request.e_param.startPose,
            request.e_param.normal,
            request.e_param.R,
            request.e_param.range,
            request.e_param.duration,
            request.e_param.stepTime,
            request.e_param.startLambda,
            request.e_param.endLambda);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Cubic:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = cubicline(
            request.cub_param.startPose,
            request.cub_param.startVel,
            request.cub_param.endPose,
            request.cub_param.endVel,
            request.cub_param.duration,
            request.cub_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::StepAccel:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = stepAccel(
            request.stepAccel_param.startPose,
            request.stepAccel_param.dir,
            request.stepAccel_param.a_before,
            request.stepAccel_param.a_after,
            request.stepAccel_param.t_step,
            request.stepAccel_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Sine:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = sineshape(
            request.sin_param.startPose,
            request.sin_param.endPose,
            request.sin_param.A,
            request.sin_param.w,
            request.sin_param.phi,
            request.sin_param.duration,
            request.sin_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    default:
    return {};
    }
    // 每个末端独立生成轨迹

    return result;
}


// ========================= 离线轨迹直接装载 =========================
TrajectoryPlanner::MultiEndTrajectory
TrajectoryPlanner::buildSingleEndOfflinePoseTrajectory(
    const std::vector<std::vector<double>>& poseRows)
{
    if (poseRows.size() < 6) {
        return {};
    }

    MultiEndTrajectory result(1);
    result[0].resize(4);
    result[0][0].resize(6);

    // 直接填充位置（无速度加速度）
    for (int dim = 0; dim < 6; ++dim) {
        result[0][0][dim] = poseRows[dim];
    }

    return result;
}


// ========================= 插值 =========================
std::vector<double>
TrajectoryPlanner::interpolatePointTrajectory(
    const std::vector<std::vector<double>>& positionTraj,
    const std::vector<double>& timeStamp,
    double targetTime)
{
    QString errorMessage;

    if (!validatePointTrajectory(positionTraj, timeStamp, errorMessage)) {
        return {};
    }

    // 边界处理
    if (targetTime <= timeStamp.front()) return positionTraj.front();
    if (targetTime >= timeStamp.back())  return positionTraj.back();

    // 找区间
    auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), targetTime);
    size_t upperIndex = std::distance(timeStamp.begin(), upperIt);
    size_t lowerIndex = upperIndex - 1;

    double t0 = timeStamp[lowerIndex];
    double t1 = timeStamp[upperIndex];
    double ratio = (targetTime - t0) / (t1 - t0);

    std::vector<double> result(positionTraj.front().size(), 0.0);

    // 线性插值
    for (size_t dim = 0; dim < result.size(); ++dim) {
        result[dim] = positionTraj[lowerIndex][dim] +
                      ratio * (positionTraj[upperIndex][dim] - positionTraj[lowerIndex][dim]);
    }

    return result;
}


// ========================= 速度估计 =========================
std::vector<double>
TrajectoryPlanner::estimatePointTrajectoryVelocity(
    const std::vector<std::vector<double>>& positionTraj,
    const std::vector<double>& timeStamp,
    double targetTime)
{
    QString errorMessage;

    if (!validatePointTrajectory(positionTraj, timeStamp, errorMessage)) {
        return {};
    }

    double t = clampTime(targetTime, timeStamp.front(), timeStamp.back());

    auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), t);
    size_t upperIndex = std::distance(timeStamp.begin(), upperIt);

    if (upperIndex == 0) upperIndex = 1;
    if (upperIndex >= timeStamp.size()) upperIndex = timeStamp.size() - 1;

    size_t lowerIndex = upperIndex - 1;

    double dt = timeStamp[upperIndex] - timeStamp[lowerIndex];
    if (dt <= 1e-9) {
        return std::vector<double>(positionTraj.front().size(), 0.0);
    }

    std::vector<double> velocity(positionTraj.front().size(), 0.0);

    // 差分法
    for (size_t dim = 0; dim < velocity.size(); ++dim) {
        velocity[dim] = (positionTraj[upperIndex][dim] - positionTraj[lowerIndex][dim]) / dt;
    }

    return velocity;
}

bool TrajectoryPlanner::loadPoseFile(const QString& path,
                                     int expectedEndNum,
                                     double positionModeUiRxOffsetRad,
                                     FileTrajectory& out,
                                     QString& errorMessage)
{
    // 文件格式约定：末端数、点数、时间戳、分段标记，然后每个末端 6 行位姿数据。
    // 解析完成后会补速度/加速度，保证后续仿真和 PVT 下发都使用统一的 MultiEndTrajectory 结构。
    QFile trajFile(path);
    if (!trajFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        errorMessage = QStringLiteral("错误：轨迹文件打开失败");
        return false;
    }

    QStringList data;
    QTextStream stream(&trajFile);
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (!line.isEmpty() && !line.startsWith('#')) {
            data.append(line);
        }
    }

    if (isSegmentPoseFileHeader(data)) {
        return loadSegmentPoseFile(data,
                                   expectedEndNum,
                                   positionModeUiRxOffsetRad,
                                   out,
                                   errorMessage);
    }

    if (data.size() < 10) {
        errorMessage = QStringLiteral("错误：轨迹文件数据不足");
        return false;
    }

    out = FileTrajectory{};
    bool ok = false;
    out.endNum = data.at(0).toInt(&ok);
    if (!ok || out.endNum <= 0) {
        errorMessage = QStringLiteral("错误：轨迹文件中的末端数量无效");
        return false;
    }
    if (expectedEndNum > 0 && out.endNum != expectedEndNum) {
        errorMessage = QStringLiteral("错误：读取的末端数量与界面配置不匹配");
        return false;
    }

    out.pointNum = data.at(1).toInt(&ok);
    if (!ok || out.pointNum < 2) {
        errorMessage = QStringLiteral("错误：轨迹文件中的轨迹点数量至少应为 2");
        return false;
    }

    const bool hasTimeStampLine = data.size() == (4 + out.endNum * 6);
    const bool hasLegacyPoseOnlyFormat = false;
    if (!hasTimeStampLine && !hasLegacyPoseOnlyFormat) {
        errorMessage = QStringLiteral("错误：轨迹文件格式不合法，应为“末端数 + 点数 + 时间戳行 + 分段标记行 + 6 行位姿/末端”");
        return false;
    }

    std::vector<double> fileTimeStamp;
    if (hasTimeStampLine) {
        if (!parseDoubleLine(data.at(2), out.pointNum, fileTimeStamp)) {
            errorMessage = QStringLiteral("错误：轨迹文件时间戳行格式不合法");
            return false;
        }
    } else {
        fileTimeStamp = buildUniformTimeAxis(out.pointNum, 1.0);
    }

    if (!parseSegmentFlagLine(data.at(3), out.pointNum, out.segmentFlags)) {
        errorMessage = QStringLiteral("错误：轨迹文件分段标记行格式不合法，应为每个轨迹点一个 0/1 标记");
        return false;
    }
    if (out.segmentFlags.front() != 0) {
        errorMessage = QStringLiteral("错误：第一个轨迹点不能标记为分段结束点");
        return false;
    }
    if (out.segmentFlags.back() != 1) {
        errorMessage = QStringLiteral("错误：最后一个轨迹点必须标记为分段结束点");
        return false;
    }

    // 分段标记用于程序控制模式逐段执行；每个 1 表示当前点是本段终点，同时作为下一段起点。
    int segmentStartIndex = 0;
    out.segmentRanges.clear();
    for (int pointIndex = 1; pointIndex < out.pointNum; ++pointIndex) {
        if (out.segmentFlags[pointIndex] == 0) {
            continue;
        }
        if (pointIndex <= segmentStartIndex) {
            errorMessage = QStringLiteral("错误：轨迹文件分段标记生成了无效分段");
            return false;
        }
        out.segmentRanges.push_back(std::make_pair(segmentStartIndex, pointIndex));
        segmentStartIndex = pointIndex;
    }
    if (out.segmentRanges.empty()) {
        errorMessage = QStringLiteral("错误：轨迹文件至少应包含一个有效分段");
        return false;
    }

    for (int index = 1; index < static_cast<int>(fileTimeStamp.size()); ++index) {
        if (fileTimeStamp[index] <= fileTimeStamp[index - 1]) {
            errorMessage = QStringLiteral("错误：轨迹文件时间戳必须严格递增");
            return false;
        }
    }

    const int poseLineBaseIndex = 4;
    out.duration = fileTimeStamp.empty() ? 0.0 : fileTimeStamp.back();

    out.offlineTraj.resize(out.endNum);
    for (int endIndex = 0; endIndex < out.endNum; ++endIndex) {
        out.offlineTraj[endIndex].resize(4);
        out.offlineTraj[endIndex][0].resize(6);
        out.offlineTraj[endIndex][1].resize(6);
        out.offlineTraj[endIndex][2].resize(6);
        out.offlineTraj[endIndex][3].resize(6);
        for (int dim = 0; dim < 6; ++dim) {
            const int lineIndex = poseLineBaseIndex + endIndex * 6 + dim;
            if (!parseDoubleLine(data.at(lineIndex), out.pointNum, out.offlineTraj[endIndex][0][dim])) {
                errorMessage = QStringLiteral("错误：轨迹文件第 %1 行位姿数据格式不合法").arg(lineIndex + 1);
                return false;
            }
            out.offlineTraj[endIndex][3][dim] = fileTimeStamp;
            out.offlineTraj[endIndex][1][dim] =
                estimateTrajectoryDerivative(out.offlineTraj[endIndex][0][dim], fileTimeStamp);
            out.offlineTraj[endIndex][2][dim] =
                estimateTrajectoryDerivative(out.offlineTraj[endIndex][1][dim], fileTimeStamp);
        }
    }

    out.firstEndStartPose.resize(6);
    for (int dim = 0; dim < 6; ++dim) {
        out.firstEndStartPose[dim] = out.offlineTraj[0][0][dim][0];
    }

    return true;
}

TrajectoryPlanner::MultiEndTrajectory
TrajectoryPlanner::sliceTrajectory(const MultiEndTrajectory& traj,
                                   int startPointIndex,
                                   int endPointIndex)
{
    if (traj.empty() || startPointIndex < 0 || endPointIndex <= startPointIndex) {
        return {};
    }

    MultiEndTrajectory sliced;
    sliced.resize(traj.size());
    for (size_t endIndex = 0; endIndex < traj.size(); ++endIndex) {
        if (traj[endIndex].size() < 4 ||
                traj[endIndex][0].empty() ||
                traj[endIndex][3].empty() ||
                traj[endIndex][3][0].empty() ||
                endPointIndex >= static_cast<int>(traj[endIndex][3][0].size())) {
            return {};
        }

        const int stateCount = static_cast<int>(traj[endIndex].size());
        sliced[endIndex].resize(stateCount);
        const double baseTime = traj[endIndex][3][0][startPointIndex];
        for (int stateIndex = 0; stateIndex < stateCount; ++stateIndex) {
            const int dimCount = static_cast<int>(traj[endIndex][stateIndex].size());
            sliced[endIndex][stateIndex].resize(dimCount);
            for (int dim = 0; dim < dimCount; ++dim) {
                if (endPointIndex >= static_cast<int>(traj[endIndex][stateIndex][dim].size())) {
                    return {};
                }
                for (int pointIndex = startPointIndex; pointIndex <= endPointIndex; ++pointIndex) {
                    double value = traj[endIndex][stateIndex][dim][pointIndex];
                    if (stateIndex == 3) {
                        value -= baseTime;
                    }
                    sliced[endIndex][stateIndex][dim].push_back(value);
                }
            }
        }
    }

    return sliced;
}

void TrajectoryPlanner::resampleTrajectory(
    std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
    double maxStep)
{
    if (maxStep <= 0.0) {
        return;
    }

    for (size_t endIndex = 0; endIndex < traj.size(); ++endIndex)
    {
        auto& tVec = traj[endIndex][3][0]; // 时间（所有dim相同）

        int dimNum = traj[endIndex][0].size();

        std::vector<std::vector<double>> newPos(dimNum);
        std::vector<std::vector<double>> newVel(dimNum);
        std::vector<std::vector<double>> newAcc(dimNum);
        std::vector<double> newTime;

        for (size_t k = 0; k < tVec.size() - 1; ++k)
        {
            double t0 = tVec[k];
            double t1 = tVec[k + 1];
            double dt = t1 - t0;

            // 原始点直接加入
            newTime.push_back(t0);
            for (int d = 0; d < dimNum; ++d) {
                newPos[d].push_back(traj[endIndex][0][d][k]);
                newVel[d].push_back(traj[endIndex][1][d][k]);
                newAcc[d].push_back(traj[endIndex][2][d][k]);
            }

            if (dt <= maxStep) continue;

            // ===== 需要插值 =====
            int insertNum = static_cast<int>(floor(dt / maxStep));

            for (int i = 1; i <= insertNum; ++i)
            {
                double ti = t0 + i * maxStep;
                if (ti >= t1) break;

                newTime.push_back(ti);

                double tau = ti - t0;

                for (int d = 0; d < dimNum; ++d)
                {
                    double p0 = traj[endIndex][0][d][k];
                    double v0 = traj[endIndex][1][d][k];
                    double p1 = traj[endIndex][0][d][k + 1];
                    double v1 = traj[endIndex][1][d][k + 1];

                    double T = dt;

                    double a0 = p0;
                    double a1 = v0;
                    double a2 = (3*(p1-p0) - (2*v0+v1)*T)/(T*T);
                    double a3 = (2*(p0-p1) + (v0+v1)*T)/(T*T*T);

                    double p = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau;
                    double v = a1 + 2*a2*tau + 3*a3*tau*tau;
                    double a = 2*a2 + 6*a3*tau;

                    newPos[d].push_back(p);
                    newVel[d].push_back(v);
                    newAcc[d].push_back(a);
                }
            }
        }

        // 最后一个点补上
        int last = tVec.size() - 1;
        newTime.push_back(tVec[last]);
        for (int d = 0; d < dimNum; ++d) {
            newPos[d].push_back(traj[endIndex][0][d][last]);
            newVel[d].push_back(traj[endIndex][1][d][last]);
            newAcc[d].push_back(traj[endIndex][2][d][last]);
        }

        // ===== 写回 =====
        traj[endIndex][0] = newPos;
        traj[endIndex][1] = newVel;
        traj[endIndex][2] = newAcc;

        for (int d = 0; d < dimNum; ++d)
            traj[endIndex][3][d] = newTime;
    }
}

bool TrajectoryPlanner::buildStopTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                      PointTrajectoryTransition& out,
                                                      QString& errorMessage)
{
    // 暂停过渡：从当前实测点平滑走到原轨迹未来某点，并记录 resumeTime 作为恢复时的参考时间。
    if (!validatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, errorMessage)) {
        return false;
    }
    if (request.currentPosition.size() != request.sourcePositionTraj.front().size()) {
        errorMessage = QStringLiteral("当前点维度与轨迹维度不一致");
        return false;
    }

    const double safeSampleTime = std::max(request.sampleTime, 0.001);
    const double safeTransitionTime = std::max(request.transitionTime, safeSampleTime);
    const double startTime = clampTime(request.currentTrajectoryTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    const double stopTime = clampTime(startTime + safeTransitionTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    const double actualDuration = std::max(stopTime - startTime, safeSampleTime);
    const int stepCount = std::max(2, static_cast<int>(std::ceil(actualDuration / safeSampleTime)) + 1);

    out = PointTrajectoryTransition{};
    out.positionTraj.reserve(stepCount);
    out.timeStamp.reserve(stepCount);
    out.positionTraj.push_back(request.currentPosition);
    out.timeStamp.push_back(0.0);

    for (int step = 1; step < stepCount; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(stepCount - 1);
        const double t = startTime + (stopTime - startTime) * ratio;
        out.positionTraj.push_back(interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, t));
        out.timeStamp.push_back(actualDuration * ratio);
    }

    out.resumeTime = stopTime;
    out.referencePosition = interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, stopTime);
    return true;
}

bool TrajectoryPlanner::buildResumeTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                        PointTrajectoryTransition& out,
                                                        QString& errorMessage)
{
    // 恢复过渡：先从当前实测点接回原轨迹 rampEndTime 处，再追加剩余原轨迹点，保证时间轴连续。
    if (!validatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, errorMessage)) {
        return false;
    }
    if (request.currentPosition.size() != request.sourcePositionTraj.front().size()) {
        errorMessage = QStringLiteral("当前点维度与轨迹维度不一致");
        return false;
    }

    const double startTime = clampTime(request.currentTrajectoryTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    if (startTime >= request.sourceTimeStamp.back() - 1e-9) {
        errorMessage = QStringLiteral("轨迹已接近终点，无剩余轨迹可缓启");
        return false;
    }

    const double safeSampleTime = std::max(request.sampleTime, 0.001);
    const double safeTransitionTime = std::max(request.transitionTime, safeSampleTime);
    const double rampEndTime = clampTime(startTime + safeTransitionTime,
                                         request.sourceTimeStamp.front(),
                                         request.sourceTimeStamp.back());
    const double rampDuration = std::max(rampEndTime - startTime, safeSampleTime);
    const int rampStepCount = std::max(2, static_cast<int>(std::ceil(rampDuration / safeSampleTime)) + 1);
    out = PointTrajectoryTransition{};
    out.positionTraj.push_back(request.currentPosition);
    out.timeStamp.push_back(0.0);

    for (int step = 1; step < rampStepCount; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(rampStepCount - 1);
        const double t = startTime + (rampEndTime - startTime) * ratio;
        out.positionTraj.push_back(interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, t));
        out.timeStamp.push_back(rampDuration * ratio);
    }

    for (size_t pointIndex = 0; pointIndex < request.sourceTimeStamp.size(); ++pointIndex) {
        if (request.sourceTimeStamp[pointIndex] <= rampEndTime + 1e-9) {
            continue;
        }
        const std::vector<double>& point = request.sourcePositionTraj[pointIndex];
        const double relativeTime = request.sourceTimeStamp[pointIndex] - startTime;
        if (relativeTime > out.timeStamp.back() + 1e-9) {
            out.positionTraj.push_back(point);
            out.timeStamp.push_back(relativeTime);
        }
    }

    out.resumeTime = rampEndTime;
    out.referencePosition = interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, rampEndTime);
    return out.positionTraj.size() >= 2;
}
