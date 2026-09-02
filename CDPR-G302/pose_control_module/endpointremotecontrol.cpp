#include "endpointremotecontrol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QStringList>
#include <QTextStream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace {

constexpr qint64 kSchedulingRecoveryDiagnosticIntervalUs = 5 * 1000 * 1000;
constexpr qint64 kEndpointRemoteRecoverableMissedPeriods = 1;
constexpr double kEndpointRemoteCommandResponsePeriods = 1.0;
constexpr double kEndpointRemoteBoundaryVelocityEpsilonMmPerSec = 1.0e-9;
constexpr double kEndpointRemoteAngularVelocityEpsilonRadPerSec = 1.0e-12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kVoxelGeometryToleranceMm = 1.0e-6;
constexpr double kVoxelAngleToleranceRad = 1.0e-12;
constexpr double kYawLockToleranceRad = 1.0e-10;
constexpr double kRotationBoundarySampleStepRad = kPi / 1800.0; // 0.1 deg
constexpr int kMaximumRotationBoundarySamples = 4096;

std::size_t voxelLinearIndex(const std::array<int, 3>& index,
                             const std::array<int, 3>& count)
{
    return (static_cast<std::size_t>(index[0]) *
            static_cast<std::size_t>(count[1]) +
            static_cast<std::size_t>(index[1])) *
            static_cast<std::size_t>(count[2]) +
            static_cast<std::size_t>(index[2]);
}

Eigen::Matrix3d eulerZYXToRotation(const Eigen::Vector3d& euler)
{
    const Eigen::AngleAxisd roll(euler(0), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(euler(1), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(euler(2), Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

std::array<double, 6> integrateYawLockedEulerRate(
        std::array<double, 6> pose,
        const std::array<double, 3>& eulerRate,
        double dtSec,
        double lockedYawRad)
{
    pose[3] += eulerRate[0] * dtSec;
    pose[4] += eulerRate[1] * dtSec;
    // Rz是会话级锁存量，不通过SO(3)固定轴积分间接求解，避免Rx/Ry复合
    // 转动产生欧拉角Rz耦合漂移。
    pose[5] = lockedYawRad;
    return pose;
}

std::array<double, 3> yawLockedEulerRateToGlobalAngularVelocity(
        const std::array<double, 6>& pose,
        const std::array<double, 3>& eulerRate)
{
    const double pitch = pose[4];
    const double yaw = pose[5];
    const double cosPitch = std::cos(pitch);
    const double sinPitch = std::sin(pitch);
    const double cosYaw = std::cos(yaw);
    const double sinYaw = std::sin(yaw);
    const double rollRate = eulerRate[0];
    const double pitchRate = eulerRate[1];
    // ZYX欧拉角速度到空间/全局角速度；eulerRate[2]固定为零。
    return {{
        cosYaw * cosPitch * rollRate - sinYaw * pitchRate,
        sinYaw * cosPitch * rollRate + cosYaw * pitchRate,
        -sinPitch * rollRate
    }};
}

std::array<double, 3> finiteStepGlobalAngularVelocity(
        const std::array<double, 6>& startPose,
        const std::array<double, 6>& endPose,
        double dtSec)
{
    std::array<double, 3> result{};
    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        result.fill(std::numeric_limits<double>::quiet_NaN());
        return result;
    }
    const Eigen::Vector3d startEuler(startPose[3], startPose[4], startPose[5]);
    const Eigen::Vector3d endEuler(endPose[3], endPose[4], endPose[5]);
    const Eigen::Matrix3d relativeRotation =
            eulerZYXToRotation(endEuler) *
            eulerZYXToRotation(startEuler).transpose();
    if(!relativeRotation.allFinite()){
        result.fill(std::numeric_limits<double>::quiet_NaN());
        return result;
    }
    const Eigen::AngleAxisd angleAxis(relativeRotation);
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    if(angleAxis.angle() >
            kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        omega = angleAxis.axis() * (angleAxis.angle() / dtSec);
    }
    result = {{omega(0), omega(1), omega(2)}};
    return result;
}

double wrappedAngleError(double value, double reference)
{
    return std::remainder(value - reference, kTwoPi);
}

constexpr double endpointRemoteGuardedTravelPeriods()
{
    // 一次候选位置积分 + 一次命令响应保持 + 允许的一次漏拍补推。
    return 1.0 + kEndpointRemoteCommandResponsePeriods +
            static_cast<double>(kEndpointRemoteRecoverableMissedPeriods);
}

qint64 endpointRemoteMonotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

template<std::size_t N>
bool finiteArray(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value){
        return std::isfinite(value);
    });
}

double vectorNorm(const std::array<double, 3>& vector)
{
    return std::sqrt(vector[0] * vector[0] +
                     vector[1] * vector[1] +
                     vector[2] * vector[2]);
}

std::array<double, 3> normalizedVector(std::array<double, 3> vector)
{
    const double norm = vectorNorm(vector);
    if(norm > 1.0){
        for(double& value : vector){
            value /= norm;
        }
    }
    return vector;
}

int boundaryDimension(int direction)
{
    return direction / 2;
}

double boundarySign(int direction)
{
    return direction % 2 == 0 ? 1.0 : -1.0;
}

QString boundaryDirectionText(int direction)
{
    static const char* const labels[kEndpointRemoteBoundaryDirectionCount] = {
        "+X", "-X", "+Y", "-Y", "+Z", "-Z"
    };
    return direction >= 0 && direction < kEndpointRemoteBoundaryDirectionCount ?
                QString::fromLatin1(labels[direction]) : QStringLiteral("未知方向");
}

QString boundaryStateText(EndpointRemoteBoundaryState state)
{
    switch(state){
    case EndpointRemoteBoundaryState::Normal:
        return QStringLiteral("正常");
    case EndpointRemoteBoundaryState::SoftBoundary:
        return QStringLiteral("软边界");
    case EndpointRemoteBoundaryState::Braking:
        return QStringLiteral("制动");
    case EndpointRemoteBoundaryState::BlockedOutward:
        return QStringLiteral("朝外阻塞");
    }
    return QStringLiteral("未知");
}

}

bool EndpointRemoteVoxelAngleMap::loadCsv(
        const QString& filePath,
        QString* errorMessage)
{
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        return fail(QStringLiteral("无法打开遥控体素角度表：%1（%2）")
                    .arg(QFileInfo(filePath).absoluteFilePath(),
                         file.errorString()));
    }
    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    if(stream.atEnd()){
        return fail(QStringLiteral("遥控体素角度表为空：%1")
                    .arg(QFileInfo(filePath).absoluteFilePath()));
    }

    QString headerLine = stream.readLine();
    if(!headerLine.isEmpty() && headerLine.at(0).unicode() == 0xfeff){
        headerLine.remove(0, 1);
    }
    const QStringList headers = headerLine.split(QLatin1Char(','));
    const auto column = [&](const char* name){
        return headers.indexOf(QString::fromLatin1(name));
    };
    const int ixColumn = column("ix");
    const int iyColumn = column("iy");
    const int izColumn = column("iz");
    const std::array<int, 3> minimumColumns{{
        column("x_min_mm"), column("y_min_mm"), column("z_min_mm")
    }};
    const std::array<int, 3> maximumColumns{{
        column("x_max_mm"), column("y_max_mm"), column("z_max_mm")
    }};
    const int rxMinimumColumn = column("rx_min_rad");
    const int rxMaximumColumn = column("rx_max_rad");
    const int ryMinimumColumn = column("ry_min_rad");
    const int ryMaximumColumn = column("ry_max_rad");
    const int validColumn = column("valid");
    std::array<int, 14> requiredColumns{{
        ixColumn, iyColumn, izColumn,
        minimumColumns[0], maximumColumns[0],
        minimumColumns[1], maximumColumns[1],
        minimumColumns[2], maximumColumns[2],
        rxMinimumColumn, rxMaximumColumn,
        ryMinimumColumn, ryMaximumColumn, validColumn
    }};
    if(std::any_of(requiredColumns.begin(), requiredColumns.end(),
                   [](int value){ return value < 0; })){
        return fail(QStringLiteral(
                        "遥控体素角度表缺少必要列：ix/iy/iz、XYZ范围、Rx/Ry范围或valid"));
    }

    struct ParsedCell {
        std::array<int, 3> index{{0, 0, 0}};
        EndpointRemoteVoxelAngleLimit limit;
        int lineNumber = 0;
    };
    std::vector<ParsedCell> parsedCells;
    std::array<int, 3> maximumIndex{{-1, -1, -1}};
    int lineNumber = 1;
    while(!stream.atEnd()){
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if(line.isEmpty()){
            continue;
        }
        const QStringList fields = line.split(QLatin1Char(','));
        if(fields.size() < headers.size()){
            return fail(QStringLiteral("遥控体素角度表第%1行列数不足")
                        .arg(lineNumber));
        }
        const auto parseInteger = [&](int columnIndex,
                                      int* result,
                                      const QString& fieldName){
            bool ok = false;
            const int value = fields.at(columnIndex).trimmed().toInt(&ok);
            if(!ok && errorMessage){
                *errorMessage = QStringLiteral("遥控体素角度表第%1行%2不是整数")
                        .arg(lineNumber).arg(fieldName);
            }
            if(ok && result){
                *result = value;
            }
            return ok;
        };
        const auto parseDouble = [&](int columnIndex,
                                     double* result,
                                     const QString& fieldName){
            bool ok = false;
            const double value = fields.at(columnIndex).trimmed().toDouble(&ok);
            ok = ok && std::isfinite(value);
            if(!ok && errorMessage){
                *errorMessage = QStringLiteral("遥控体素角度表第%1行%2不是有限数值")
                        .arg(lineNumber).arg(fieldName);
            }
            if(ok && result){
                *result = value;
            }
            return ok;
        };

        ParsedCell parsed;
        parsed.lineNumber = lineNumber;
        if(!parseInteger(ixColumn, &parsed.index[0], QStringLiteral("ix")) ||
                !parseInteger(iyColumn, &parsed.index[1], QStringLiteral("iy")) ||
                !parseInteger(izColumn, &parsed.index[2], QStringLiteral("iz"))){
            return false;
        }
        for(int dim = 0; dim < 3; ++dim){
            if(parsed.index[dim] < 0){
                return fail(QStringLiteral("遥控体素角度表第%1行索引不能为负数")
                            .arg(lineNumber));
            }
            maximumIndex[dim] = std::max(maximumIndex[dim], parsed.index[dim]);
            if(!parseDouble(minimumColumns[dim],
                            &parsed.limit.minimumMm[dim],
                            QStringLiteral("空间下界")) ||
                    !parseDouble(maximumColumns[dim],
                                 &parsed.limit.maximumMm[dim],
                                 QStringLiteral("空间上界"))){
                return false;
            }
        }
        int validValue = 0;
        if(!parseDouble(rxMinimumColumn, &parsed.limit.rxMinimumRad,
                        QStringLiteral("rx_min_rad")) ||
                !parseDouble(rxMaximumColumn, &parsed.limit.rxMaximumRad,
                             QStringLiteral("rx_max_rad")) ||
                !parseDouble(ryMinimumColumn, &parsed.limit.ryMinimumRad,
                             QStringLiteral("ry_min_rad")) ||
                !parseDouble(ryMaximumColumn, &parsed.limit.ryMaximumRad,
                             QStringLiteral("ry_max_rad")) ||
                !parseInteger(validColumn, &validValue, QStringLiteral("valid"))){
            return false;
        }
        parsed.limit.valid = validValue == 1;
        if(validValue != 0 && validValue != 1){
            return fail(QStringLiteral("遥控体素角度表第%1行valid只能为0或1")
                        .arg(lineNumber));
        }
        parsedCells.push_back(parsed);
    }
    if(parsedCells.empty()){
        return fail(QStringLiteral("遥控体素角度表没有数据行"));
    }

    EndpointRemoteVoxelAngleMap candidate;
    candidate.sourceFilePath = QFileInfo(filePath).absoluteFilePath();
    for(int dim = 0; dim < 3; ++dim){
        candidate.cellCount[dim] = maximumIndex[dim] + 1;
        if(candidate.cellCount[dim] <= 0 || candidate.cellCount[dim] > 1000){
            return fail(QStringLiteral("遥控体素角度表维度%1的格子数量无效")
                        .arg(dim));
        }
        candidate.workspaceMinimumMm[dim] =
                std::numeric_limits<double>::infinity();
        candidate.workspaceMaximumMm[dim] =
                -std::numeric_limits<double>::infinity();
        for(const ParsedCell& parsed : parsedCells){
            candidate.workspaceMinimumMm[dim] = std::min(
                        candidate.workspaceMinimumMm[dim],
                        parsed.limit.minimumMm[dim]);
            candidate.workspaceMaximumMm[dim] = std::max(
                        candidate.workspaceMaximumMm[dim],
                        parsed.limit.maximumMm[dim]);
        }
        candidate.cellSizeMm[dim] =
                (candidate.workspaceMaximumMm[dim] -
                 candidate.workspaceMinimumMm[dim]) /
                static_cast<double>(candidate.cellCount[dim]);
    }
    const std::size_t expectedCellCount =
            static_cast<std::size_t>(candidate.cellCount[0]) *
            static_cast<std::size_t>(candidate.cellCount[1]) *
            static_cast<std::size_t>(candidate.cellCount[2]);
    if(expectedCellCount == 0 || expectedCellCount > 1000000){
        return fail(QStringLiteral("遥控体素角度表网格规模无效：%1")
                    .arg(static_cast<qulonglong>(expectedCellCount)));
    }
    candidate.cells.resize(expectedCellCount);
    std::vector<bool> occupied(expectedCellCount, false);
    for(const ParsedCell& parsed : parsedCells){
        for(int dim = 0; dim < 3; ++dim){
            const double expectedMinimum =
                    candidate.workspaceMinimumMm[dim] +
                    parsed.index[dim] * candidate.cellSizeMm[dim];
            const double expectedMaximum =
                    expectedMinimum + candidate.cellSizeMm[dim];
            if(std::abs(parsed.limit.minimumMm[dim] - expectedMinimum) >
                        kVoxelGeometryToleranceMm ||
                    std::abs(parsed.limit.maximumMm[dim] - expectedMaximum) >
                        kVoxelGeometryToleranceMm){
                return fail(QStringLiteral(
                                "遥控体素角度表第%1行维度%2不符合规则网格")
                            .arg(parsed.lineNumber).arg(dim));
            }
        }
        const std::size_t linear =
                voxelLinearIndex(parsed.index, candidate.cellCount);
        if(occupied[linear]){
            return fail(QStringLiteral(
                            "遥控体素角度表第%1行索引[%2,%3,%4]重复")
                        .arg(parsed.lineNumber)
                        .arg(parsed.index[0]).arg(parsed.index[1])
                        .arg(parsed.index[2]));
        }
        occupied[linear] = true;
        candidate.cells[linear] = parsed.limit;
    }
    if(std::find(occupied.begin(), occupied.end(), false) != occupied.end()){
        return fail(QStringLiteral(
                        "遥控体素角度表网格不完整：期望%1格，实际%2格")
                    .arg(static_cast<qulonglong>(expectedCellCount))
                    .arg(static_cast<qulonglong>(parsedCells.size())));
    }

    QString validationError;
    if(!candidate.validate(&validationError)){
        return fail(validationError);
    }
    *this = std::move(candidate);
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool EndpointRemoteVoxelAngleMap::validate(QString* errorMessage) const
{
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(sourceFilePath.trimmed().isEmpty() ||
            !finiteArray(workspaceMinimumMm) ||
            !finiteArray(workspaceMaximumMm) ||
            !finiteArray(cellSizeMm)){
        return fail(QStringLiteral("遥控体素角度表来源或网格数值无效"));
    }
    std::size_t expectedCellCount = 1;
    for(int dim = 0; dim < 3; ++dim){
        if(cellCount[dim] <= 0 || cellSizeMm[dim] <= 0.0 ||
                workspaceMinimumMm[dim] >= workspaceMaximumMm[dim] ||
                std::abs(workspaceMinimumMm[dim] +
                         cellCount[dim] * cellSizeMm[dim] -
                         workspaceMaximumMm[dim]) >
                    kVoxelGeometryToleranceMm){
            return fail(QStringLiteral("遥控体素角度表网格维度%1无效")
                        .arg(dim));
        }
        expectedCellCount *= static_cast<std::size_t>(cellCount[dim]);
    }
    if(cells.size() != expectedCellCount){
        return fail(QStringLiteral("遥控体素角度表单元数量与网格尺寸不一致"));
    }
    for(std::size_t linear = 0; linear < cells.size(); ++linear){
        const EndpointRemoteVoxelAngleLimit& limit = cells[linear];
        if(!limit.valid || !finiteArray(limit.minimumMm) ||
                !finiteArray(limit.maximumMm) ||
                !std::isfinite(limit.rxMinimumRad) ||
                !std::isfinite(limit.rxMaximumRad) ||
                !std::isfinite(limit.ryMinimumRad) ||
                !std::isfinite(limit.ryMaximumRad) ||
                limit.rxMinimumRad >= limit.rxMaximumRad ||
                limit.ryMinimumRad >= limit.ryMaximumRad ||
                limit.rxMinimumRad > kVoxelAngleToleranceRad ||
                limit.rxMaximumRad < -kVoxelAngleToleranceRad ||
                limit.ryMinimumRad > kVoxelAngleToleranceRad ||
                limit.ryMaximumRad < -kVoxelAngleToleranceRad){
            return fail(QStringLiteral("遥控体素角度表第%1个单元无效或不包含零姿态")
                        .arg(static_cast<qulonglong>(linear)));
        }
    }
    return true;
}

const EndpointRemoteVoxelAngleLimit* EndpointRemoteVoxelAngleMap::limitAt(
        const std::array<int, 3>& index) const
{
    for(int dim = 0; dim < 3; ++dim){
        if(index[dim] < 0 || index[dim] >= cellCount[dim]){
            return nullptr;
        }
    }
    const std::size_t linear = voxelLinearIndex(index, cellCount);
    return linear < cells.size() && cells[linear].valid ?
                &cells[linear] : nullptr;
}

const EndpointRemoteVoxelAngleLimit* EndpointRemoteVoxelAngleMap::limitForPose(
        const std::array<double, 6>& pose,
        std::array<int, 3>* index) const
{
    std::array<int, 3> resolved{{0, 0, 0}};
    for(int dim = 0; dim < 3; ++dim){
        const double coordinate = pose[dim];
        if(!std::isfinite(coordinate) ||
                coordinate < workspaceMinimumMm[dim] ||
                coordinate > workspaceMaximumMm[dim]){
            return nullptr;
        }
        if(coordinate >= workspaceMaximumMm[dim]){
            resolved[dim] = cellCount[dim] - 1;
        }
        else{
            resolved[dim] = static_cast<int>(std::floor(
                        (coordinate - workspaceMinimumMm[dim]) /
                        cellSizeMm[dim]));
        }
    }
    const EndpointRemoteVoxelAngleLimit* limit = limitAt(resolved);
    if(limit && index){
        *index = resolved;
    }
    return limit;
}

bool EndpointRemoteVoxelAngleMap::commonRxRyRange(
        std::array<double, 2>* rxRangeRad,
        std::array<double, 2>* ryRangeRad) const
{
    if(!rxRangeRad || !ryRangeRad || cells.empty()){
        return false;
    }
    std::array<double, 2> rx{{
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    }};
    std::array<double, 2> ry = rx;
    for(const EndpointRemoteVoxelAngleLimit& limit : cells){
        if(!limit.valid){
            return false;
        }
        rx[0] = std::max(rx[0], limit.rxMinimumRad);
        rx[1] = std::min(rx[1], limit.rxMaximumRad);
        ry[0] = std::max(ry[0], limit.ryMinimumRad);
        ry[1] = std::min(ry[1], limit.ryMaximumRad);
    }
    if(!finiteArray(rx) || !finiteArray(ry) ||
            rx[0] >= rx[1] || ry[0] >= ry[1]){
        return false;
    }
    *rxRangeRad = rx;
    *ryRangeRad = ry;
    return true;
}

double EndpointRemoteConfig::outwardRestartResponseTimeSec() const
{
    return endpointRemoteGuardedTravelPeriods() *
            static_cast<double>(onlineVelocity.periodUs) / 1000000.0;
}

double EndpointRemoteConfig::outwardRestartSafeAccelerationMmPerSec2() const
{
    return translationAccelerationMmPerSec2 *
            outwardRestartSafeAccelerationRatio;
}

double EndpointRemoteConfig::outwardRestartGuardMm() const
{
    const double safeAcceleration =
            outwardRestartSafeAccelerationMmPerSec2();
    if(!std::isfinite(safeAcceleration) || safeAcceleration <= 0.0){
        return std::numeric_limits<double>::infinity();
    }
    return translationSpeedMmPerSec * translationSpeedMmPerSec /
                (2.0 * safeAcceleration) +
            translationSpeedMmPerSec * outwardRestartResponseTimeSec() +
            outwardRestartPoseErrorMarginMm;
}

bool EndpointRemoteConfig::validate(QString* errorMessage) const
{
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    QString onlineError;
    if(!onlineVelocity.validate(&onlineError)){
        return fail(QStringLiteral("末端遥控在线速度参数无效：%1").arg(onlineError));
    }
    if(!finiteArray(initialPose) ||
            !finiteArray(workspaceMinimum) ||
            !finiteArray(workspaceMaximum) ||
            !finiteArray(orientationMinimumRad) ||
            !finiteArray(orientationMaximumRad) ||
            !finiteArray(translationSafeOrientationMinimumRad) ||
            !finiteArray(translationSafeOrientationMaximumRad) ||
            !finiteArray(motorUnitPerRadian) ||
            !finiteArray(motorPositionMinimum) ||
            !finiteArray(motorPositionMaximum) ||
            !finiteArray(preparedMotorPosition)){
        return fail(QStringLiteral("末端遥控配置包含非有限数值"));
    }
    for(int dim = 0; dim < 3; ++dim){
        if(workspaceMinimum[dim] >= workspaceMaximum[dim] ||
                initialPose[dim] <= workspaceMinimum[dim] ||
                initialPose[dim] >= workspaceMaximum[dim]){
            return fail(QStringLiteral("遥控初始位姿或工作空间维度%1无效").arg(dim));
        }
        if(orientationMinimumRad[dim] >= orientationMaximumRad[dim] ||
                translationSafeOrientationMinimumRad[dim] >=
                    translationSafeOrientationMaximumRad[dim] ||
                translationSafeOrientationMinimumRad[dim] <
                    orientationMinimumRad[dim] ||
                translationSafeOrientationMaximumRad[dim] >
                    orientationMaximumRad[dim] ||
                (dim == 2 &&
                 (initialPose[dim + 3] < orientationMinimumRad[dim] ||
                  initialPose[dim + 3] > orientationMaximumRad[dim]))){
            return fail(QStringLiteral("遥控姿态硬边界、平动安全姿态范围或初始姿态维度%1无效")
                        .arg(dim));
        }
    }
    QString voxelMapError;
    if(!voxelAngleLimits.validate(&voxelMapError)){
        return fail(QStringLiteral("遥控体素角度表无效：%1")
                    .arg(voxelMapError));
    }
    for(int dim = 0; dim < 3; ++dim){
        if(std::abs(voxelAngleLimits.workspaceMinimumMm[dim] -
                    workspaceMinimum[dim]) > kVoxelGeometryToleranceMm ||
                std::abs(voxelAngleLimits.workspaceMaximumMm[dim] -
                         workspaceMaximum[dim]) > kVoxelGeometryToleranceMm){
            return fail(QStringLiteral(
                            "遥控体素角度表维度%1范围[%2,%3] mm与工作空间[%4,%5] mm不一致")
                        .arg(dim)
                        .arg(voxelAngleLimits.workspaceMinimumMm[dim], 0, 'f', 3)
                        .arg(voxelAngleLimits.workspaceMaximumMm[dim], 0, 'f', 3)
                        .arg(workspaceMinimum[dim], 0, 'f', 3)
                        .arg(workspaceMaximum[dim], 0, 'f', 3));
        }
    }
    std::array<double, 2> commonRxRange{};
    std::array<double, 2> commonRyRange{};
    if(!voxelAngleLimits.commonRxRyRange(&commonRxRange, &commonRyRange) ||
            translationSafeOrientationMinimumRad[0] <
                commonRxRange[0] - kVoxelAngleToleranceRad ||
            translationSafeOrientationMaximumRad[0] >
                commonRxRange[1] + kVoxelAngleToleranceRad ||
            translationSafeOrientationMinimumRad[1] <
                commonRyRange[0] - kVoxelAngleToleranceRad ||
            translationSafeOrientationMaximumRad[1] >
                commonRyRange[1] + kVoxelAngleToleranceRad){
        return fail(QStringLiteral(
                        "平动安全Rx/Ry范围不是所有体素允许范围的有效子集"));
    }
    const EndpointRemoteVoxelAngleLimit* initialVoxelLimit =
            voxelAngleLimits.limitForPose(initialPose);
    if(!initialVoxelLimit ||
            initialPose[3] < initialVoxelLimit->rxMinimumRad ||
            initialPose[3] > initialVoxelLimit->rxMaximumRad ||
            initialPose[4] < initialVoxelLimit->ryMinimumRad ||
            initialPose[4] > initialVoxelLimit->ryMaximumRad){
        return fail(QStringLiteral(
                        "遥控初始Rx/Ry姿态不在起点体素的有效角度范围内"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(std::abs(motorUnitPerRadian[axis]) < 1.0e-12 ||
                motorPositionMinimum[axis] >= motorPositionMaximum[axis] ||
                preparedMotorPosition[axis] < motorPositionMinimum[axis] ||
                preparedMotorPosition[axis] > motorPositionMaximum[axis]){
            return fail(QStringLiteral("遥控电机轴%1的单位换算、准备位置或软限位无效")
                        .arg(axis + 1));
        }
    }
    if(!std::isfinite(preparedMotorDriftTolerance) ||
            preparedMotorDriftTolerance <= 0.0 ||
            !std::isfinite(translationSpeedMmPerSec) ||
            translationSpeedMmPerSec <= 0.0 ||
            !std::isfinite(translationAccelerationMmPerSec2) ||
            translationAccelerationMmPerSec2 <= 0.0 ||
            !std::isfinite(maximumAngularSpeedRadPerSec) ||
            maximumAngularSpeedRadPerSec <= 0.0 ||
            !std::isfinite(maximumAngularAccelerationRadPerSec2) ||
            maximumAngularAccelerationRadPerSec2 <= 0.0 ||
            !std::isfinite(softBoundaryMarginMm) ||
            softBoundaryMarginMm <= 0.0 ||
            !std::isfinite(boundaryReleaseHysteresisMm) ||
            boundaryReleaseHysteresisMm <= 0.0 ||
            !std::isfinite(outwardRestartSafeAccelerationRatio) ||
            outwardRestartSafeAccelerationRatio <= 0.0 ||
            outwardRestartSafeAccelerationRatio > 1.0 ||
            !std::isfinite(outwardRestartPoseErrorMarginMm) ||
            outwardRestartPoseErrorMarginMm < 0.0 ||
            inputHeartbeatTimeoutUs <= 0){
        return fail(QStringLiteral("遥控速度、加速度、边界参数、起点漂移阈值或输入超时无效"));
    }
    const double outwardRestartGuardMm =
            this->outwardRestartGuardMm();
    if(!std::isfinite(outwardRestartGuardMm) ||
            outwardRestartGuardMm <= 0.0){
        return fail(QStringLiteral("末端遥控朝外重启保护层计算结果无效"));
    }
    for(int dim = 0; dim < 3; ++dim){
        const double span = workspaceMaximum[dim] - workspaceMinimum[dim];
        if(2.0 * softBoundaryMarginMm >= span ||
                boundaryReleaseHysteresisMm >= span){
            return fail(QStringLiteral("遥控工作空间维度%1的软边界或回滞距离过大")
                        .arg(dim));
        }
        if(2.0 * (outwardRestartGuardMm +
                  boundaryReleaseHysteresisMm) >= span){
            return fail(QStringLiteral(
                        "遥控工作空间维度%1无法容纳朝外重启保护层G=%2 mm及回滞距离")
                        .arg(dim)
                        .arg(outwardRestartGuardMm, 0, 'f', 3));
        }
    }
    if(kinematics.anchorCableCoordinate.size() != kOnlineVelocityAxisCount ||
            kinematics.cableMotorScaleRadPerMm.size() != kOnlineVelocityAxisCount){
        return fail(QStringLiteral("末端遥控运动学必须包含完整八轴参数"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        const WinchCompensation::AxisConfig axisWinch =
                axis < static_cast<int>(kinematics.winchConfig.size()) ?
                    kinematics.winchConfig[axis] :
                    WinchCompensation::AxisConfig();
        if(!std::isfinite(kinematics.cableMotorScaleRadPerMm[axis]) ||
                (!WinchCompensation::isEnabled(axisWinch) &&
                 std::abs(kinematics.cableMotorScaleRadPerMm[axis]) < 1.0e-12)){
            return fail(QStringLiteral("末端遥控轴%1缺少有效绞盘或线性电机换算")
                        .arg(axis + 1));
        }
    }
    return true;
}

bool EndpointRemoteControl::prepare(const EndpointRemoteConfig& remoteConfig,
                                    QString* errorMessage)
{
    if(isActive()){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控正在运行，不能重新准备");
        }
        return false;
    }
    QString validationError;
    if(!remoteConfig.validate(&validationError)){
        if(errorMessage){
            *errorMessage = validationError;
        }
        return false;
    }

    CompensatedCableKinematics preparedKinematics;
    const CompensatedCableKinematics::PoseMatrix initialPoseMatrix{
        std::vector<double>(remoteConfig.initialPose.begin(),
                            remoteConfig.initialPose.end())
    };
    if(!preparedKinematics.initialize(remoteConfig.kinematics,
                                      initialPoseMatrix,
                                      {},
                                      &validationError)){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控运动学初始化失败：%1")
                    .arg(validationError);
        }
        return false;
    }
    CompensatedCableKinematics::State initialState =
            preparedKinematics.initialState();
    const CompensatedCableKinematics::Evaluation initialEvaluation =
            preparedKinematics.evaluatePose(initialPoseMatrix, initialState);
    if(!initialEvaluation.valid ||
            initialEvaluation.relativeMotorThetaRad.size() != kOnlineVelocityAxisCount){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控初始八轴坐标计算失败：%1")
                    .arg(initialEvaluation.errorMessage);
        }
        return false;
    }

    config = remoteConfig;
    kinematics = preparedKinematics;
    committedKinematicsState = initialEvaluation.nextState;
    committedRelativeMotorThetaRad = initialEvaluation.relativeMotorThetaRad;
    committedPose = config.initialPose;
    requestedMotionMode = EndpointRemoteMotionMode::None;
    activeMotionMode = EndpointRemoteMotionMode::None;
    requestedDirection.fill(0.0);
    committedEffectiveVelocity.fill(0.0);
    committedEffectiveEulerRate.fill(0.0);
    committedGlobalAngularVelocity.fill(0.0);
    lockedYawRad = config.initialPose[5];
    boundaryState.fill(EndpointRemoteBoundaryState::Normal);
    actualStartMotorPosition.fill(0.0);
    lastCommandVelocity.fill(0.0);
    pendingCandidate = Candidate();
    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
    pendingPriorActuationProfile = EndpointRemoteActuationProfile::Disarmed;
    actualStartCaptured = false;
    inputHeartbeatArmed = false;
    inputHeartbeatStale = false;
    waitStartUs = 0;
    sessionStartUs = 0;
    nextDueUs = 0;
    lastCommandDispatchUs = 0;
    lastInputUpdateUs = 0;
    lastUsedFrameSequence = 0;
    lastUsedFrameSequenceValid = false;
    lastSchedulingRecoveryDiagnosticUs = 0;
    suppressedSchedulingRecoveryDiagnosticCount = 0;

    currentStatus = EndpointRemoteStatus();
    currentStatus.state = EndpointRemoteStatus::State::Prepared;
    currentStatus.actuationProfile = EndpointRemoteActuationProfile::Disarmed;
    currentStatus.message = QStringLiteral("末端遥控已准备，尚未下发速度");
    currentStatus.initialPose = config.initialPose;
    currentStatus.desiredPose = config.initialPose;
    currentStatus.workspaceMinimum = config.workspaceMinimum;
    currentStatus.workspaceMaximum = config.workspaceMaximum;
    currentStatus.requestedMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.activeMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.yawLockReferenceRad = lockedYawRad;
    currentStatus.yawLockErrorRad = 0.0;
    currentStatus.angularBoundaryBrakingActive = false;
    currentStatus.outwardRestartGuardMm =
            config.outwardRestartGuardMm();
    currentStatus.outwardRestartSafeAccelerationMmPerSec2 =
            config.outwardRestartSafeAccelerationMmPerSec2();
    currentStatus.outwardRestartResponseTimeMs =
            config.outwardRestartResponseTimeSec() * 1000.0;
    currentStatus.outwardRestartPoseErrorMarginMm =
            config.outwardRestartPoseErrorMarginMm;
    currentStatus.referencePosition = config.preparedMotorPosition;
    currentStatus.inputSourceUiFresh = true;
    currentStatus.inputSourceUiAgeUs = 0;
    std::array<EndpointRemoteBoundaryState,
               kEndpointRemoteBoundaryDirectionCount> initialBoundaryState{};
    initialBoundaryState.fill(EndpointRemoteBoundaryState::Normal);
    initialBoundaryState = advanceBoundaryStates(
                committedPose, committedEffectiveVelocity,
                initialBoundaryState);
    updateBoundaryStatus(committedPose, initialBoundaryState);
    updateOrientationStatus(committedPose);
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool EndpointRemoteControl::start(qint64 nowUs, QString* errorMessage)
{
    Q_UNUSED(nowUs);
    if(!isPrepared()){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备末端遥控");
        }
        return false;
    }
    requestedMotionMode = EndpointRemoteMotionMode::None;
    activeMotionMode = EndpointRemoteMotionMode::None;
    requestedDirection.fill(0.0);
    committedEffectiveVelocity.fill(0.0);
    committedEffectiveEulerRate.fill(0.0);
    committedGlobalAngularVelocity.fill(0.0);
    lastCommandVelocity.fill(0.0);
    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
    pendingPriorActuationProfile = EndpointRemoteActuationProfile::Disarmed;
    actualStartCaptured = false;
    inputHeartbeatArmed = false;
    waitStartUs = 0;
    sessionStartUs = 0;
    nextDueUs = 0;
    lastCommandDispatchUs = 0;
    lastInputUpdateUs = 0;
    lastUsedFrameSequenceValid = false;
    currentStatus.state = EndpointRemoteStatus::State::WaitingForTrace;
    currentStatus.actuationProfile =
            EndpointRemoteActuationProfile::VerifyingStationary;
    currentStatus.message = QStringLiteral("等待独立输入监督器的初始零方向握手");
    currentStatus.inputAgeUs = -1;
    currentStatus.inputSourceUiFresh = true;
    currentStatus.inputSourceUiAgeUs = 0;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

void EndpointRemoteControl::resetTraceWaitClock(qint64 nowUs)
{
    if(currentStatus.state != EndpointRemoteStatus::State::WaitingForTrace){
        return;
    }
    waitStartUs = inputHeartbeatArmed ? nowUs : 0;
    nextDueUs = inputHeartbeatArmed ? nowUs : 0;
    lastUsedFrameSequenceValid = false;
}

void EndpointRemoteControl::updateInput(
        EndpointRemoteMotionMode motionMode,
        const std::array<double, 3>& normalizedDirection,
        quint64 sequence,
        qint64 nowUs,
        bool uiSourceFresh,
        qint64 uiSourceAgeUs)
{
    if(!isActive()){
        return;
    }
    const bool firstHeartbeat = !inputHeartbeatArmed;
    const bool heartbeatRecovered = inputHeartbeatStale;
    const bool uiSourceBecameStale =
            currentStatus.inputSourceUiFresh && !uiSourceFresh;
    const bool uiSourceRecovered =
            !currentStatus.inputSourceUiFresh && uiSourceFresh;
    if(!uiSourceFresh || !finiteArray(normalizedDirection)){
        requestedMotionMode = EndpointRemoteMotionMode::None;
        requestedDirection.fill(0.0);
    }
    else{
        requestedMotionMode =
                motionMode == EndpointRemoteMotionMode::Translation ||
                motionMode == EndpointRemoteMotionMode::YawLockedEulerRotation ?
                    motionMode : EndpointRemoteMotionMode::None;
        requestedDirection = normalizedVector(normalizedDirection);
    }
    lastInputUpdateUs = nowUs;
    currentStatus.inputSequence = sequence;
    currentStatus.inputAgeUs = 0;
    currentStatus.inputSourceUiFresh = uiSourceFresh;
    currentStatus.inputSourceUiAgeUs = std::max<qint64>(0, uiSourceAgeUs);
    currentStatus.requestedMotionMode = requestedMotionMode;
    inputHeartbeatStale = false;
    if(uiSourceBecameStale){
        currentStatus.message = QStringLiteral(
                    "上游输入源已失效，独立监督线程已锁存零方向；请将当前输入源回零后恢复");
    }
    else if(uiSourceRecovered){
        currentStatus.message = QStringLiteral(
                    "上游输入源已恢复并确认零方向；可继续接收新的遥控输入");
    }
    else if(heartbeatRecovered &&
            currentStatus.state == EndpointRemoteStatus::State::Running){
        currentStatus.message = QStringLiteral(
                    "末端遥控输入心跳已恢复；松开全部方向键时目标速度为零");
    }
    if(firstHeartbeat){
        inputHeartbeatArmed = true;
        if(currentStatus.state == EndpointRemoteStatus::State::WaitingForTrace){
            waitStartUs = nowUs;
            nextDueUs = nowUs;
            currentStatus.message = QStringLiteral(
                        "输入监督器初始零方向已同步，等待可靠的同帧八轴 Runtime Trace");
        }
    }
}

EndpointRemoteStep EndpointRemoteControl::step(
        const OnlineVelocityFeedback& feedback,
        qint64 nowUs,
        const EndpointRemoteTimingContext& timing)
{
    EndpointRemoteStep result;
    result.monotonicUs = nowUs;
    result.wallClockUs = feedback.wallClockUs;
    result.logicalFrameSequence = feedback.logicalFrameSequence;
    result.actualPosition = feedback.actualPosition;
    result.actualVelocity = feedback.actualVelocity;
    result.inputDirection = requestedDirection;
    result.requestedMotionMode = requestedMotionMode;
    result.activeMotionMode = activeMotionMode;
    result.actuationProfile = currentStatus.actuationProfile;
    if(!isActive()){
        return result;
    }

    currentStatus.latestWorkerLoopIntervalUs = timing.workerLoopIntervalUs;
    currentStatus.latestTraceReadDurationUs = timing.traceReadDurationUs;
    currentStatus.maximumTraceReadDurationUs = std::max(
                currentStatus.maximumTraceReadDurationUs,
                timing.traceReadDurationUs);
    currentStatus.latestPreDispatchDurationUs = timing.preDispatchDurationUs;
    currentStatus.maximumPreDispatchDurationUs = std::max(
                currentStatus.maximumPreDispatchDurationUs,
                timing.preDispatchDurationUs);
    currentStatus.previousWorkerLoopDurationUs =
            timing.previousWorkerLoopDurationUs;

    const bool ready = feedbackReady(feedback);
    if(ready){
        for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
            if(feedback.motorStateMachine[axis] == 4){
                continue;
            }
            const QString statusWordHex =
                    QString::number(feedback.motorStatusWord[axis], 16)
                    .rightJustified(4, QLatin1Char('0'))
                    .toUpper();
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "末端遥控同帧驱动状态异常：轴%1，0x6041=%2(0x%3)，解码状态=%4，要求=4(Operation enabled)，逻辑序号=%5；已在发出本周期速度命令前急停")
                    .arg(axis + 1)
                    .arg(feedback.motorStatusWord[axis])
                    .arg(statusWordHex)
                    .arg(feedback.motorStateMachine[axis])
                    .arg(feedback.logicalFrameSequence);
            setFault(result.reason);
            return result;
        }
    }
    result.traceValidationCompletedUs = endpointRemoteMonotonicNowUs();
    if(currentStatus.state == EndpointRemoteStatus::State::WaitingForTrace){
        // start() 只建立待启动会话。必须等主线程完成安全配置和界面收尾，
        // 再收到监督器对本会话发送的第一份零方向后，才允许切入 Running。
        // 因此心跳计时不可能在用户点击进入之前或启动收尾期间提前开始。
        if(!inputHeartbeatArmed){
            currentStatus.inputAgeUs = -1;
            return result;
        }
        if(!ready){
            if(waitStartUs > 0 &&
                    nowUs - waitStartUs > config.onlineVelocity.initialTraceWaitTimeoutUs){
                result.action = EndpointRemoteStep::Action::EmergencyStop;
                result.reason = QStringLiteral(
                            "末端遥控等待Runtime Trace超时：帧年龄=%1 us，上限=%2 us，FIFO追平=%3，来源/序号/时序/丢帧=%4/%5/%6/%7")
                        .arg(feedback.newestFrameAgeUs)
                        .arg(config.onlineVelocity.traceFeedbackDelayLimitUs())
                        .arg(feedback.fifoCaughtUp ? 1 : 0)
                        .arg(feedback.fromTrace ? 1 : 0)
                        .arg(feedback.frameSequenceValid ? 1 : 0)
                        .arg(feedback.timingReliable ? 1 : 0)
                        .arg(feedback.traceLost ? 1 : 0);
                setFault(result.reason);
            }
            return result;
        }
        const double stationaryThreshold = std::max(
                    config.onlineVelocity.startVelocityThreshold, 1.0e-9);
        for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
            if(std::fabs(feedback.actualVelocity[axis]) <= stationaryThreshold &&
                    std::fabs(feedback.tracedCommandVelocity[axis]) <=
                        stationaryThreshold){
                continue;
            }
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "末端遥控进入前同帧静止确认失败：轴%1实际/Trace指令速度=%2/%3，阈值=%4；未授权JOG命令")
                    .arg(axis + 1)
                    .arg(feedback.actualVelocity[axis], 0, 'f', 6)
                    .arg(feedback.tracedCommandVelocity[axis], 0, 'f', 6)
                    .arg(stationaryThreshold, 0, 'f', 6);
            setFault(result.reason);
            return result;
        }
        for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
            const double drift = std::fabs(
                        feedback.actualPosition[axis] -
                        config.preparedMotorPosition[axis]);
            if(drift > config.preparedMotorDriftTolerance){
                result.action = EndpointRemoteStep::Action::EmergencyStop;
                result.reason = QStringLiteral(
                            "进入遥控期间电机%1发生未授权移动：漂移=%2，允许=%3")
                        .arg(axis + 1)
                        .arg(drift, 0, 'f', 6)
                        .arg(config.preparedMotorDriftTolerance, 0, 'f', 6);
                setFault(result.reason);
                return result;
            }
            actualStartMotorPosition[axis] = feedback.actualPosition[axis];
        }
        actualStartCaptured = true;
        sessionStartUs = nowUs;
        nextDueUs = nowUs + config.onlineVelocity.periodUs;
        lastCommandDispatchUs = nowUs;
        // lastInputUpdateUs 始终来自独立输入监督线程，不再用控制循环时间
        // 伪造心跳；UI是否仍存活则由独立的租约状态明确传入。
        currentStatus.inputAgeUs = std::max<qint64>(
                    0, nowUs - lastInputUpdateUs);
        lastUsedFrameSequence = feedback.logicalFrameSequence;
        lastUsedFrameSequenceValid = true;
        currentStatus.state = EndpointRemoteStatus::State::Running;
        currentStatus.actuationProfile =
                EndpointRemoteActuationProfile::ArmedIdle;
        currentStatus.message = currentStatus.inputSourceUiFresh ?
                    QStringLiteral("末端遥控运行中；松开全部方向键时目标速度为零") :
                    QStringLiteral("末端遥控已进入零速运行，但上游输入源失效；请先将当前输入源回零");
        currentStatus.actualPosition = feedback.actualPosition;
        currentStatus.actualVelocity = feedback.actualVelocity;
        currentStatus.latestLogicalFrameSequence = feedback.logicalFrameSequence;
        currentStatus.referencePosition = actualStartMotorPosition;
        result.diagnosticMessage = QStringLiteral(
                    "末端遥控调度已启动：在线周期=%1 us，ControlWorker本次循环间隔=%2 us，Trace读取=%3 us，循环入口至遥控调度=%4 us，Trace帧年龄=%5 us，FIFO有效/空闲=%6/%7，逻辑帧=%8")
                .arg(config.onlineVelocity.periodUs)
                .arg(timing.workerLoopIntervalUs)
                .arg(timing.traceReadDurationUs)
                .arg(timing.preDispatchDurationUs)
                .arg(feedback.newestFrameAgeUs)
                .arg(feedback.fifoValidNum)
                .arg(feedback.fifoFreeNum)
                .arg(feedback.logicalFrameSequence);
        return result;
    }

    if(nowUs < nextDueUs){
        return result;
    }
    if(nextDueUs <= 0){
        nextDueUs = nowUs;
    }
    const qint64 lateUs = std::max<qint64>(0, nowUs - nextDueUs);
    const qint64 skipped = lateUs / config.onlineVelocity.periodUs;
    const qint64 commandIntervalUs = lastCommandDispatchUs > 0 ?
                std::max<qint64>(0, nowUs - lastCommandDispatchUs) : 0;
    currentStatus.latestScheduleLatenessUs = lateUs;
    currentStatus.maximumScheduleLatenessUs = std::max(
                currentStatus.maximumScheduleLatenessUs, lateUs);
    currentStatus.latestCommandIntervalUs = commandIntervalUs;
    currentStatus.maximumCommandIntervalUs = std::max(
                currentStatus.maximumCommandIntervalUs,
                commandIntervalUs);
    nextDueUs += (skipped + 1) * config.onlineVelocity.periodUs;

    if(!ready ||
            (lastUsedFrameSequenceValid &&
             feedback.logicalFrameSequence <= lastUsedFrameSequence)){
        result.action = EndpointRemoteStep::Action::EmergencyStop;
        result.reason = QStringLiteral(
                    "末端遥控运行周期未取得新的可靠同帧反馈：帧年龄=%1 us，上限=%2 us，FIFO追平=%3，有效/空闲=%4/%5，本次帧数=%6，逻辑序号=%7，上次已用=%8；ControlWorker循环间隔=%9 us，上轮完整循环=%10 us，Trace读取=%11 us，循环入口至遥控调度=%12 us，调度迟到=%13 us，距上次成功命令=%14 us")
                .arg(feedback.newestFrameAgeUs)
                .arg(config.onlineVelocity.traceFeedbackDelayLimitUs())
                .arg(feedback.fifoCaughtUp ? 1 : 0)
                .arg(feedback.fifoValidNum)
                .arg(feedback.fifoFreeNum)
                .arg(feedback.frameCount)
                .arg(feedback.logicalFrameSequence)
                .arg(lastUsedFrameSequence)
                .arg(timing.workerLoopIntervalUs)
                .arg(timing.previousWorkerLoopDurationUs)
                .arg(timing.traceReadDurationUs)
                .arg(timing.preDispatchDurationUs)
                .arg(lateUs)
                .arg(commandIntervalUs);
        setFault(result.reason);
        return result;
    }
    lastUsedFrameSequence = feedback.logicalFrameSequence;
    lastUsedFrameSequenceValid = true;
    currentStatus.latestLogicalFrameSequence = feedback.logicalFrameSequence;

    if(lastInputUpdateUs <= 0 ||
            nowUs - lastInputUpdateUs > config.inputHeartbeatTimeoutUs){
        currentStatus.inputAgeUs =
                lastInputUpdateUs > 0 ? nowUs - lastInputUpdateUs : -1;
        const double stationaryThreshold = std::max(
                    config.onlineVelocity.startVelocityThreshold, 1.0e-9);
        const bool requestedStationary =
                vectorNorm(requestedDirection) <= 1.0e-12;
        const bool endpointStationary =
                vectorNorm(committedEffectiveVelocity) <= stationaryThreshold &&
                vectorNorm(committedEffectiveEulerRate) <=
                    kEndpointRemoteAngularVelocityEpsilonRadPerSec;
        const bool motorCommandStationary = std::all_of(
                    lastCommandVelocity.begin(),
                    lastCommandVelocity.end(),
                    [stationaryThreshold](double value){
            return std::isfinite(value) &&
                    std::fabs(value) <= stationaryThreshold;
        });
        if(!requestedStationary || !endpointStationary ||
                !motorCommandStationary){
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "末端遥控输入心跳超时%1 us（上限%2 us，输入序号%3，已执行命令%4次，请求方向范数=%5，有效末端速度范数=%6 mm/s，电机命令最大绝对值=%7），已停止以防继续保持旧的非零速度")
                    .arg(currentStatus.inputAgeUs)
                    .arg(config.inputHeartbeatTimeoutUs)
                    .arg(currentStatus.inputSequence)
                    .arg(currentStatus.commandCount)
                    .arg(vectorNorm(requestedDirection), 0, 'f', 6)
                    .arg(vectorNorm(committedEffectiveVelocity), 0, 'f', 6)
                    .arg(std::fabs(*std::max_element(
                             lastCommandVelocity.begin(),
                             lastCommandVelocity.end(),
                             [](double left, double right){
                        return std::fabs(left) < std::fabs(right);
                    })), 0, 'f', 6);
            setFault(result.reason);
            return result;
        }

        // 独立输入监督线程如果异常停顿，只有在请求、有效末端速度和电机
        // 速度均为零时才允许保持零速等待恢复；非零状态仍立即故障停止。
        requestedMotionMode = EndpointRemoteMotionMode::None;
        requestedDirection.fill(0.0);
        if(!inputHeartbeatStale){
            inputHeartbeatStale = true;
            currentStatus.message = QStringLiteral(
                        "输入监督心跳暂时超时，但当前请求/末端/电机速度均为零，保持零速等待监督线程恢复");
            result.diagnosticWarning = true;
            result.diagnosticMessage = QStringLiteral(
                        "末端遥控独立监督心跳延迟：年龄=%1 us，上限=%2 us，输入序号=%3，当前确认静止，未故障退出；请检查输入监督线程调度或会话生命周期")
                    .arg(currentStatus.inputAgeUs)
                    .arg(config.inputHeartbeatTimeoutUs)
                    .arg(currentStatus.inputSequence);
        }
    }

    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(feedback.actualPosition[axis] < config.motorPositionMinimum[axis] ||
                feedback.actualPosition[axis] > config.motorPositionMaximum[axis]){
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral("末端遥控电机%1实际位置已越过软件限位")
                    .arg(axis + 1);
            setFault(result.reason);
            return result;
        }
    }

    if(skipped > 0){
        currentStatus.missedCycleCount += static_cast<quint64>(skipped);
        const double stationaryThreshold = std::max(
                    config.onlineVelocity.startVelocityThreshold, 1.0e-9);
        const bool endpointStationary =
                vectorNorm(committedEffectiveVelocity) <= stationaryThreshold &&
                vectorNorm(committedEffectiveEulerRate) <=
                    kEndpointRemoteAngularVelocityEpsilonRadPerSec;
        const bool motorCommandStationary = std::all_of(
                    lastCommandVelocity.begin(),
                    lastCommandVelocity.end(),
                    [stationaryThreshold](double value){
            return std::isfinite(value) &&
                    std::fabs(value) <= stationaryThreshold;
        });
        const bool safelyStationary =
                endpointStationary && motorCommandStationary;
        const auto scheduleDetail = [&](){
            return QStringLiteral(
                        "漏拍=%1，迟到=%2 us，周期=%3 us，距上次成功命令=%4 us，ControlWorker循环间隔=%5 us，上轮完整循环=%6 us，Trace读取=%7 us，循环入口至遥控调度=%8 us，命令API最近/最大=%9/%10 us，Trace帧年龄=%11 us，FIFO有效/空闲=%12/%13，逻辑帧=%14，输入年龄=%15 us，命令数=%16")
                    .arg(skipped)
                    .arg(lateUs)
                    .arg(config.onlineVelocity.periodUs)
                    .arg(commandIntervalUs)
                    .arg(timing.workerLoopIntervalUs)
                    .arg(timing.previousWorkerLoopDurationUs)
                    .arg(timing.traceReadDurationUs)
                    .arg(timing.preDispatchDurationUs)
                    .arg(currentStatus.latestCommandApiUs)
                    .arg(currentStatus.maximumCommandApiUs)
                    .arg(feedback.newestFrameAgeUs)
                    .arg(feedback.fifoValidNum)
                    .arg(feedback.fifoFreeNum)
                    .arg(feedback.logicalFrameSequence)
                    .arg(std::max<qint64>(0, nowUs - lastInputUpdateUs))
                    .arg(currentStatus.commandCount);
        };

        if(!safelyStationary &&
                skipped > kEndpointRemoteRecoverableMissedPeriods){
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "末端遥控调度严重超期，非零速度下不允许补推多个周期：%1；有效末端速度=[%2,%3,%4] mm/s")
                    .arg(scheduleDetail())
                    .arg(committedEffectiveVelocity[0], 0, 'f', 6)
                    .arg(committedEffectiveVelocity[1], 0, 'f', 6)
                    .arg(committedEffectiveVelocity[2], 0, 'f', 6);
            setFault(result.reason);
            return result;
        }

        QString recoveryMode = QStringLiteral("静止状态仅重同步调度时钟");
        if(!safelyStationary){
            // 漏拍期间硬件仍保持上一条非零速度。用上一有效末端速度补推恰好
            // 一个标称周期，使开环期望位姿追上真实经过的时间；不伪造一条
            // 未实际下发的速度命令，也不改写lastCommandVelocity。
            const Candidate catchUp = buildCandidate(
                        committedEffectiveVelocity,
                        committedEffectiveEulerRate,
                        false);
            if(!catchUp.valid){
                result.action = EndpointRemoteStep::Action::EmergencyStop;
                result.reason = QStringLiteral(
                            "末端遥控单周期漏拍无法安全追赶：%1；补推失败=%2")
                        .arg(scheduleDetail(), catchUp.errorMessage);
                setFault(result.reason);
                return result;
            }
            committedPose = catchUp.pose;
            committedRelativeMotorThetaRad = catchUp.relativeMotorThetaRad;
            committedKinematicsState = catchUp.kinematicsState;
            currentStatus.desiredPose = committedPose;
            currentStatus.referencePosition = catchUp.referencePosition;
            updateOrientationStatus(committedPose);
            if(!stoppingTrajectoryInsideWorkspace(committedEffectiveVelocity) ||
                    !stoppingRotationInsideBounds(
                        committedEffectiveEulerRate)){
                result.action = EndpointRemoteStep::Action::EmergencyStop;
                result.reason = QStringLiteral(
                            "末端遥控单周期漏拍补推后已无可靠制动距离：%1；补推期望位姿=[%2,%3,%4] mm")
                        .arg(scheduleDetail())
                        .arg(committedPose[0], 0, 'f', 3)
                        .arg(committedPose[1], 0, 'f', 3)
                        .arg(committedPose[2], 0, 'f', 3);
                setFault(result.reason);
                return result;
            }
            recoveryMode = QStringLiteral(
                        "按上一有效末端速度补推1个周期");
        }
        currentStatus.schedulingRecoveryCount++;
        const bool recoveryDiagnosticDue =
                lastSchedulingRecoveryDiagnosticUs <= 0 ||
                nowUs - lastSchedulingRecoveryDiagnosticUs >=
                    kSchedulingRecoveryDiagnosticIntervalUs;
        if(recoveryDiagnosticDue){
            const quint64 suppressedCount =
                    suppressedSchedulingRecoveryDiagnosticCount;
            result.diagnosticWarning = true;
            result.diagnosticMessage = QStringLiteral(
                        "末端遥控调度抖动已安全追赶（严格限频汇总）：%1；恢复方式=%2；累计恢复/漏拍=%3/%4；距上次日志期间已抑制重复日志=%5；限频窗口=%6 ms")
                    .arg(scheduleDetail(), recoveryMode)
                    .arg(currentStatus.schedulingRecoveryCount)
                    .arg(currentStatus.missedCycleCount)
                    .arg(suppressedCount)
                    .arg(kSchedulingRecoveryDiagnosticIntervalUs / 1000);
            lastSchedulingRecoveryDiagnosticUs = nowUs;
            suppressedSchedulingRecoveryDiagnosticCount = 0;
        }
        else{
            ++suppressedSchedulingRecoveryDiagnosticCount;
        }
    }

    result.planningStartedUs = endpointRemoteMonotonicNowUs();
    const bool translationStopped =
            vectorNorm(committedEffectiveVelocity) <=
                kEndpointRemoteBoundaryVelocityEpsilonMmPerSec;
    const bool rotationStopped =
            vectorNorm(committedEffectiveEulerRate) <=
                kEndpointRemoteAngularVelocityEpsilonRadPerSec;
    if(translationStopped && rotationStopped){
        // 模式切换只发生在两类速度都已真正归零的规划周期开始处。
        activeMotionMode = requestedMotionMode;
    }
    currentStatus.requestedMotionMode = requestedMotionMode;
    currentStatus.activeMotionMode = activeMotionMode;
    result.requestedMotionMode = requestedMotionMode;
    result.activeMotionMode = activeMotionMode;

    const auto currentBoundaryDistance = boundaryDistances(committedPose);
    std::array<EndpointRemoteBoundaryState,
               kEndpointRemoteBoundaryDirectionCount> nextBoundaryState =
            advanceBoundaryStates(committedPose,
                                  committedEffectiveVelocity,
                                  boundaryState);

    std::array<double, 3> targetVelocity{};
    std::array<double, 3> targetEulerRate{};
    const bool translationRequested =
            requestedMotionMode == EndpointRemoteMotionMode::Translation &&
            activeMotionMode == EndpointRemoteMotionMode::Translation;
    const bool rotationRequested =
            requestedMotionMode ==
                EndpointRemoteMotionMode::YawLockedEulerRotation &&
            activeMotionMode ==
                EndpointRemoteMotionMode::YawLockedEulerRotation;
    const bool translationOrientationSafe =
            orientationInsideBounds(committedPose, true);
    currentStatus.translationBlockedByOrientation =
            !translationOrientationSafe;
    if(translationRequested && !translationOrientationSafe){
        // 姿态未回正时，不只是界面告警：平动目标保持为全零，后续的加速度
        // 限制只负责将可能残留的平动速度减到零，不会接受新的平动速度指令。
        currentStatus.message = QStringLiteral(
                    "平动指令已拒绝：姿态未回正，当前只能使用转动模式恢复到平动安全阈值内");
    }
    if(translationRequested && translationOrientationSafe){
        for(int dim = 0; dim < 3; ++dim){
            targetVelocity[dim] =
                    requestedDirection[dim] * config.translationSpeedMmPerSec;
        }
    }
    if(rotationRequested){
        targetEulerRate[0] =
                requestedDirection[0] * config.maximumAngularSpeedRadPerSec;
        targetEulerRate[1] =
                requestedDirection[1] * config.maximumAngularSpeedRadPerSec;
        // 手柄第三分量始终表示Rz欧拉角速度，并明确锁定为零。真实全局角速度
        // 的Z分量由当前Ry自动生成，只用于抵消欧拉角耦合。
        targetEulerRate[2] = 0.0;
    }
    const bool boundaryBrakingActive = std::any_of(
                nextBoundaryState.begin(),
                nextBoundaryState.end(),
                [](EndpointRemoteBoundaryState state){
        return state == EndpointRemoteBoundaryState::Braking;
    });
    if(boundaryBrakingActive){
        // 一旦进入制动阶段，先把完整末端速度向量减到零，避免切向加速
        // 分走合成加速度预算。停稳后再保留切向和向内指令。
        targetVelocity.fill(0.0);
    }
    else{
        for(int direction = 0;
            direction < kEndpointRemoteBoundaryDirectionCount;
            ++direction){
            const int dim = boundaryDimension(direction);
            const double sign = boundarySign(direction);
            const double outwardTarget = sign * targetVelocity[dim];
            if(outwardTarget <= 0.0){
                continue;
            }
            if(nextBoundaryState[direction] ==
                    EndpointRemoteBoundaryState::BlockedOutward){
                // 只删除被阻塞面的朝外分量；组合命令中的切向和向内分量保留。
                targetVelocity[dim] = 0.0;
                continue;
            }
            if(currentBoundaryDistance[direction] <=
                    config.softBoundaryMarginMm){
                const double dtSec = static_cast<double>(
                            config.onlineVelocity.periodUs) / 1000000.0;
                const double guardedTravelTimeSec =
                        endpointRemoteGuardedTravelPeriods() * dtSec;
                const double brakingOffset =
                        config.translationAccelerationMmPerSec2 *
                        guardedTravelTimeSec;
                // 求解 v^2/(2a) + v*T_guard <= d 的正根。这样软边界
                // 限速与后面的完整停止包络使用同一周期预算，不会在20 ms
                // 下仍按理想的纯制动距离给出过大的朝外速度。
                const double allowedOutwardSpeed = std::max(
                            0.0,
                            std::sqrt(std::max(
                                0.0,
                                brakingOffset * brakingOffset +
                                2.0 *
                                    config.translationAccelerationMmPerSec2 *
                                    currentBoundaryDistance[direction])) -
                            brakingOffset);
                if(outwardTarget > allowedOutwardSpeed){
                    targetVelocity[dim] = sign * allowedOutwardSpeed;
                }
            }
        }
    }

    std::array<double, 3> desiredEffective =
            accelerationLimitedVelocity(targetVelocity);
    std::array<bool, kEndpointRemoteBoundaryDirectionCount> violatedFaces{};
    bool voxelAngleLimitViolated = false;
    currentStatus.voxelAngleBoundaryBrakingActive = false;
    if(!stoppingTrajectoryInsideWorkspace(desiredEffective,
                                          &violatedFaces,
                                          &voxelAngleLimitViolated)){
        for(int direction = 0;
            direction < kEndpointRemoteBoundaryDirectionCount;
            ++direction){
            if(violatedFaces[direction] &&
                    nextBoundaryState[direction] !=
                    EndpointRemoteBoundaryState::BlockedOutward){
                nextBoundaryState[direction] =
                        EndpointRemoteBoundaryState::Braking;
            }
        }
        if(voxelAngleLimitViolated){
            currentStatus.voxelAngleBoundaryBrakingActive = true;
            currentStatus.message = QStringLiteral(
                        "目标方向前方体素的Rx/Ry允许范围更小，正在按平移加速度上限制动");
        }
        targetVelocity.fill(0.0);
        desiredEffective = accelerationLimitedVelocity(targetVelocity);
        std::array<bool, kEndpointRemoteBoundaryDirectionCount>
                brakingViolatedFaces{};
        bool brakingVoxelAngleLimitViolated = false;
        if(!stoppingTrajectoryInsideWorkspace(desiredEffective,
                                              &brakingViolatedFaces,
                                              &brakingVoxelAngleLimitViolated)){
            QStringList unsafeFaces;
            for(int direction = 0;
                direction < kEndpointRemoteBoundaryDirectionCount;
                ++direction){
                if(brakingViolatedFaces[direction]){
                    unsafeFaces.append(boundaryDirectionText(direction));
                }
            }
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = brakingVoxelAngleLimitViolated ?
                        QStringLiteral(
                            "末端遥控最大减速度已无法在相邻体素姿态边界前停稳，期望位姿=[%1,%2,%3] mm")
                            .arg(committedPose[0], 0, 'f', 3)
                            .arg(committedPose[1], 0, 'f', 3)
                            .arg(committedPose[2], 0, 'f', 3) :
                        QStringLiteral(
                            "末端遥控最大减速度已无法在平动硬边界前停稳：方向=%1，期望位姿=[%2,%3,%4] mm")
                            .arg(unsafeFaces.join(QStringLiteral(",")))
                            .arg(committedPose[0], 0, 'f', 3)
                            .arg(committedPose[1], 0, 'f', 3)
                            .arg(committedPose[2], 0, 'f', 3);
            setFault(result.reason);
            return result;
        }
    }

    std::array<double, 3> desiredEffectiveEulerRate =
            accelerationLimitedEulerRate(targetEulerRate);
    currentStatus.angularBoundaryBrakingActive = false;
    if(!stoppingRotationInsideBounds(
                desiredEffectiveEulerRate)){
        // 与平动硬边界一致：先按最大设定角加速度制动，不允许继续朝越界方向加速。
        targetEulerRate.fill(0.0);
        currentStatus.angularBoundaryBrakingActive = true;
        desiredEffectiveEulerRate =
                accelerationLimitedEulerRate(targetEulerRate);
        if(!stoppingRotationInsideBounds(
                    desiredEffectiveEulerRate)){
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "末端遥控按最大角减速度已无法在当前体素Rx/Ry姿态硬边界前停稳，已急停");
            setFault(result.reason);
            return result;
        }
    }

    Candidate accepted = buildCandidate(
                desiredEffective,
                desiredEffectiveEulerRate);
    if(!accepted.valid){
        // 电机速度/加速度约束不能逐轴裁剪，否则会破坏平台方向。沿任务空间
        // 速度变化统一缩放，寻找所有八轴都可接受的最大比例。
        const Candidate base = buildCandidate(
                    committedEffectiveVelocity,
                    committedEffectiveEulerRate);
        if(!base.valid ||
                !stoppingTrajectoryInsideWorkspace(committedEffectiveVelocity) ||
                !stoppingRotationInsideBounds(
                    committedEffectiveEulerRate)){
            result.action = EndpointRemoteStep::Action::EmergencyStop;
            result.reason = QStringLiteral("末端遥控无法保持连续安全速度：%1；零变化检查：%2")
                    .arg(accepted.errorMessage, base.errorMessage);
            setFault(result.reason);
            return result;
        }
        double lower = 0.0;
        double upper = 1.0;
        accepted = base;
        for(int iteration = 0; iteration < 20; ++iteration){
            const double ratio = 0.5 * (lower + upper);
            std::array<double, 3> trialVelocity{};
            std::array<double, 3> trialAngularVelocity{};
            for(int dim = 0; dim < 3; ++dim){
                trialVelocity[dim] = committedEffectiveVelocity[dim] +
                        (desiredEffective[dim] - committedEffectiveVelocity[dim]) * ratio;
                trialAngularVelocity[dim] =
                        committedEffectiveEulerRate[dim] +
                        (desiredEffectiveEulerRate[dim] -
                         committedEffectiveEulerRate[dim]) * ratio;
            }
            const Candidate trial = buildCandidate(trialVelocity,
                                                   trialAngularVelocity);
            if(trial.valid &&
                    stoppingTrajectoryInsideWorkspace(trialVelocity) &&
                    stoppingRotationInsideBounds(trialAngularVelocity)){
                lower = ratio;
                accepted = trial;
            }
            else{
                upper = ratio;
            }
        }
    }

    if(!stoppingTrajectoryInsideWorkspace(accepted.effectiveVelocity) ||
            !stoppingRotationInsideBounds(
                accepted.effectiveEulerRate)){
        result.action = EndpointRemoteStep::Action::EmergencyStop;
        result.reason = QStringLiteral("末端遥控八轴约束缩放后失去安全制动轨迹，已急停");
        setFault(result.reason);
        return result;
    }

    nextBoundaryState = advanceBoundaryStates(
                accepted.pose, accepted.effectiveVelocity,
                nextBoundaryState);
    accepted.boundaryState = nextBoundaryState;

    result.planningCompletedUs = endpointRemoteMonotonicNowUs();
    const qint64 planningUs = std::max<qint64>(
                0, result.planningCompletedUs - result.planningStartedUs);
    currentStatus.latestPlanningUs = planningUs;
    currentStatus.maximumPlanningUs = std::max(
                currentStatus.maximumPlanningUs, planningUs);

    const bool commandIsZero = std::all_of(
                accepted.commandVelocity.begin(),
                accepted.commandVelocity.end(),
                [](double value){
        return std::isfinite(value) && std::fabs(value) <= 1.0e-9;
    });
    const EndpointRemoteActuationProfile actuationProfile =
            currentStatus.actuationProfile;
    if(commandIsZero &&
            (actuationProfile == EndpointRemoteActuationProfile::ArmedIdle ||
             actuationProfile == EndpointRemoteActuationProfile::ZeroHolding)){
        // 尚未启动JOG，或已经成功下发过一次零速后，连续零目标只是控制器内部
        // 保持状态。不要为它排队HardwareThread任务，也不要把它计为速度命令。
        pendingCandidateValid = false;
        pendingCommandIntent = EndpointRemoteCommandIntent::None;
        currentStatus.targetVelocityMmPerSec = targetVelocity;
        currentStatus.effectiveVelocityMmPerSec = accepted.effectiveVelocity;
        currentStatus.targetEulerRateRadPerSec = targetEulerRate;
        currentStatus.effectiveEulerRateRadPerSec =
                accepted.effectiveEulerRate;
        currentStatus.effectiveGlobalAngularVelocityRadPerSec =
                accepted.globalAngularVelocity;
        currentStatus.effectiveGlobalAngularAccelerationRadPerSec2 =
                accepted.globalAngularAccelerationRadPerSec2;
        currentStatus.referencePosition = accepted.referencePosition;
        currentStatus.commandVelocity.fill(0.0);
        currentStatus.inputAgeUs = std::max<qint64>(
                    0, nowUs - lastInputUpdateUs);
        updateBoundaryStatus(accepted.pose, accepted.boundaryState);
        updateOrientationStatus(accepted.pose);
        return result;
    }

    pendingCandidate = accepted;
    pendingCandidateValid = true;
    pendingPriorActuationProfile = actuationProfile;
    pendingCommandIntent = commandIsZero ?
                EndpointRemoteCommandIntent::EnterZeroHolding :
                EndpointRemoteCommandIntent::StartOrUpdateJog;
    currentStatus.actuationProfile = commandIsZero ?
                EndpointRemoteActuationProfile::Stopping :
                (actuationProfile == EndpointRemoteActuationProfile::ArmedIdle ||
                 actuationProfile == EndpointRemoteActuationProfile::ZeroHolding ?
                     EndpointRemoteActuationProfile::StartingJog :
                     EndpointRemoteActuationProfile::JogActive);
    result.action = EndpointRemoteStep::Action::CommandVelocity;
    result.actuationProfile = currentStatus.actuationProfile;
    result.commandIntent = pendingCommandIntent;
    result.desiredPose = accepted.pose;
    result.targetVelocityMmPerSec = targetVelocity;
    result.effectiveVelocityMmPerSec = accepted.effectiveVelocity;
    result.targetEulerRateRadPerSec = targetEulerRate;
    result.effectiveEulerRateRadPerSec = accepted.effectiveEulerRate;
    result.effectiveGlobalAngularVelocityRadPerSec =
            accepted.globalAngularVelocity;
    result.effectiveGlobalAngularAccelerationRadPerSec2 =
            accepted.globalAngularAccelerationRadPerSec2;
    result.referencePosition = accepted.referencePosition;
    result.commandVelocity = accepted.commandVelocity;
    currentStatus.targetVelocityMmPerSec = targetVelocity;
    currentStatus.targetEulerRateRadPerSec = targetEulerRate;
    currentStatus.inputAgeUs = std::max<qint64>(
                0, nowUs - lastInputUpdateUs);
    return result;
}

void EndpointRemoteControl::noteCommandResult(
        const EndpointRemoteStep& stepResult,
        bool commandOk,
        qint64 apiDurationUs,
        qint64 fullCycleDurationUs,
        const QString& commandFailureReason,
        const OnlineVelocityFeedback& commandFeedback)
{
    currentStatus.latestCommandApiUs = apiDurationUs;
    currentStatus.maximumCommandApiUs =
            std::max(currentStatus.maximumCommandApiUs, apiDurationUs);
    currentStatus.latestFullCycleUs = fullCycleDurationUs;
    currentStatus.maximumFullCycleUs =
            std::max(currentStatus.maximumFullCycleUs, fullCycleDurationUs);
    currentStatus.actualPosition = commandFeedback.actualPosition;
    currentStatus.actualVelocity = commandFeedback.actualVelocity;
    if(stepResult.action != EndpointRemoteStep::Action::CommandVelocity){
        pendingCandidateValid = false;
        return;
    }
    if(!commandOk || !pendingCandidateValid){
        pendingCandidateValid = false;
        pendingCommandIntent = EndpointRemoteCommandIntent::None;
        setFault(commandFailureReason.isEmpty() ?
                     QStringLiteral("末端遥控八轴速度下发失败（底层未返回明确原因）") :
                     commandFailureReason);
        return;
    }

    committedPose = pendingCandidate.pose;
    committedEffectiveVelocity = pendingCandidate.effectiveVelocity;
    committedEffectiveEulerRate = pendingCandidate.effectiveEulerRate;
    committedGlobalAngularVelocity =
            pendingCandidate.globalAngularVelocity;
    committedRelativeMotorThetaRad = pendingCandidate.relativeMotorThetaRad;
    committedKinematicsState = pendingCandidate.kinematicsState;
    lastCommandVelocity = pendingCandidate.commandVelocity;
    lastCommandDispatchUs = stepResult.monotonicUs;
    currentStatus.desiredPose = committedPose;
    currentStatus.effectiveVelocityMmPerSec = committedEffectiveVelocity;
    currentStatus.effectiveEulerRateRadPerSec =
            committedEffectiveEulerRate;
    currentStatus.effectiveGlobalAngularVelocityRadPerSec =
            committedGlobalAngularVelocity;
    currentStatus.effectiveGlobalAngularAccelerationRadPerSec2 =
            pendingCandidate.globalAngularAccelerationRadPerSec2;
    currentStatus.activeMotionMode = activeMotionMode;
    currentStatus.referencePosition = pendingCandidate.referencePosition;
    currentStatus.commandVelocity = pendingCandidate.commandVelocity;
    updateBoundaryStatus(committedPose, pendingCandidate.boundaryState);
    updateOrientationStatus(committedPose);
    currentStatus.actuationProfile =
            pendingCommandIntent ==
                EndpointRemoteCommandIntent::EnterZeroHolding ?
                EndpointRemoteActuationProfile::ZeroHolding :
                EndpointRemoteActuationProfile::JogActive;
    if(commandFeedback.frameSequenceValid){
        lastUsedFrameSequence = commandFeedback.logicalFrameSequence;
        lastUsedFrameSequenceValid = true;
        currentStatus.latestLogicalFrameSequence =
                commandFeedback.logicalFrameSequence;
    }
    currentStatus.commandCount++;
    currentStatus.elapsedSec = sessionStartUs > 0 ?
                static_cast<double>(stepResult.monotonicUs - sessionStartUs) / 1000000.0 :
                0.0;
    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
}

void EndpointRemoteControl::noteCommandDeferred(
        const EndpointRemoteStep& stepResult,
        const OnlineVelocityFeedback& commandFeedback,
        const QString& reason)
{
    Q_UNUSED(stepResult);
    currentStatus.actualPosition = commandFeedback.actualPosition;
    currentStatus.actualVelocity = commandFeedback.actualVelocity;
    currentStatus.message = reason.isEmpty() ?
                QStringLiteral("末端遥控启动JOG前等待下一份5 ms内的新鲜同帧Trace") :
                reason;
}

bool EndpointRemoteControl::cancelPreparedCommandIfInputChanged(
        const EndpointRemoteStep& preparedStep,
        qint64 nowUs)
{
    if(!pendingCandidateValid ||
            preparedStep.action != EndpointRemoteStep::Action::CommandVelocity ||
            currentStatus.actuationProfile !=
                EndpointRemoteActuationProfile::StartingJog){
        return false;
    }
    bool sameDirection = currentStatus.inputSourceUiFresh &&
            preparedStep.requestedMotionMode == requestedMotionMode;
    for(int dim = 0; sameDirection && dim < 3; ++dim){
        sameDirection = std::fabs(
                    preparedStep.inputDirection[dim] - requestedDirection[dim]) <=
                1.0e-12;
    }
    if(sameDirection){
        return false;
    }

    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
    currentStatus.actuationProfile = pendingPriorActuationProfile;
    currentStatus.commandVelocity = lastCommandVelocity;
    nextDueUs = nowUs;
    currentStatus.message = QStringLiteral(
                "JOG启动前输入方向或上游输入源状态已变化，已丢弃未下发命令并等待重新规划");
    return true;
}

void EndpointRemoteControl::stop(bool fault, const QString& reason)
{
    if(!isActive() && !isPrepared()){
        return;
    }
    requestedMotionMode = EndpointRemoteMotionMode::None;
    activeMotionMode = EndpointRemoteMotionMode::None;
    requestedDirection.fill(0.0);
    committedEffectiveVelocity.fill(0.0);
    committedEffectiveEulerRate.fill(0.0);
    committedGlobalAngularVelocity.fill(0.0);
    lastCommandVelocity.fill(0.0);
    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
    inputHeartbeatArmed = false;
    inputHeartbeatStale = false;
    lastCommandDispatchUs = 0;
    currentStatus.state = fault ?
                EndpointRemoteStatus::State::Fault :
                EndpointRemoteStatus::State::Stopped;
    currentStatus.actuationProfile = fault ?
                EndpointRemoteActuationProfile::Faulted :
                EndpointRemoteActuationProfile::Disarmed;
    currentStatus.message = reason;
    currentStatus.targetVelocityMmPerSec.fill(0.0);
    currentStatus.effectiveVelocityMmPerSec.fill(0.0);
    currentStatus.targetEulerRateRadPerSec.fill(0.0);
    currentStatus.effectiveEulerRateRadPerSec.fill(0.0);
    currentStatus.effectiveGlobalAngularVelocityRadPerSec.fill(0.0);
    currentStatus.effectiveGlobalAngularAccelerationRadPerSec2 = 0.0;
    currentStatus.requestedMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.activeMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.angularBoundaryBrakingActive = false;
    currentStatus.voxelAngleBoundaryBrakingActive = false;
    currentStatus.commandVelocity.fill(0.0);
}

bool EndpointRemoteControl::isActive() const
{
    return currentStatus.state == EndpointRemoteStatus::State::WaitingForTrace ||
            currentStatus.state == EndpointRemoteStatus::State::Running;
}

bool EndpointRemoteControl::isPrepared() const
{
    return currentStatus.state == EndpointRemoteStatus::State::Prepared;
}

const EndpointRemoteConfig& EndpointRemoteControl::currentConfig() const
{
    return config;
}

EndpointRemoteStatus EndpointRemoteControl::status() const
{
    return currentStatus;
}

bool EndpointRemoteControl::feedbackReady(const OnlineVelocityFeedback& feedback) const
{
    const qint64 delayLimitUs =
            config.onlineVelocity.traceFeedbackDelayLimitUs();
    return feedback.fromTrace &&
            feedback.frameSequenceValid &&
            feedback.timingReliable &&
            feedback.fifoCaughtUp &&
            !feedback.traceLost &&
            feedback.monotonicUs > 0 &&
            delayLimitUs > 0 &&
            feedback.newestFrameAgeUs >= 0 &&
            feedback.newestFrameAgeUs <= delayLimitUs &&
            finiteArray(feedback.actualPosition) &&
            finiteArray(feedback.actualVelocity) &&
            finiteArray(feedback.tracedCommandVelocity);
}

std::array<double, 3> EndpointRemoteControl::accelerationLimitedVelocity(
        const std::array<double, 3>& targetVelocity) const
{
    std::array<double, 3> result = committedEffectiveVelocity;
    std::array<double, 3> velocityDelta{};
    for(int dim = 0; dim < 3; ++dim){
        velocityDelta[dim] =
                targetVelocity[dim] - committedEffectiveVelocity[dim];
    }
    const double deltaNorm = vectorNorm(velocityDelta);
    const double dtSec =
            static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    const double maxVelocityDelta =
            config.translationAccelerationMmPerSec2 * dtSec;
    const double deltaScale =
            deltaNorm > maxVelocityDelta && deltaNorm > 0.0 ?
                maxVelocityDelta / deltaNorm : 1.0;
    for(int dim = 0; dim < 3; ++dim){
        result[dim] += velocityDelta[dim] * deltaScale;
    }
    return result;
}

std::array<double, 3> EndpointRemoteControl::accelerationLimitedEulerRate(
        const std::array<double, 3>& targetEulerRate) const
{
    std::array<double, 3> boundedTarget = targetEulerRate;
    boundedTarget[2] = 0.0;
    const double targetNorm = vectorNorm(boundedTarget);
    if(targetNorm > config.maximumAngularSpeedRadPerSec &&
            targetNorm > 0.0){
        const double scale =
                config.maximumAngularSpeedRadPerSec / targetNorm;
        boundedTarget[0] *= scale;
        boundedTarget[1] *= scale;
    }
    std::array<double, 3> result = committedEffectiveEulerRate;
    std::array<double, 3> velocityDelta{};
    for(int dim = 0; dim < 3; ++dim){
        velocityDelta[dim] = boundedTarget[dim] -
                committedEffectiveEulerRate[dim];
    }
    const double deltaNorm = vectorNorm(velocityDelta);
    const double dtSec =
            static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    const double maxVelocityDelta =
            config.maximumAngularAccelerationRadPerSec2 * dtSec;
    const double deltaScale =
            deltaNorm > maxVelocityDelta && deltaNorm > 0.0 ?
                maxVelocityDelta / deltaNorm : 1.0;
    for(int dim = 0; dim < 3; ++dim){
        result[dim] += velocityDelta[dim] * deltaScale;
    }
    result[2] = 0.0;

    // 欧拉角速度变化量满足上限后，再检查换算得到的真实全局角速度和
    // 真实角加速度。复合Rx/Ry运动会产生|Rx_dot*Ry_dot|的几何角加速度，
    // 因此一般需要比单纯的欧拉角速度增量再保守一些。
    if(!yawLockedAngularDynamicsWithinLimits(result)){
        double lower = 0.0;
        double upper = 1.0;
        std::array<double, 3> safe = committedEffectiveEulerRate;
        if(!yawLockedAngularDynamicsWithinLimits(safe)){
            return safe;
        }
        for(int iteration = 0; iteration < 30; ++iteration){
            const double ratio = 0.5 * (lower + upper);
            std::array<double, 3> trial{};
            for(int dim = 0; dim < 3; ++dim){
                trial[dim] = committedEffectiveEulerRate[dim] +
                        (result[dim] - committedEffectiveEulerRate[dim]) *
                        ratio;
            }
            trial[2] = 0.0;
            if(yawLockedAngularDynamicsWithinLimits(trial)){
                lower = ratio;
                safe = trial;
            }
            else{
                upper = ratio;
            }
        }
        result = safe;
    }
    for(double& value : result){
        if(std::fabs(value) <=
                kEndpointRemoteAngularVelocityEpsilonRadPerSec){
            value = 0.0;
        }
    }
    return result;
}

std::array<double, kEndpointRemoteBoundaryDirectionCount>
EndpointRemoteControl::boundaryDistances(
        const std::array<double, 6>& pose) const
{
    return {{
        config.workspaceMaximum[0] - pose[0],
        pose[0] - config.workspaceMinimum[0],
        config.workspaceMaximum[1] - pose[1],
        pose[1] - config.workspaceMinimum[1],
        config.workspaceMaximum[2] - pose[2],
        pose[2] - config.workspaceMinimum[2]
    }};
}

std::array<EndpointRemoteBoundaryState,
           kEndpointRemoteBoundaryDirectionCount>
EndpointRemoteControl::advanceBoundaryStates(
        const std::array<double, 6>& pose,
        const std::array<double, 3>& effectiveVelocity,
        std::array<EndpointRemoteBoundaryState,
                   kEndpointRemoteBoundaryDirectionCount> states) const
{
    const auto distances = boundaryDistances(pose);
    const double guardMm = config.outwardRestartGuardMm();
    const double releaseDistanceMm =
            guardMm + config.boundaryReleaseHysteresisMm;
    for(int direction = 0;
        direction < kEndpointRemoteBoundaryDirectionCount;
        ++direction){
        const int dim = boundaryDimension(direction);
        const double outwardVelocity =
                boundarySign(direction) * effectiveVelocity[dim];
        const auto nonGuardState = [&](){
            const bool keepSoftBoundary =
                    states[direction] ==
                        EndpointRemoteBoundaryState::SoftBoundary &&
                    distances[direction] <=
                        config.softBoundaryMarginMm +
                        config.boundaryReleaseHysteresisMm;
            return distances[direction] <= config.softBoundaryMarginMm ||
                    keepSoftBoundary ?
                        EndpointRemoteBoundaryState::SoftBoundary :
                        EndpointRemoteBoundaryState::Normal;
        };

        switch(states[direction]){
        case EndpointRemoteBoundaryState::BlockedOutward:
            // 松键/重按不解除闭锁。只有已提交的开环期望位姿真正向内
            // 退到 G+H 之外，才允许该面重新接受朝外速度。
            if(distances[direction] >= releaseDistanceMm){
                states[direction] = nonGuardState();
            }
            break;
        case EndpointRemoteBoundaryState::Braking:
            if(outwardVelocity <=
                    kEndpointRemoteBoundaryVelocityEpsilonMmPerSec){
                states[direction] = distances[direction] <= guardMm ?
                            EndpointRemoteBoundaryState::BlockedOutward :
                            nonGuardState();
            }
            break;
        case EndpointRemoteBoundaryState::Normal:
        case EndpointRemoteBoundaryState::SoftBoundary:
            if(distances[direction] <= guardMm){
                // 每个面的法向速度独立判定，因此角点处可以在同一周期
                // 同时闭锁两个或三个朝外分量，不依赖三维合速度归零。
                states[direction] = outwardVelocity >
                        kEndpointRemoteBoundaryVelocityEpsilonMmPerSec ?
                            EndpointRemoteBoundaryState::Braking :
                            EndpointRemoteBoundaryState::BlockedOutward;
            }
            else{
                states[direction] = nonGuardState();
            }
            break;
        }
    }
    return states;
}

bool EndpointRemoteControl::stoppingTrajectoryInsideWorkspace(
        const std::array<double, 3>& effectiveVelocity,
        std::array<bool, kEndpointRemoteBoundaryDirectionCount>* violatedFaces,
        bool* voxelAngleLimitViolated) const
{
    if(violatedFaces){
        violatedFaces->fill(false);
    }
    if(voxelAngleLimitViolated){
        *voxelAngleLimitViolated = false;
    }
    if(!finiteArray(effectiveVelocity)){
        return false;
    }
    const double dtSec =
            static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    std::array<double, 3> candidatePosition{{
        committedPose[0] + effectiveVelocity[0] * dtSec,
        committedPose[1] + effectiveVelocity[1] * dtSec,
        committedPose[2] + effectiveVelocity[2] * dtSec
    }};
    std::array<double, 3> stoppingPosition = candidatePosition;
    const double speed = vectorNorm(effectiveVelocity);
    if(speed > 1.0e-12){
        // 候选周期之后保守保留一个完整命令响应周期，并额外覆盖允许的
        // 单周期漏拍补推。20 ms与1/2/5/10 ms使用同一套按配置周期缩放
        // 的停止包络；周期越长，边界会相应更早进入制动。随后再按最大
        // 末端减速度沿当前有效速度方向减到零。安全区域是凸长方体，因此
        // 候选点和制动终点都在内部即可保证整条直线制动段都在内部。
        const double guardedHoldPeriods =
                endpointRemoteGuardedTravelPeriods() - 1.0;
        const double stoppingDistance =
                speed * speed /
                (2.0 * config.translationAccelerationMmPerSec2) +
                speed * dtSec * guardedHoldPeriods;
        for(int dim = 0; dim < 3; ++dim){
            stoppingPosition[dim] +=
                    effectiveVelocity[dim] / speed * stoppingDistance;
        }
    }

    bool inside = true;
    for(int dim = 0; dim < 3; ++dim){
        const bool belowMinimum =
                candidatePosition[dim] < config.workspaceMinimum[dim] ||
                stoppingPosition[dim] < config.workspaceMinimum[dim];
        const bool aboveMaximum =
                candidatePosition[dim] > config.workspaceMaximum[dim] ||
                stoppingPosition[dim] > config.workspaceMaximum[dim];
        if(belowMinimum){
            inside = false;
            if(violatedFaces){
                (*violatedFaces)[2 * dim + 1] = true;
            }
        }
        if(aboveMaximum){
            inside = false;
            if(violatedFaces){
                (*violatedFaces)[2 * dim] = true;
            }
        }
    }
    if(inside && speed > kEndpointRemoteBoundaryVelocityEpsilonMmPerSec &&
            !translationSegmentInsideVoxelAngleLimits(
                {{committedPose[0], committedPose[1], committedPose[2]}},
                stoppingPosition,
                committedPose)){
        inside = false;
        if(voxelAngleLimitViolated){
            *voxelAngleLimitViolated = true;
        }
    }
    return inside;
}

bool EndpointRemoteControl::translationSegmentInsideVoxelAngleLimits(
        const std::array<double, 3>& startPosition,
        const std::array<double, 3>& endPosition,
        const std::array<double, 6>& orientationPose) const
{
    std::array<double, 6> startPose = orientationPose;
    std::array<double, 6> endPose = orientationPose;
    for(int dim = 0; dim < 3; ++dim){
        startPose[dim] = startPosition[dim];
        endPose[dim] = endPosition[dim];
    }
    std::array<int, 3> currentIndex{};
    std::array<int, 3> endIndex{};
    if(!config.voxelAngleLimits.limitForPose(startPose, &currentIndex) ||
            !config.voxelAngleLimits.limitForPose(endPose, &endIndex)){
        return false;
    }
    const auto orientationAllowedInCell = [&](const std::array<int, 3>& index){
        const EndpointRemoteVoxelAngleLimit* limit =
                config.voxelAngleLimits.limitAt(index);
        return limit &&
                orientationPose[3] >= limit->rxMinimumRad &&
                orientationPose[3] <= limit->rxMaximumRad &&
                orientationPose[4] >= limit->ryMinimumRad &&
                orientationPose[4] <= limit->ryMaximumRad &&
                orientationPose[5] >=
                    config.translationSafeOrientationMinimumRad[2] &&
                orientationPose[5] <=
                    config.translationSafeOrientationMaximumRad[2];
    };
    if(!orientationAllowedInCell(currentIndex)){
        return false;
    }

    std::array<int, 3> step{{0, 0, 0}};
    std::array<double, 3> nextCrossing{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    }};
    std::array<double, 3> crossingIncrement = nextCrossing;
    for(int dim = 0; dim < 3; ++dim){
        const double delta = endPosition[dim] - startPosition[dim];
        if(std::abs(delta) <= kVoxelGeometryToleranceMm){
            continue;
        }
        if(delta > 0.0){
            step[dim] = 1;
            const double boundary =
                    config.voxelAngleLimits.workspaceMinimumMm[dim] +
                    (currentIndex[dim] + 1) *
                    config.voxelAngleLimits.cellSizeMm[dim];
            nextCrossing[dim] =
                    (boundary - startPosition[dim]) / delta;
            crossingIncrement[dim] =
                    config.voxelAngleLimits.cellSizeMm[dim] / delta;
        }
        else{
            step[dim] = -1;
            const double boundary =
                    config.voxelAngleLimits.workspaceMinimumMm[dim] +
                    currentIndex[dim] *
                    config.voxelAngleLimits.cellSizeMm[dim];
            nextCrossing[dim] =
                    (boundary - startPosition[dim]) / delta;
            crossingIncrement[dim] =
                    -config.voxelAngleLimits.cellSizeMm[dim] / delta;
        }
        nextCrossing[dim] = std::max(0.0, nextCrossing[dim]);
    }

    const int maximumVisits = config.voxelAngleLimits.cellCount[0] +
            config.voxelAngleLimits.cellCount[1] +
            config.voxelAngleLimits.cellCount[2] + 3;
    for(int visit = 0; currentIndex != endIndex; ++visit){
        if(visit >= maximumVisits){
            return false;
        }
        const double crossing = std::min(
                    nextCrossing[0],
                    std::min(nextCrossing[1], nextCrossing[2]));
        if(!std::isfinite(crossing) || crossing > 1.0 + 1.0e-12){
            return false;
        }
        for(int dim = 0; dim < 3; ++dim){
            if(nextCrossing[dim] <= crossing + 1.0e-12){
                currentIndex[dim] += step[dim];
                nextCrossing[dim] += crossingIncrement[dim];
            }
        }
        if(!orientationAllowedInCell(currentIndex)){
            return false;
        }
    }
    return true;
}

bool EndpointRemoteControl::orientationBoundsForPose(
        const std::array<double, 6>& pose,
        bool translationSafe,
        std::array<double, 3>* minimum,
        std::array<double, 3>* maximum,
        const EndpointRemoteVoxelAngleLimit** voxelLimit,
        std::array<int, 3>* voxelIndex) const
{
    if(!minimum || !maximum){
        return false;
    }
    *minimum = translationSafe ?
                config.translationSafeOrientationMinimumRad :
                config.orientationMinimumRad;
    *maximum = translationSafe ?
                config.translationSafeOrientationMaximumRad :
                config.orientationMaximumRad;
    std::array<int, 3> resolvedIndex{};
    const EndpointRemoteVoxelAngleLimit* resolvedLimit =
            config.voxelAngleLimits.limitForPose(pose, &resolvedIndex);
    if(!resolvedLimit){
        return false;
    }
    if(!translationSafe){
        // CSV是当前配置下Rx/Ry的真实转动硬边界；旧的Rx/Ry全局占位值
        // 不再参与裁剪。平动回正阈值则由所有体素范围的交集单独给出。
        (*minimum)[0] = resolvedLimit->rxMinimumRad;
        (*maximum)[0] = resolvedLimit->rxMaximumRad;
        (*minimum)[1] = resolvedLimit->ryMinimumRad;
        (*maximum)[1] = resolvedLimit->ryMaximumRad;
    }
    if(voxelLimit){
        *voxelLimit = resolvedLimit;
    }
    if(voxelIndex){
        *voxelIndex = resolvedIndex;
    }
    return true;
}

bool EndpointRemoteControl::orientationInsideBounds(
        const std::array<double, 6>& pose,
        bool translationSafe) const
{
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    if(!orientationBoundsForPose(pose, translationSafe,
                                 &minimum, &maximum)){
        return false;
    }
    for(int dim = 0; dim < 3; ++dim){
        if(!std::isfinite(pose[dim + 3]) ||
                pose[dim + 3] < minimum[dim] ||
                pose[dim + 3] > maximum[dim]){
            return false;
        }
    }
    return true;
}

bool EndpointRemoteControl::yawLockedAngularDynamicsWithinLimits(
        const std::array<double, 3>& effectiveEulerRate,
        std::array<double, 3>* globalAngularVelocity,
        double* globalAngularAccelerationRadPerSec2,
        QString* errorMessage) const
{
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(!finiteArray(effectiveEulerRate) ||
            !finiteArray(committedEffectiveEulerRate) ||
            !finiteArray(committedGlobalAngularVelocity) ||
            !finiteArray(committedPose) ||
            !std::isfinite(lockedYawRad) ||
            std::abs(effectiveEulerRate[2]) >
                kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        return fail(QStringLiteral(
                        "Rz锁定转动包含无效数值或非零Rz欧拉角速度"));
    }
    const double dtSec =
            static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        return fail(QStringLiteral("Rz锁定转动控制周期无效"));
    }
    const std::array<double, 6> nextPose = integrateYawLockedEulerRate(
                committedPose, effectiveEulerRate, dtSec, lockedYawRad);
    const std::array<double, 3> startInstantaneousOmega =
            yawLockedEulerRateToGlobalAngularVelocity(
                committedPose, effectiveEulerRate);
    const std::array<double, 3> endInstantaneousOmega =
            yawLockedEulerRateToGlobalAngularVelocity(
                nextPose, effectiveEulerRate);
    const std::array<double, 3> intervalGlobalOmega =
            finiteStepGlobalAngularVelocity(committedPose, nextPose, dtSec);
    if(!finiteArray(startInstantaneousOmega) ||
            !finiteArray(endInstantaneousOmega) ||
            !finiteArray(intervalGlobalOmega)){
        return fail(QStringLiteral("Rx/Ry欧拉角速度换算真实全局角速度失败"));
    }
    const double physicalSpeed = std::max(
                vectorNorm(intervalGlobalOmega),
                std::max(vectorNorm(startInstantaneousOmega),
                         vectorNorm(endInstantaneousOmega)));

    std::array<double, 3> eulerAcceleration{};
    for(int dim = 0; dim < 3; ++dim){
        eulerAcceleration[dim] =
                (effectiveEulerRate[dim] -
                 committedEffectiveEulerRate[dim]) / dtSec;
    }
    const double maximumRollRate = std::max(
                std::abs(committedEffectiveEulerRate[0]),
                std::abs(effectiveEulerRate[0]));
    const double maximumPitchRate = std::max(
                std::abs(committedEffectiveEulerRate[1]),
                std::abs(effectiveEulerRate[1]));
    // 对Rz_dot=0的ZYX欧拉角，物理角加速度由互相正交的三部分组成：
    // Rx_ddot、Ry_ddot和转轴变化项Rx_dot*Ry_dot。
    const double geometricAccelerationBound =
            maximumRollRate * maximumPitchRate;
    const double analyticAccelerationBound = std::hypot(
                vectorNorm(eulerAcceleration),
                geometricAccelerationBound);
    std::array<double, 3> intervalOmegaDelta{};
    for(int dim = 0; dim < 3; ++dim){
        intervalOmegaDelta[dim] = intervalGlobalOmega[dim] -
                committedGlobalAngularVelocity[dim];
    }
    const double discreteAcceleration =
            vectorNorm(intervalOmegaDelta) / dtSec;
    const double physicalAcceleration =
            std::max(analyticAccelerationBound, discreteAcceleration);
    const double yawError =
            std::abs(wrappedAngleError(nextPose[5], lockedYawRad));

    if(globalAngularVelocity){
        *globalAngularVelocity = intervalGlobalOmega;
    }
    if(globalAngularAccelerationRadPerSec2){
        *globalAngularAccelerationRadPerSec2 = physicalAcceleration;
    }
    if(!std::isfinite(physicalSpeed) ||
            physicalSpeed > config.maximumAngularSpeedRadPerSec +
                kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        return fail(QStringLiteral(
                        "真实全局角速度%1 rad/s超过上限%2 rad/s")
                    .arg(physicalSpeed, 0, 'f', 9)
                    .arg(config.maximumAngularSpeedRadPerSec, 0, 'f', 9));
    }
    if(!std::isfinite(physicalAcceleration) ||
            physicalAcceleration >
                config.maximumAngularAccelerationRadPerSec2 + 1.0e-12){
        return fail(QStringLiteral(
                        "真实全局角加速度%1 rad/s²超过上限%2 rad/s²（几何项上界%3 rad/s²）")
                    .arg(physicalAcceleration, 0, 'f', 9)
                    .arg(config.maximumAngularAccelerationRadPerSec2,
                         0, 'f', 9)
                    .arg(geometricAccelerationBound, 0, 'f', 9));
    }
    if(!std::isfinite(yawError) || yawError > kYawLockToleranceRad){
        return fail(QStringLiteral(
                        "开环期望Rz偏离锁存值%1 rad，超过容差%2 rad")
                    .arg(yawError, 0, 'e', 3)
                    .arg(kYawLockToleranceRad, 0, 'e', 3));
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool EndpointRemoteControl::yawLockedRotationTrajectoryInsideBounds(
        const std::array<double, 6>& startPose,
        const std::array<double, 3>& unitEulerRate,
        double angleRad) const
{
    if(!finiteArray(startPose) || !finiteArray(unitEulerRate) ||
            !std::isfinite(angleRad) ||
            std::abs(unitEulerRate[2]) >
                kEndpointRemoteAngularVelocityEpsilonRadPerSec ||
            std::abs(wrappedAngleError(startPose[5], lockedYawRad)) >
                kYawLockToleranceRad ||
            !orientationInsideBounds(startPose, false)){
        return false;
    }
    if(std::abs(angleRad) <= kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        return true;
    }
    const int sampleCount = static_cast<int>(std::ceil(
                std::abs(angleRad) / kRotationBoundarySampleStepRad));
    if(sampleCount <= 0 || sampleCount > kMaximumRotationBoundarySamples){
        return false;
    }
    for(int sample = 1; sample <= sampleCount; ++sample){
        const double sampleAngle = angleRad *
                static_cast<double>(sample) /
                static_cast<double>(sampleCount);
        const std::array<double, 6> samplePose =
                integrateYawLockedEulerRate(startPose,
                                            unitEulerRate,
                                            sampleAngle,
                                            lockedYawRad);
        if(std::abs(wrappedAngleError(samplePose[5], lockedYawRad)) >
                    kYawLockToleranceRad ||
                !orientationInsideBounds(samplePose, false)){
            return false;
        }
    }
    return true;
}

bool EndpointRemoteControl::stoppingRotationInsideBounds(
        const std::array<double, 3>& effectiveEulerRate) const
{
    if(!finiteArray(effectiveEulerRate) ||
            !yawLockedAngularDynamicsWithinLimits(effectiveEulerRate) ||
            !orientationInsideBounds(committedPose, false)){
        return false;
    }
    const double speed = vectorNorm(effectiveEulerRate);
    if(speed <= kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        return true;
    }
    const double dtSec =
            static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    const double geometricAcceleration =
            std::abs(effectiveEulerRate[0] * effectiveEulerRate[1]);
    const double accelerationSquared =
            config.maximumAngularAccelerationRadPerSec2 *
            config.maximumAngularAccelerationRadPerSec2 -
            geometricAcceleration * geometricAcceleration;
    if(accelerationSquared <= 0.0){
        return false;
    }
    // 为Rx/Ry复合运动的转轴变化项预留角加速度预算；用整个停止过程中
    // 最不利的起始几何项得到保守且恒定的欧拉角减速度。
    const double safeEulerDeceleration = std::sqrt(accelerationSquared);
    const double stoppingAngle = speed * speed /
            (2.0 * safeEulerDeceleration) +
            speed * dtSec * endpointRemoteGuardedTravelPeriods();
    std::array<double, 3> unitEulerRate{};
    for(int dim = 0; dim < 3; ++dim){
        unitEulerRate[dim] = effectiveEulerRate[dim] / speed;
    }
    // 从当前姿态起覆盖候选积分、命令响应/漏拍保持和物理角加速度约束下的
    // 完整制动过程；Rz始终固定为会话锁存值。
    return yawLockedRotationTrajectoryInsideBounds(
                committedPose,
                unitEulerRate,
                stoppingAngle);
}

void EndpointRemoteControl::updateOrientationStatus(
        const std::array<double, 6>& pose)
{
    const bool wasTranslationInterlockActive =
            currentStatus.translationBlockedByOrientation;
    currentStatus.yawLockReferenceRad = lockedYawRad;
    currentStatus.yawLockErrorRad =
            std::abs(wrappedAngleError(pose[5], lockedYawRad));
    std::array<double, 3> hardMinimum{};
    std::array<double, 3> hardMaximum{};
    std::array<int, 3> voxelIndex{};
    const EndpointRemoteVoxelAngleLimit* voxelLimit = nullptr;
    const bool hardBoundsAvailable = orientationBoundsForPose(
                pose, false, &hardMinimum, &hardMaximum,
                &voxelLimit, &voxelIndex);
    std::array<double, 3> translationMinimum{};
    std::array<double, 3> translationMaximum{};
    const bool translationBoundsAvailable = orientationBoundsForPose(
                pose, true, &translationMinimum, &translationMaximum);
    currentStatus.voxelAngleLimitAvailable =
            hardBoundsAvailable && voxelLimit;
    if(currentStatus.voxelAngleLimitAvailable){
        currentStatus.voxelIndex = voxelIndex;
        currentStatus.voxelMinimumMm = voxelLimit->minimumMm;
        currentStatus.voxelMaximumMm = voxelLimit->maximumMm;
        currentStatus.voxelRxRangeRad = {{
            voxelLimit->rxMinimumRad, voxelLimit->rxMaximumRad
        }};
        currentStatus.voxelRyRangeRad = {{
            voxelLimit->ryMinimumRad, voxelLimit->ryMaximumRad
        }};
        constexpr double radToDeg = 180.0 / kPi;
        currentStatus.voxelAngleLimitSummary = QStringLiteral(
                    "期望位置体素[%1,%2,%3]：X[%4,%5]、Y[%6,%7]、Z[%8,%9] mm；Rx[%10,%11]°，Ry[%12,%13]°")
                .arg(voxelIndex[0]).arg(voxelIndex[1]).arg(voxelIndex[2])
                .arg(voxelLimit->minimumMm[0], 0, 'f', 1)
                .arg(voxelLimit->maximumMm[0], 0, 'f', 1)
                .arg(voxelLimit->minimumMm[1], 0, 'f', 1)
                .arg(voxelLimit->maximumMm[1], 0, 'f', 1)
                .arg(voxelLimit->minimumMm[2], 0, 'f', 1)
                .arg(voxelLimit->maximumMm[2], 0, 'f', 1)
                .arg(voxelLimit->rxMinimumRad * radToDeg, 0, 'f', 2)
                .arg(voxelLimit->rxMaximumRad * radToDeg, 0, 'f', 2)
                .arg(voxelLimit->ryMinimumRad * radToDeg, 0, 'f', 2)
                .arg(voxelLimit->ryMaximumRad * radToDeg, 0, 'f', 2);
        currentStatus.voxelAngleLimitSummary += QStringLiteral(
                    "；全立方体平动回正阈值Rx[%1,%2]°、Ry[%3,%4]°")
                .arg(config.translationSafeOrientationMinimumRad[0] *
                     radToDeg, 0, 'f', 2)
                .arg(config.translationSafeOrientationMaximumRad[0] *
                     radToDeg, 0, 'f', 2)
                .arg(config.translationSafeOrientationMinimumRad[1] *
                     radToDeg, 0, 'f', 2)
                .arg(config.translationSafeOrientationMaximumRad[1] *
                     radToDeg, 0, 'f', 2);
    }
    else{
        currentStatus.voxelIndex = {{-1, -1, -1}};
        currentStatus.voxelAngleLimitSummary = QStringLiteral(
                    "开环期望位置没有对应的有效体素角度范围");
    }
    if(!hardBoundsAvailable){
        hardMinimum = config.orientationMinimumRad;
        hardMaximum = config.orientationMaximumRad;
    }
    currentStatus.angularBoundaryDistanceRad = {{
        hardMaximum[0] - pose[3],
        pose[3] - hardMinimum[0],
        hardMaximum[1] - pose[4],
        pose[4] - hardMinimum[1],
        hardMaximum[2] - pose[5],
        pose[5] - hardMinimum[2]
    }};
    static const char* const boundaryLabels[6] = {
        "+Rx", "-Rx", "+Ry", "-Ry", "+Rz", "-Rz"
    };
    int nearestBoundary = 0;
    for(int direction = 1;
        direction < kEndpointRemoteAngularBoundaryDirectionCount;
        ++direction){
        if(currentStatus.angularBoundaryDistanceRad[direction] <
                currentStatus.angularBoundaryDistanceRad[nearestBoundary]){
            nearestBoundary = direction;
        }
    }
    currentStatus.angularBoundarySummary = QStringLiteral(
                "最近姿态硬边界%1，距离%2 deg；Rz锁存误差=%3 deg%4")
            .arg(QString::fromLatin1(boundaryLabels[nearestBoundary]))
            .arg(currentStatus.angularBoundaryDistanceRad[nearestBoundary] *
                 180.0 / kPi, 0, 'f', 2)
            .arg(currentStatus.yawLockErrorRad * 180.0 / kPi,
                 0, 'e', 2)
            .arg(currentStatus.angularBoundaryBrakingActive ?
                     QStringLiteral("（正在按角加速度上限制动）") :
                     QString());
    currentStatus.translationBlockedByOrientation =
            !translationBoundsAvailable ||
            !orientationInsideBounds(pose, true);
    QStringList unsafeDimensions;
    static const char* const labels[3] = {"Rx", "Ry", "Rz"};
    constexpr double radToDeg = 180.0 / kPi;
    for(int dim = 0; dim < 3; ++dim){
        if(!translationBoundsAvailable ||
                pose[dim + 3] < translationMinimum[dim] ||
                pose[dim + 3] > translationMaximum[dim]){
            unsafeDimensions.append(QStringLiteral(
                        "%1=%2 deg（平动安全范围[%3,%4] deg）")
                    .arg(QString::fromLatin1(labels[dim]))
                    .arg(pose[dim + 3] * radToDeg, 0, 'f', 2)
                    .arg(translationBoundsAvailable ?
                             translationMinimum[dim] * radToDeg :
                             std::numeric_limits<double>::quiet_NaN(),
                         0, 'f', 2)
                    .arg(translationBoundsAvailable ?
                             translationMaximum[dim] * radToDeg :
                             std::numeric_limits<double>::quiet_NaN(),
                         0, 'f', 2));
        }
    }
    currentStatus.orientationRecoverySummary = unsafeDimensions.isEmpty() ?
                QString() :
                QStringLiteral("姿态未回正，平动联锁已生效：%1；即使收到平动指令也不会运动，请按住食指扳机用转动模式回正")
                .arg(unsafeDimensions.join(QStringLiteral("；")));
    if(currentStatus.translationBlockedByOrientation &&
            !wasTranslationInterlockActive){
        currentStatus.message = QStringLiteral(
                    "姿态已超出平动安全阈值，平动联锁生效；请使用转动模式回正");
    }
    else if(!currentStatus.translationBlockedByOrientation &&
            wasTranslationInterlockActive){
        currentStatus.message = QStringLiteral(
                    "姿态已回正到平动安全阈值内，平动联锁解除，可以继续接受平动指令");
    }
}

void EndpointRemoteControl::updateBoundaryStatus(
        const std::array<double, 6>& candidatePose,
        std::array<EndpointRemoteBoundaryState,
                   kEndpointRemoteBoundaryDirectionCount> nextState)
{
    const auto distances = boundaryDistances(candidatePose);
    QStringList transitions;
    QStringList activeStates;
    for(int direction = 0;
        direction < kEndpointRemoteBoundaryDirectionCount;
        ++direction){
        if(nextState[direction] != boundaryState[direction]){
            transitions.append(QStringLiteral(
                                   "%1：%2→%3（距硬边界%4 mm，朝外重启保护层G=%5 mm）")
                               .arg(boundaryDirectionText(direction),
                                    boundaryStateText(boundaryState[direction]),
                                    boundaryStateText(nextState[direction]))
                               .arg(distances[direction], 0, 'f', 3)
                               .arg(config.outwardRestartGuardMm(),
                                    0, 'f', 3));
        }
        if(nextState[direction] != EndpointRemoteBoundaryState::Normal){
            activeStates.append(QStringLiteral("%1=%2(%3 mm)")
                                .arg(boundaryDirectionText(direction),
                                     boundaryStateText(nextState[direction]))
                                .arg(distances[direction], 0, 'f', 3));
        }
    }

    boundaryState = nextState;
    currentStatus.boundaryState = nextState;
    currentStatus.boundaryDistanceMm = distances;
    currentStatus.boundarySummary = activeStates.isEmpty() ?
                QStringLiteral("全部方向正常") : activeStates.join(QStringLiteral("；"));
    if(!transitions.isEmpty()){
        currentStatus.boundaryEventSequence++;
        currentStatus.latestBoundaryEvent = transitions.join(QStringLiteral("；"));
        qInfo().noquote() << QStringLiteral("末端遥控边界保护：%1")
                             .arg(currentStatus.latestBoundaryEvent);
    }
}

EndpointRemoteControl::Candidate EndpointRemoteControl::buildCandidate(
        const std::array<double, 3>& effectiveVelocity,
        const std::array<double, 3>& effectiveEulerRate,
        bool enforceCommandDynamics) const
{
    Candidate candidate;
    if(!actualStartCaptured || !finiteArray(effectiveVelocity) ||
            !finiteArray(effectiveEulerRate)){
        candidate.errorMessage = QStringLiteral("遥控起始电机位置尚未锁存或末端速度无效");
        return candidate;
    }
    if(vectorNorm(effectiveVelocity) >
                kEndpointRemoteBoundaryVelocityEpsilonMmPerSec &&
            vectorNorm(effectiveEulerRate) >
                kEndpointRemoteAngularVelocityEpsilonRadPerSec){
        candidate.errorMessage = QStringLiteral(
                    "平动速度与Rz锁定Rx/Ry欧拉角速度不能在同一周期同时非零");
        return candidate;
    }
    candidate.pose = committedPose;
    candidate.effectiveVelocity = effectiveVelocity;
    candidate.effectiveEulerRate = effectiveEulerRate;
    if(!yawLockedAngularDynamicsWithinLimits(
            effectiveEulerRate,
            &candidate.globalAngularVelocity,
            &candidate.globalAngularAccelerationRadPerSec2,
            &candidate.errorMessage)){
        candidate.errorMessage = QStringLiteral(
                    "Rz锁定转动真实角速度/角加速度检查失败：%1")
                .arg(candidate.errorMessage);
        return candidate;
    }
    const double dtSec = static_cast<double>(config.onlineVelocity.periodUs) / 1000000.0;
    for(int dim = 0; dim < 3; ++dim){
        candidate.pose[dim] += effectiveVelocity[dim] * dtSec;
        if(!std::isfinite(candidate.pose[dim]) ||
                candidate.pose[dim] < config.workspaceMinimum[dim] ||
                candidate.pose[dim] > config.workspaceMaximum[dim]){
            candidate.errorMessage = QStringLiteral("候选开环期望位姿在维度%1超出工作空间")
                    .arg(dim);
            return candidate;
        }
    }
    candidate.pose = integrateYawLockedEulerRate(
                candidate.pose,
                effectiveEulerRate,
                dtSec,
                lockedYawRad);
    if(std::abs(wrappedAngleError(candidate.pose[5], lockedYawRad)) >
            kYawLockToleranceRad){
        candidate.errorMessage = QStringLiteral(
                    "候选开环期望Rz偏离会话锁存值");
        return candidate;
    }
    if(!orientationInsideBounds(candidate.pose, false)){
        candidate.errorMessage = QStringLiteral(
                    "候选开环期望姿态超出当前体素Rx/Ry或Rz姿态硬边界");
        return candidate;
    }
    const CompensatedCableKinematics::PoseMatrix poseMatrix{
        std::vector<double>(candidate.pose.begin(), candidate.pose.end())
    };
    const CompensatedCableKinematics::Evaluation evaluation =
            kinematics.evaluatePose(poseMatrix, committedKinematicsState);
    if(!evaluation.valid ||
            evaluation.relativeMotorThetaRad.size() != kOnlineVelocityAxisCount ||
            committedRelativeMotorThetaRad.size() != kOnlineVelocityAxisCount){
        candidate.errorMessage = QStringLiteral("候选位姿运动学计算失败：%1")
                .arg(evaluation.errorMessage);
        return candidate;
    }
    candidate.relativeMotorThetaRad = evaluation.relativeMotorThetaRad;
    candidate.kinematicsState = evaluation.nextState;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        candidate.referencePosition[axis] = actualStartMotorPosition[axis] +
                config.motorUnitPerRadian[axis] *
                candidate.relativeMotorThetaRad[axis];
        candidate.commandVelocity[axis] = config.motorUnitPerRadian[axis] *
                (candidate.relativeMotorThetaRad[axis] -
                 committedRelativeMotorThetaRad[axis]) / dtSec;
        if(!std::isfinite(candidate.referencePosition[axis]) ||
                !std::isfinite(candidate.commandVelocity[axis])){
            candidate.errorMessage = QStringLiteral("轴%1候选位置或速度无效")
                    .arg(axis + 1);
            return candidate;
        }
        if(candidate.referencePosition[axis] < config.motorPositionMinimum[axis] ||
                candidate.referencePosition[axis] > config.motorPositionMaximum[axis]){
            candidate.errorMessage = QStringLiteral("轴%1候选位置%2超出软件限位[%3,%4]")
                    .arg(axis + 1)
                    .arg(candidate.referencePosition[axis], 0, 'f', 6)
                    .arg(config.motorPositionMinimum[axis], 0, 'f', 6)
                    .arg(config.motorPositionMaximum[axis], 0, 'f', 6);
            return candidate;
        }
        if(enforceCommandDynamics &&
                std::fabs(candidate.commandVelocity[axis]) >
                config.onlineVelocity.maxVelocity + 1.0e-12){
            candidate.errorMessage = QStringLiteral("轴%1候选速度%2超过在线上限%3")
                    .arg(axis + 1)
                    .arg(candidate.commandVelocity[axis], 0, 'f', 6)
                    .arg(config.onlineVelocity.maxVelocity, 0, 'f', 6);
            return candidate;
        }
        const double acceleration = std::fabs(
                    candidate.commandVelocity[axis] - lastCommandVelocity[axis]) / dtSec;
        if(enforceCommandDynamics &&
                acceleration > config.onlineVelocity.maxAcceleration + 1.0e-12){
            candidate.errorMessage = QStringLiteral("轴%1候选加速度%2超过在线上限%3")
                    .arg(axis + 1)
                    .arg(acceleration, 0, 'f', 6)
                    .arg(config.onlineVelocity.maxAcceleration, 0, 'f', 6);
            return candidate;
        }
    }
    candidate.valid = true;
    return candidate;
}

void EndpointRemoteControl::setFault(const QString& reason)
{
    pendingCandidateValid = false;
    pendingCommandIntent = EndpointRemoteCommandIntent::None;
    requestedMotionMode = EndpointRemoteMotionMode::None;
    activeMotionMode = EndpointRemoteMotionMode::None;
    requestedDirection.fill(0.0);
    committedEffectiveVelocity.fill(0.0);
    committedEffectiveEulerRate.fill(0.0);
    committedGlobalAngularVelocity.fill(0.0);
    lastCommandVelocity.fill(0.0);
    inputHeartbeatArmed = false;
    inputHeartbeatStale = false;
    currentStatus.state = EndpointRemoteStatus::State::Fault;
    currentStatus.actuationProfile = EndpointRemoteActuationProfile::Faulted;
    currentStatus.message = reason;
    // 故障发生后保留最终开环位姿和反馈用于诊断，但所有命令/目标速度显示
    // 必须立即归零，不能让已失效的最后一条方向看起来仍在被控制器保持。
    currentStatus.targetVelocityMmPerSec.fill(0.0);
    currentStatus.effectiveVelocityMmPerSec.fill(0.0);
    currentStatus.targetEulerRateRadPerSec.fill(0.0);
    currentStatus.effectiveEulerRateRadPerSec.fill(0.0);
    currentStatus.effectiveGlobalAngularVelocityRadPerSec.fill(0.0);
    currentStatus.effectiveGlobalAngularAccelerationRadPerSec2 = 0.0;
    currentStatus.requestedMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.activeMotionMode = EndpointRemoteMotionMode::None;
    currentStatus.angularBoundaryBrakingActive = false;
    currentStatus.voxelAngleBoundaryBrakingActive = false;
    currentStatus.commandVelocity.fill(0.0);
}
