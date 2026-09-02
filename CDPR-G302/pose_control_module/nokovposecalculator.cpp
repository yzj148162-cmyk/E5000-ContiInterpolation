/*
 * 文件总览：
 * - NokovPoseCalculator 的实现文件，完成标记点排序、姿态解算和欧拉角输出。
 * - 算法假设输入标记点数量满足 REQUIRED_MARKER_COUNT，不满足时由调用方保留上一帧或提示采集失败。
 */

#include "nokovposecalculator.h"

#include <Eigen/Dense>

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
// constexpr double kMocapToQtYawDeg = 0.3555;
// constexpr double kMocapToQtYawDeg = 0.365210689450862;
// constexpr double kMocapToQtYawDeg = 0.800394062171612;
constexpr double kMocapToQtYawDeg = 1.2634;

Eigen::Vector3d markerToVector(const MarkerPoint& marker)
{
    return Eigen::Vector3d(marker.x, marker.y, marker.z);
}

Eigen::Vector3d transformMocapPointToQt(const Eigen::Vector3d& point)
{
    const double theta = kMocapToQtYawDeg * kPi / 180.0;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = std::cos(theta);
    rotation(0, 1) = -std::sin(theta);
    rotation(1, 0) = std::sin(theta);
    rotation(1, 1) = std::cos(theta);

    // const Eigen::Vector3d offset(157.6381, 279.2485, 48.5099);
    // const Eigen::Vector3d offset(152.62675,281.9953,48.5727);
    // const Eigen::Vector3d offset(-502.4504,-542.6511,-141.5380);
    // const Eigen::Vector3d offset(-168.2879，-519.5231，-141.4515);
    const Eigen::Vector3d offset(-168.2879,-519.5231,-141.4515);
    return rotation * point + offset;
}

QVector3D toQVector3D(const Eigen::Vector3d& value)
{
    return QVector3D(static_cast<float>(value.x()),
                     static_cast<float>(value.y()),
                     static_cast<float>(value.z()));
}
}

bool NokovPoseCalculator::update(const QVector<MarkerPoint>& markers, Result& result)
{
    result = Result{};

    QVector<MarkerPoint> orderedMarkers = orderByCaptureSequence(markers);
    if (orderedMarkers.size() != REQUIRED_MARKER_COUNT) {
        return false;
    }
    for (const MarkerPoint& marker : orderedMarkers) {
        if (!std::isfinite(marker.x) ||
            !std::isfinite(marker.y) ||
            !std::isfinite(marker.z)) {
            return false;
        }
    }

    return calculateFromOrderedMarkers(orderedMarkers, result);
}

bool NokovPoseCalculator::hasRoleAssignment() const
{
    return true;
}

void NokovPoseCalculator::resetRoleAssignment()
{
}

QVector<MarkerPoint> NokovPoseCalculator::orderByCaptureSequence(const QVector<MarkerPoint>& markers)
{
    if (markers.size() != REQUIRED_MARKER_COUNT) {
        return {};
    }

    return markers;
}

bool NokovPoseCalculator::calculateFromOrderedMarkers(const QVector<MarkerPoint>& orderedMarkers,
                                                      Result& result)
{
    if (orderedMarkers.size() != REQUIRED_MARKER_COUNT) {
        return false;
    }

    const Eigen::Vector3d a1 = transformMocapPointToQt(markerToVector(orderedMarkers[0]));
    const Eigen::Vector3d a2 = transformMocapPointToQt(markerToVector(orderedMarkers[1]));
    const Eigen::Vector3d a3 = transformMocapPointToQt(markerToVector(orderedMarkers[2]));

    Eigen::Matrix3d globalPoints;
    globalPoints.col(0) = a1;
    globalPoints.col(1) = a2;
    globalPoints.col(2) = a3;

    Eigen::Matrix3d localPoints;

    // localPoints.col(0) = Eigen::Vector3d(0.0, 0.0, 551.42);
    // localPoints.col(1) = Eigen::Vector3d(0.0, -493.21, 246.60);
    // localPoints.col(2) = Eigen::Vector3d(-375.81, -122.11, 366.22);

    localPoints.col(0) = Eigen::Vector3d(206.36, -67.05, 144.83);
    localPoints.col(1) = Eigen::Vector3d(162.37, 168.19, 115.76);
    localPoints.col(2) = Eigen::Vector3d(-127.54, 175.54, 144.83);

    /*
     * 保留旧 4 点动捕位姿解算方案：
     * const Eigen::Vector3d a1 = transformMocapPointToQt(markerToVector(orderedMarkers[0]));
     * const Eigen::Vector3d a2 = transformMocapPointToQt(markerToVector(orderedMarkers[1]));
     * const Eigen::Vector3d a3 = transformMocapPointToQt(markerToVector(orderedMarkers[2]));
     * const Eigen::Vector3d a4 = transformMocapPointToQt(markerToVector(orderedMarkers[3]));
     *
     * Eigen::Matrix3d globalPoints;
     * globalPoints.col(0) = a1;
     * globalPoints.col(1) = (a2 + a4) / 2.0;
     * globalPoints.col(2) = a3;
     *
     * Eigen::Matrix3d localPoints;
     * localPoints.col(0) = Eigen::Vector3d(0.0, 0.0, 551.42);
     * localPoints.col(1) = Eigen::Vector3d(-285.85, 393.43, 243.15);
     * localPoints.col(2) = Eigen::Vector3d(-375.81, -122.11, 366.22);
     */

    const Eigen::Vector3d globalMean = globalPoints.rowwise().mean();
    const Eigen::Vector3d localMean = localPoints.rowwise().mean();
    const Eigen::Matrix3d globalCentered = globalPoints.colwise() - globalMean;
    const Eigen::Matrix3d localCentered = localPoints.colwise() - localMean;
    const Eigen::Matrix3d h = localCentered * globalCentered.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d u = svd.matrixU();
    Eigen::Matrix3d v = svd.matrixV();
    Eigen::Matrix3d rotation = v * u.transpose();
    if (rotation.determinant() < 0.0) {
        v.col(2) *= -1.0;
        rotation = v * u.transpose();
    }

    const Eigen::Vector3d translation = globalMean - rotation * localMean;

    const double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) +
                                rotation(1, 0) * rotation(1, 0));
    const bool singular = sy < 1e-6;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (!singular) {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        pitch = std::atan2(-rotation(2, 0), sy);
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        roll = std::atan2(-rotation(1, 2), rotation(1, 1));
        pitch = std::atan2(-rotation(2, 0), sy);
        yaw = 0.0;
    }

    const Eigen::Vector3d eulerDeg(roll * 180.0 / kPi,
                                   pitch * 180.0 / kPi,
                                   yaw * 180.0 / kPi);

    if (!translation.allFinite() || !eulerDeg.allFinite()) {
        return false;
    }

    result.positionMm = toQVector3D(translation);
    result.eulerDeg = toQVector3D(eulerDeg);
    result.orderedMarkers = orderedMarkers;
    return true;
}
