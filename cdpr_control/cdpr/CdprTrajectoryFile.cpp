#include "CdprTrajectoryFile.h"

#include <QCryptographicHash>
#include <QByteArrayView>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

namespace {
QByteArray serializeExpectedTrajectory(const CdprOfflinePvtPlan &plan)
{
    QByteArray payload;
    QTextStream stream(&payload, QIODevice::WriteOnly);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(9);
    stream << "time_s,platform_x_m,platform_y_m,platform_z_m"
              ",platform_roll_rad,platform_pitch_rad,platform_yaw_rad";
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        stream << ",axis" << plan.axes[static_cast<size_t>(cable)]
               << "_planned_position_deg";
    }
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        stream << ",axis" << plan.axes[static_cast<size_t>(cable)]
               << "_planned_velocity_deg_per_s";
    }
    stream << '\n';
    for (qsizetype point = 0; point < plan.timeS.size(); ++point) {
        stream << plan.timeS.at(point);
        for (double value : plan.platformPose.at(point)) {
            stream << ',' << value;
        }
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            stream << ',' << plan.axisPositionDegree[static_cast<size_t>(cable)].at(point);
        }
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            stream << ',' << plan.axisVelocityDegreePerSecond[static_cast<size_t>(cable)].at(point);
        }
        stream << '\n';
    }
    stream.flush();
    return payload;
}
}

bool CdprTrajectoryFile::exportExpectedTrajectory(
    const CdprOfflinePvtPlan &plan, const QString &path, QString &sha256Value,
    QString &errorMessage)
{
    if (plan.timeS.size() < 2 || plan.platformPose.size() != plan.timeS.size()) {
        errorMessage = QStringLiteral("期望轨迹为空或时间轴不完整。");
        return false;
    }
    const QByteArray payload = serializeExpectedTrajectory(plan);
    sha256Value = QString::fromLatin1(
        QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex());
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        errorMessage = QStringLiteral("无法创建轨迹目录：%1").arg(QFileInfo(path).absolutePath());
        return false;
    }
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)
        || file.write(payload) != payload.size()
        || !file.commit()) {
        errorMessage = QStringLiteral("无法写入期望轨迹：%1").arg(file.errorString());
        return false;
    }
    return true;
}

bool CdprTrajectoryFile::prepareCache(CdprOfflinePvtPlan &plan,
                                      const QString &cacheRoot,
                                      QString &errorMessage)
{
    const QString stagingPath = QDir(cacheRoot).filePath(QStringLiteral("pending.csv"));
    QString hash;
    if (!exportExpectedTrajectory(plan, stagingPath, hash, errorMessage)) {
        return false;
    }
    QCryptographicHash identityHash(QCryptographicHash::Sha256);
    identityHash.addData(plan.configurationSnapshotJson.toUtf8());
    identityHash.addData(QByteArrayView("\n", 1));
    identityHash.addData(hash.toLatin1());
    const QString planId = QString::fromLatin1(identityHash.result().toHex());
    const QString directory = QDir(cacheRoot).filePath(planId);
    const QString targetPath = QDir(directory).filePath(QStringLiteral("expected_trajectory.csv"));
    if (!QDir().mkpath(directory)) {
        QFile::remove(stagingPath);
        errorMessage = QStringLiteral("无法创建轨迹缓存目录：%1").arg(directory);
        return false;
    }
    if (!QFileInfo::exists(targetPath)) {
        if (!QFile::rename(stagingPath, targetPath)) {
            QFile::remove(stagingPath);
            errorMessage = QStringLiteral("无法提交轨迹缓存文件：%1").arg(targetPath);
            return false;
        }
    } else {
        QFile::remove(stagingPath);
        if (!verify(targetPath, hash, errorMessage)) {
            return false;
        }
    }
    plan.planId = planId;
    plan.expectedTrajectoryPath = QFileInfo(targetPath).absoluteFilePath();
    plan.expectedTrajectorySha256 = hash;
    return true;
}

QString CdprTrajectoryFile::sha256(const QString &path, QString &errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        errorMessage = QStringLiteral("无法读取轨迹缓存：%1").arg(file.errorString());
        return {};
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) {
        errorMessage = QStringLiteral("计算轨迹缓存SHA-256失败：%1").arg(path);
        return {};
    }
    return QString::fromLatin1(hash.result().toHex());
}

bool CdprTrajectoryFile::verify(const QString &path,
                                const QString &expectedSha256,
                                QString &errorMessage)
{
    if (path.isEmpty() || expectedSha256.isEmpty()) {
        errorMessage = QStringLiteral("轨迹缓存路径或SHA-256为空。");
        return false;
    }
    const QString actual = sha256(path, errorMessage);
    if (actual.isEmpty()) {
        return false;
    }
    if (actual.compare(expectedSha256, Qt::CaseInsensitive) != 0) {
        errorMessage = QStringLiteral("轨迹缓存SHA-256不匹配；请重新生成轨迹。");
        return false;
    }
    return true;
}
