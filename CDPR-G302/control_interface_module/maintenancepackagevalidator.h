#ifndef MAINTENANCEPACKAGEVALIDATOR_H
#define MAINTENANCEPACKAGEVALIDATOR_H

#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>

// 维护更新包的版本识别、清单校验及 assets 载荷安全写入。
namespace MaintenancePackageValidator {

struct MaintenanceUpdatePackageMetadata
{
    QString packageName;
    QString packageId;
    QString packageVersion;
    QString targetSoftware;
    QString buildTime;
    QString releaseNotes;
    QStringList payloadSummaries;
    int payloadCount = 0;
    QString versionSource;
    QString manifestSource;
    QString manifestError;
    QJsonObject manifest;
};

QStringList maintenanceUpdatePackageAllowedSuffixes();
QJsonObject applyMaintenanceUpdatePackagePayloads(
        const MaintenanceUpdatePackageMetadata& metadata,
        bool* ok);
bool parseMaintenanceVersionComponents(const QString& rawVersion,
                                       QVector<int>& components,
                                       QString* normalized = nullptr,
                                       QString* errorMessage = nullptr);
int compareMaintenanceVersions(const QVector<int>& left,
                               const QVector<int>& right);
QString currentMaintenanceSoftwareVersion(QString* source = nullptr);
bool extractMaintenanceUpdatePackageMetadata(
        const QString& filePath,
        MaintenanceUpdatePackageMetadata& metadata,
        QString& errorMessage);
QJsonObject maintenanceUpdatePackageMetadataToJson(
        const MaintenanceUpdatePackageMetadata& metadata);

} // namespace MaintenancePackageValidator

#endif // MAINTENANCEPACKAGEVALIDATOR_H
