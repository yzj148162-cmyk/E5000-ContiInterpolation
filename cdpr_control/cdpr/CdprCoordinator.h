#ifndef CDPRCOORDINATOR_H
#define CDPRCOORDINATOR_H

#include <QObject>

#include "CdprConfiguration.h"
#include "common/ContiTypes.h"

class CdprCoordinator : public QObject
{
    Q_OBJECT

public:
    explicit CdprCoordinator(QObject *parent = nullptr);

public slots:
    void loadConfiguration(const QString &path);
    void validateConfiguration();
    void writeConfigurationTemplate(const QString &path);
    void updateHardwareStatus(const ContiStatus &status);

signals:
    void statusChanged(const CdprUiStatus &status);
    void logMessage(const QString &message);

private:
    void publishStatus();

    CdprConfiguration configuration_;
    QString configurationPath_;
    QStringList validationMessages_;
    bool configurationLoaded_ = false;
    bool boardInitialized_ = false;
    int detectedAxisCount_ = 0;
    quint16 enabledAxisMask_ = 0;
};

#endif // CDPRCOORDINATOR_H
