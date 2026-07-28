#ifndef CDPRCONFIGURATION_H
#define CDPRCONFIGURATION_H

#include <array>

#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVector>

struct CdprVector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct CdprCableAxisConfig
{
    int cable = 0;
    int axis = 0;
    int direction = 1;
    CdprVector3 frameAnchorM;
    CdprVector3 platformAnchorM;
    double initialCableLengthM = 0.0;
    double drumRadiusM = 0.0;
    double minimumCableLengthM = 0.0;
    double maximumCableLengthM = 0.0;
};

struct CdprRigidBodyConfig
{
    double massKg = 0.0;
    CdprVector3 centerOfMassM;
    std::array<double, 9> inertiaKgM2 {};
};

struct CdprForceSensorConfig
{
    CdprVector3 originInPlatformM;
    std::array<double, 9> rotationSensorToPlatform {};
    int sensorSign = 1;
};

struct CdprConfiguration
{
    QString name;
    QString coordinateConvention;
    bool parametersConfirmed = false;
    std::array<double, 6> initialPlatformPose {};
    CdprRigidBodyConfig physicalPlatform;
    CdprRigidBodyConfig virtualBody;
    CdprForceSensorConfig forceSensor;
    std::array<CdprCableAxisConfig, 8> cables {};
    int controlPeriodUs = 1000;
    double maximumPositionErrorDegree = 2.0;
};

struct CdprAxisView
{
    int cable = 0;
    int axis = 0;
    int direction = 1;
    QString frameAnchor;
    QString platformAnchor;
    bool online = false;
    bool enabled = false;
};

struct CdprUiStatus
{
    bool configurationLoaded = false;
    bool configurationValid = false;
    QString configurationPath;
    QString configurationId;
    QString stateText = QStringLiteral("未加载配置");
    QString summary;
    QStringList validationMessages;
    QVector<CdprAxisView> axes;
    int onlineAxisCount = 0;
    bool boardInitialized = false;
    bool controlStartAvailable = false;
};

class CdprConfigurationFile
{
public:
    static bool load(const QString &path, CdprConfiguration &configuration,
                     QStringList &errors);
    static bool writeTemplate(const QString &path, QString &error);
    static QStringList validate(const CdprConfiguration &configuration);
    static QString identifier(const CdprConfiguration &configuration);
    static QString summary(const CdprConfiguration &configuration);
};

Q_DECLARE_METATYPE(CdprUiStatus)

#endif // CDPRCONFIGURATION_H
