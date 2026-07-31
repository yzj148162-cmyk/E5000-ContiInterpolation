#ifndef CDPRCOORDINATOR_H
#define CDPRCOORDINATOR_H

#include <QObject>
#include <memory>

#include "CdprConfiguration.h"
#include "CdprDynamics.h"
#include "CdprForceInput.h"
#include "CdprKinematics.h"
#include "NokovMarkerProvider.h"
#include "common/ContiTypes.h"

class QTimer;

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
    void setInitialPoseSource(int source);
    void setPresetInitialPose(double x, double y, double z,
                              double roll, double pitch, double yaw);
    void connectNokov(const QString &serverAddress);
    void disconnectNokov();
    void captureInitialState();
    void setForceInputSource(int source);
    void setSimulatedSensorWrench(double fx, double fy, double fz,
                                  double mx, double my, double mz);
    void clearSimulatedSensorWrench();
    void resetDynamics();
    void advanceDynamicsOnce();
    void prepareOfflinePvt(const CdprOfflinePvtRequest &request);

signals:
    void statusChanged(const CdprUiStatus &status);
    void logMessage(const QString &message);
    void offlinePvtPlanReady(const CdprOfflinePvtPlan &plan);

private:
    void rebuildInitialKinematics();
    bool resetDynamicsFromSelectedPose(QString *errorText = nullptr);
    CdprWrenchTransformResult currentPlatformWrench(
        const CdprFrameStamp &stamp) const;
    void publishStatus();

    CdprConfiguration configuration_;
    std::unique_ptr<CdprKinematics> kinematics_;
    std::unique_ptr<CdprWrenchTransformer> wrenchTransformer_;
    CdprDynamics dynamics_;
    SimulatedWrenchSource simulatedWrenchSource_;
    TraceFtSensorSource traceFtSensorSource_;
    NokovMarkerProvider nokovProvider_;
    PendingMarkerGeometryPoseEstimator markerPoseEstimator_;
    CdprStartupState startupState_;
    CdprRobotState robotState_;
    CdprInitialPoseSource initialPoseSource_ =
        CdprInitialPoseSource::Preset;
    CdprForceInputSource forceInputSource_ =
        CdprForceInputSource::Simulated;
    CdprVector6 presetInitialPose_ {};
    QString kinematicsSummary_;
    QString kinematicsError_;
    QString dynamicsError_;
    QString configurationPath_;
    QStringList validationMessages_;
    bool configurationLoaded_ = false;
    bool kinematicsReady_ = false;
    bool boardInitialized_ = false;
    int detectedAxisCount_ = 0;
    quint16 enabledAxisMask_ = 0;
    quint64 previewSequence_ = 0;
    QTimer *statusTimer_ = nullptr;
};

#endif // CDPRCOORDINATOR_H
