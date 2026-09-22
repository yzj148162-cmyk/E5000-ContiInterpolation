#ifndef FORCEINTERACTIONREPLAYEXPORTER_H
#define FORCEINTERACTIONREPLAYEXPORTER_H

#include "compensatedcablekinematics.h"
#include "onlinevelocitycontrol.h"
#include "physicalworkspaceboundary.h"

#include <QString>

#include <array>
#include <vector>

// Frozen, self-contained description of one force-interaction run.  MATLAB
// consumes the JSON sidecar written from this structure and never reads live
// UI settings or files from the legacy MATLAB project.
struct ForceInteractionReplayExportContext
{
    QString csvPath;
    QString stageName;
    QString wrenchSourceName;
    QString machineTemplateName;
    QString actuatorTemplateName;
    int controlPeriodUs = 0;
    int tracePeriodUs = 0;
    bool translationOnly = false;

    CompensatedCableKinematics::Configuration kinematics;
    PhysicalWorkspaceBoundaryConfig physicalWorkspace;
    DynamicWorkspaceSafetyConfig workspaceSafety;
    std::vector<double> initialPoseMmRad;
    std::vector<double> referenceCableLengthMm;
    OnlineVelocityAxisArray motorUnitPerRadian{};
    OnlineVelocityAxisArray motorSafetyRelativeMinimum{};
    OnlineVelocityAxisArray motorSafetyRelativeMaximum{};
    OnlineVelocityAxisArray actualStartPosition{};
    OnlineVelocityAxisArray actualStartSafetyRelativePosition{};
    std::array<double, kOnlineVelocityAxisCount> traceDelayMs{};
    std::array<bool, kOnlineVelocityAxisCount> traceDelayValid{};
    std::array<int, kOnlineVelocityAxisCount> hardwareAxis{};
    std::array<int, kOnlineVelocityAxisCount> slaveId{};
    std::array<double, kOnlineVelocityAxisCount> pulsePerUnit{};
    std::array<double, kOnlineVelocityAxisCount> hardwareDirectionSign{};

    int terminalState = 0;
    QString terminalMessage;
    QString safetyStopReason;
    bool experimentValid = false;
    quint64 commandCount = 0;
    quint64 writtenRecordCount = 0;
    quint64 droppedRecordCount = 0;
};

class ForceInteractionReplayExporter
{
public:
    static QString sidecarPathForCsv(const QString& csvPath);
    static bool writeJson(const ForceInteractionReplayExportContext& context,
                          QString* outputPath = nullptr,
                          QString* errorMessage = nullptr);
};

#endif // FORCEINTERACTIONREPLAYEXPORTER_H
