#ifndef MACHINEKINEMATICSPROFILE_H
#define MACHINEKINEMATICSPROFILE_H

#include "hardwareinterface.h"
#include "physicalworkspaceboundary.h"

#include <array>
#include <vector>

class QDoubleSpinBox;

namespace Ui {
class MainWindow;
}

// ACC/G302 机型的不可变运动学、轴映射和传感器参数表。
namespace MachineKinematicsProfileCatalog {

enum class MachineProfileKind {
    ACC,
    G302
};

struct CablePointProfile {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct MachineKinematicsProfile {
    const char* name = "";
    HardwareInterface::RuntimeTraceConfigType runtimeTraceConfigType =
            HardwareInterface::RuntimeTraceConfigType::ACC;
    int axisCount = 0;
    int modeledCableAxisCount = 0;
    bool linearModuleHeightModuleEnabled = true;
    int forceSensorDataLen = 2;
    bool forceSensorSigned = false;
    bool staticSensorHome = false;
    double frameLengthMm = 0.0;
    double frameWidthMm = 0.0;
    double frameHeightMm = 0.0;
    double pulleyRadiusMm = 0.0;
    double workspaceZMinMm = 0.0;
    double workspaceWarningMarginMm = 0.0;
    double workspaceSevereOverflowMm = 0.0;
    // G302末端开环遥控专用的固定保守平移工作空间。姿态不在下述平动安全范围时，
    // 平移会被禁止，只允许先用转动模式恢复。
    std::array<double, 3> endpointRemoteWorkspaceMinimumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> endpointRemoteWorkspaceMaximumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> endpointRemoteReferenceOrientationRad{{0.0, 0.0, 0.0}};
    // 程序起点与可信实测姿态的一致性阈值。
    double endpointRemoteOrientationToleranceRad = 0.0;
    // 末端转动的临时占位硬边界，以及允许平动的更小姿态范围。
    std::array<double, 3> endpointRemoteOrientationMinimumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> endpointRemoteOrientationMaximumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> endpointRemoteTranslationSafeOrientationMinimumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> endpointRemoteTranslationSafeOrientationMaximumRad{{0.0, 0.0, 0.0}};
    double endpointRemoteSoftBoundaryMarginMm = 0.0;
    double endpointRemoteBoundaryReleaseHysteresisMm = 0.0;
    double barycenterForceMinN = 0.0;
    double barycenterForceMaxN = 0.0;
    double leadshineRatedMotorTorqueNm = 0.0;
    double torqueServoVelocityLimitRpm = 0.0;
    double cableWinchRadiusMm = 0.0;
    // 绳索收紧控制量到电机位置/力矩命令的机型方向符号：ACC=+1，G302=-1。
    double cableMotorDirectionSign = 1.0;
    double winchPitchMmPerRev = 0.0;
    double winchProjectionMm = 0.0;
    double cableMotorPosLimitRev = 0.0;
    double linearMotorPosLimitUnit = 0.0;
    double cableMotorVelLimitRevPerSec = 0.0;
    double otherMotorVelLimitRevPerSec = 0.0;
    double cableForceLimitN = 0.0;
    double actualTorqueLimitNm = 0.0;
    std::vector<int> winchCompensatedCableNumbers;
    std::vector<int> hardwareAxisNo;
    std::vector<int> slaveId;
    std::vector<double> forceSensorCof;
    std::vector<double> ropeElasticFixedLengthL0Mm;
    std::vector<CablePointProfile> cableEndPoints;
    std::vector<CablePointProfile> cableStartPoints;
};

const MachineKinematicsProfile& machineKinematicsProfile(MachineProfileKind kind);
MachineProfileKind currentMachineProfileKind(const Ui::MainWindow* ui);
const MachineKinematicsProfile& currentMachineKinematicsProfile(
        const Ui::MainWindow* ui);
PhysicalWorkspaceBoundaryConfig physicalWorkspaceBoundaryConfig(
        const MachineKinematicsProfile& profile);
void applyCablePointProfile(const std::vector<QDoubleSpinBox*>& xSpin,
                            const std::vector<QDoubleSpinBox*>& ySpin,
                            const std::vector<QDoubleSpinBox*>& zSpin,
                            const std::vector<CablePointProfile>& points);

} // namespace MachineKinematicsProfileCatalog

#endif // MACHINEKINEMATICSPROFILE_H
