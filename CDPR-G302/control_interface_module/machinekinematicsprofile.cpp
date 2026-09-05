#include "machinekinematicsprofile.h"

#include "ui_mainwindow.h"

#include <QDoubleSpinBox>

#include <algorithm>

namespace MachineKinematicsProfileCatalog {
namespace {

constexpr double kBarycenterForceMinN = 20.0;
constexpr double kBarycenterForceMaxN = 997.0;
constexpr double kTorqueServoVelocityLimitRpm = 600.01;
constexpr double kAccActualTorqueLimitNm = 60.0;
constexpr double kG302ActualTorqueLimitNm = 40.0;

} // namespace

const MachineKinematicsProfile& machineKinematicsProfile(MachineProfileKind kind)
{
    static const MachineKinematicsProfile accProfile{
        "ACC",
        HardwareInterface::RuntimeTraceConfigType::ACC,
        12,
        8,
        true,
        2,
        false,
        false,
        4400.0,
        4800.0,
        4500.0,
        18.5,
        -120.0,
        300.0,
        100.0,
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        0.0,
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        0.0,
        0.0,
        kBarycenterForceMinN,
        kBarycenterForceMaxN,
        45.0,
        kTorqueServoVelocityLimitRpm,
        43.0,
        1.0,
        7.0,
        1027.0,
        14.0,
        300.0,
        10.0,
        10.0,
        997.0,
        kAccActualTorqueLimitNm,
        {2, 4, 6, 8},
        {0, 1, 5, 4, 6, 7, 11, 10, 2, 3, 8, 9},
        {1001, 1002, 1007, 1006, 1008, 1009, 1014, 1013, 1003, 1005, 1010, 1012},
        {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        {6972.18, 2865.71, 6972.18, 2865.71, 6972.18, 2865.71, 6972.18, 2865.71},
        {
            {-482.133, 156.7446, -253.582},
            {-298.034, 410.3261, 253.5815},
            {482.1332, 156.7446, -253.582},
            {298.0335, 410.3261, 253.5815},
            {298.0335, -410.326, -253.582},
            {482.1332, -156.745, 253.5815},
            {-298.034, -410.326, -253.582},
            {-482.133, -156.745, 253.5815}
        },
        {
            {-2955.589743, 2616.04406, 2497.516755},
            {-2736.593954, 2951.657846, -1.05113897},
            {2947.376464, 2609.008031, 2497.343844},
            {2737.5506, 2954.589789, 1.29644947},
            {2943.078959, -2609.243625, 2496.856358},
            {2738.539461, -2953.55438, -1.27683448},
            {-2956.60384, -2603.160837, 2496.707093},
            {-2739.496107, -2952.693255, 1.03152398}
        }
    };

    static const MachineKinematicsProfile g302Profile{
        "G302",
        HardwareInterface::RuntimeTraceConfigType::G302,
        8,
        8,
        false,
        4,
        true,
        false,
        2800.0,
        2800.0,
        2670.0,
        17.5,
        -100.0,
        300.0,
        100.0,
        {{-500.0, -500.0, 400.0}},
        {{500.0, 500.0, 1400.0}},
        {{0.0, 0.0, 0.0}},
        0.04363323129985824, // 2.5 deg
        {{-0.17453292519943295, -0.17453292519943295, -0.08726646259971647}}, // -10/-10/-5 deg
        {{ 0.17453292519943295,  0.17453292519943295,  0.08726646259971647}}, // +10/+10/+5 deg
        {{-0.08726646259971647, -0.08726646259971647, -0.04363323129985824}}, // -5/-5/-2.5 deg
        {{ 0.08726646259971647,  0.08726646259971647,  0.04363323129985824}}, // +5/+5/+2.5 deg
        100.0,
        20.0,
        5.0,
        400.0,
        34.5,
        450.0,
        76.0,
        -1.0,
        5.5,
        680.0,
        7.0,
        300.0,
        7.5,
        10.0,
        400.0,
        kG302ActualTorqueLimitNm,
        {2, 4, 6, 8},
        {4, 5, 7, 6, 0, 1, 3, 2},
        {1005, 1006, 1008, 1007, 1001, 1002, 1004, 1003},
        {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        {2900.0, 1550.0, 2900.0, 1550.0, 2900.0, 1550.0, 2900.0, 1550.0},
        {
            {-235.61, 76.55, -123.87},
            {-145.62, 200.42, 123.87},
            {235.61, 76.55, -123.87},
            {145.62, 200.42, 123.87},
            {145.62, -200.42, -123.87},
            {235.61, -76.55, 123.87},
            {-145.62, -200.42, -123.87},
            {-235.61, -76.55, 123.87}
        },
        {
            {-1359.7417694037499, 1227.8132062007601, 1794.62460652498},
            {-1223.4261986031499, 1359.10538149128, 2.4284245247785989},
            {1364.2567526065, 1227.3728621052101, 1787.7869400019599},
            {1225.06183529901, 1361.8618405174, -2.319708671885099},
            {1366.55150907436, -1220.2830235679501, 1790.4813218241},
            {1222.2815475284799, -1360.0457759840899, 2.3377208657031012},
            {-1354.5388896793299, -1220.15331356819, 1793.0807305997801},
            {-1223.91718422434, -1360.9214460245901, -2.4464367185963987}
        }
    };

    return kind == MachineProfileKind::G302 ? g302Profile : accProfile;
}

MachineProfileKind currentMachineProfileKind(const Ui::MainWindow* ui)
{
    return ui && ui->devUseLite && ui->devUseLite->isChecked()
            ? MachineProfileKind::G302
            : MachineProfileKind::ACC;
}

const MachineKinematicsProfile& currentMachineKinematicsProfile(const Ui::MainWindow* ui)
{
    return machineKinematicsProfile(currentMachineProfileKind(ui));
}

PhysicalWorkspaceBoundaryConfig physicalWorkspaceBoundaryConfig(
        const MachineKinematicsProfile& profile)
{
    PhysicalWorkspaceBoundaryConfig config;
    config.frameMinimumMm = {{-profile.frameLengthMm * 0.5,
                              -profile.frameWidthMm * 0.5,
                              profile.workspaceZMinMm}};
    config.frameMaximumMm = {{profile.frameLengthMm * 0.5,
                              profile.frameWidthMm * 0.5,
                              profile.frameHeightMm}};
    config.platformPointsLocalMm.reserve(profile.cableEndPoints.size());
    for(const CablePointProfile& point : profile.cableEndPoints){
        config.platformPointsLocalMm.push_back({{point.x, point.y, point.z}});
    }
    // 当前两套模板尚无经过实机验收的全局姿态硬边界。几何边界始终启用；
    // 姿态边界待参数确认后在同一配置中开启，不能借用遥控专用的小角度范围。
    config.orientationBoundsEnabled = false;
    return config;
}

void applyCablePointProfile(const std::vector<QDoubleSpinBox*>& xSpin,
                            const std::vector<QDoubleSpinBox*>& ySpin,
                            const std::vector<QDoubleSpinBox*>& zSpin,
                            const std::vector<CablePointProfile>& points)
{
    const int pointCount = std::min({static_cast<int>(points.size()),
                                     static_cast<int>(xSpin.size()),
                                     static_cast<int>(ySpin.size()),
                                     static_cast<int>(zSpin.size())});
    for(int i = 0; i < pointCount; ++i){
        if(xSpin[i]){
            xSpin[i]->setValue(points[i].x);
        }
        if(ySpin[i]){
            ySpin[i]->setValue(points[i].y);
        }
        if(zSpin[i]){
            zSpin[i]->setValue(points[i].z);
        }
    }
}


} // namespace MachineKinematicsProfileCatalog
