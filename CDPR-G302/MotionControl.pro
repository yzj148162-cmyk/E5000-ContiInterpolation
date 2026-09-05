# 工程总览：
# - 这是 MotionControl（G302 缆驱机器人控制软件）的 qmake 工程文件，负责声明 Qt 模块、源码、头文件、UI 和外部库。
# - 项目业务源码按功能放在 pose_control_module、control_interface_module、embedded_module；third_party、ALGLIB、Nokov/雷赛 SDK 等只作为依赖接入。
# - 新人阅读代码时建议从 control_interface_module/main.cpp -> MainWindow -> HardwareInterface/ControlWorker/SafetyMonitor -> 仿真与轨迹模块的顺序进入。

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += datavisualization
QT += serialport
QT += printsupport
QT += network

CONFIG += c++17
CONFIG += compile_commands_json
QMAKE_PROJECT_DEPTH = 0
MOC_DIR = $$OUT_PWD/moc
TARGET = MotionControl
MOTION_CONTROL_APP_VERSION = 1.0.0
VERSION = $$MOTION_CONTROL_APP_VERSION

SOURCES += \
    control_interface_module/curvedrawer.cpp \
    control_interface_module/datavisualizationcontroller.cpp \
    control_interface_module/endpointremoteinputsupervisor.cpp \
    control_interface_module/guitimingdiagnostics.cpp \
    control_interface_module/infolabel.cpp \
    control_interface_module/intbtn.cpp \
    control_interface_module/machinekinematicsprofile.cpp \
    control_interface_module/main.cpp \
    control_interface_module/mainwindow.cpp \
    control_interface_module/maintenancepackagevalidator.cpp \
    control_interface_module/outputpathvalidator.cpp \
    control_interface_module/qcustomplot.cpp \
    control_interface_module/runtimejsoncodec.cpp \
    control_interface_module/runtimediagnostics.cpp \
    control_interface_module/sessionrecorder.cpp \
    control_interface_module/softwarefaultguard.cpp \
    control_interface_module/structuredfaultlogwriter.cpp \
    control_interface_module/x56directinputreader.cpp \
    control_interface_module/x56inputworker.cpp \
    pose_control_module/MatrixFun.cpp \
    pose_control_module/barycenter.cpp \
    pose_control_module/controlworker.cpp \
    pose_control_module/cdprdynamics.cpp \
    pose_control_module/compensatedcablekinematics.cpp \
    pose_control_module/endpointremotecontrol.cpp \
    pose_control_module/forceinteractionrunrecorder.cpp \
    pose_control_module/forceinteractionruntimecontrol.cpp \
    pose_control_module/forceinteractionsoftwarevalidator.cpp \
    pose_control_module/forcecontroller.cpp \
    pose_control_module/forwardkinematicssolver.cpp \
    pose_control_module/forcepid0525.cpp \
    pose_control_module/kalmanhandler.cpp \
    pose_control_module/linalg.cpp \
    pose_control_module/nokovposecalculator.cpp \
    pose_control_module/onlinevelocitycontrol.cpp \
    pose_control_module/physicalworkspaceboundary.cpp \
    pose_control_module/optimization.cpp \
    pose_control_module/pidhandler.cpp \
    pose_control_module/positionsimulationmodel.cpp \
    pose_control_module/pvtexecutionworker.cpp \
    pose_control_module/ropeelasticcompensation.cpp \
    pose_control_module/simulationworker.cpp \
    pose_control_module/solvers.cpp \
    pose_control_module/trajectorygenerator.cpp \
    pose_control_module/trajectoryplanner.cpp \
    pose_control_module/wrenchsource.cpp \
    pose_control_module/wrenchtransformer.cpp \
    pose_control_module/winchcompensation.cpp \
    embedded_module/NokovMinimalClient.cpp \
    embedded_module/hardwareinterface.cpp \
    embedded_module/hardwareinterface_jogfast.cpp \
    embedded_module/monitorthread.cpp \
    embedded_module/motivelocalhandlerthread.cpp \
    embedded_module/motortorquetestworker.cpp \
    embedded_module/safetymonitor.cpp \
    embedded_module/udpfeedbacksender.cpp \
    embedded_module/udpcommworker.cpp \
    third_party/alglib/alglibinternal.cpp \
    third_party/alglib/alglibmisc.cpp \
    third_party/alglib/ap.cpp \



HEADERS += \
    control_interface_module/runtimefeatureswitches.h \
    third_party/lt_dmc/LTDMC.h \
    control_interface_module/csvexportutils.h \
    control_interface_module/curvedrawer.h \
    control_interface_module/datavisualizationcontroller.h \
    control_interface_module/endpointremoteinputsupervisor.h \
    control_interface_module/guitimingdiagnostics.h \
    control_interface_module/infolabel.h \
    control_interface_module/intbtn.h \
    control_interface_module/machinekinematicsprofile.h \
    control_interface_module/mainwindow.h \
    control_interface_module/maintenancepackagevalidator.h \
    control_interface_module/outputpathvalidator.h \
    control_interface_module/qcustomplot.h \
    control_interface_module/runtimejsoncodec.h \
    control_interface_module/runtimediagnostics.h \
    control_interface_module/sessionrecorder.h \
    control_interface_module/softwarefaultguard.h \
    control_interface_module/structuredfaultlogwriter.h \
    control_interface_module/x56directinputreader.h \
    control_interface_module/x56inputworker.h \
    pose_control_module/MatrixFun.h \
    pose_control_module/barycenter.h \
    pose_control_module/controlworker.h \
    pose_control_module/cdprdynamics.h \
    pose_control_module/compensatedcablekinematics.h \
    pose_control_module/endpointremotecontrol.h \
    pose_control_module/forceinteractionrunrecorder.h \
    pose_control_module/forceinteractionruntimecontrol.h \
    pose_control_module/forceinteractionsoftwarevalidator.h \
    pose_control_module/forceinteractiontypes.h \
    pose_control_module/forcecontroller.h \
    pose_control_module/forwardkinematicssolver.h \
    pose_control_module/forcepid0525.h \
    pose_control_module/kalmanhandler.h \
    pose_control_module/linalg.h \
    pose_control_module/nokovposecalculator.h \
    pose_control_module/onlinevelocitycontrol.h \
    pose_control_module/physicalworkspaceboundary.h \
    pose_control_module/optimization.h \
    pose_control_module/pidhandler.h \
    pose_control_module/positionsimulationmodel.h \
    pose_control_module/pvtexecutionworker.h \
    pose_control_module/ropeelasticcompensation.h \
    pose_control_module/simulationworker.h \
    pose_control_module/solvers.h \
    pose_control_module/trajectorygenerator.h \
    pose_control_module/trajectoryplanner.h \
    pose_control_module/wrenchsource.h \
    pose_control_module/wrenchtransformer.h \
    pose_control_module/winchcompensation.h \
    embedded_module/NokovMinimalClient.h \
    embedded_module/hardwareinterface.h \
    embedded_module/macro.h \
    embedded_module/monitorthread.h \
    embedded_module/runtimepathutils.h \
    embedded_module/motivelocalhandlerthread.h \
    embedded_module/motortorquetestworker.h \
    embedded_module/safetymonitor.h \
    embedded_module/udpfeedbacksender.h \
    embedded_module/udpcommworker.h \
    embedded_module/udppackettypes.h \
    third_party/alglib/alglibinternal.h \
    third_party/alglib/alglibmisc.h \
    third_party/alglib/ap.h \

# mainwindow.cpp 直接包含源码目录中的 ui_mainwindow.h；将 uic 输出固定到该目录，
# 避免构建目录中的生成文件与实际参与编译的界面头文件不一致。
UI_DIR = $$PWD/control_interface_module

FORMS += \
    mainwindow.ui

win32-msvc* {
    QMAKE_CXXFLAGS += /bigobj
    LIBS += user32.lib dinput8.lib dxguid.lib
}
win32-g++:LIBS += -luser32 -ldinput8 -ldxguid

DEFINES += USE_NOKOV
DEFINES += AE_NO_SSE2 AE_NO_AVX2 AE_NO_FMA
DEFINES += G302_SOURCE_DIR=\\\"$$PWD\\\"
DEFINES += MOTION_CONTROL_APP_VERSION=\\\"$$MOTION_CONTROL_APP_VERSION\\\"

DISTFILES += \
    2.5m_k.xlsx \
    4.5m_k.xlsx \
    assets/workspace_envelope_2p5m.obj \
    assets/workspace_envelope_4p5m.obj \
    tools/generate_workspace_envelope_obj.py

INCLUDEPATH += $$PWD
INCLUDEPATH += $$PWD/pose_control_module
INCLUDEPATH += $$PWD/control_interface_module
INCLUDEPATH += $$PWD/embedded_module
INCLUDEPATH += $$PWD/third_party/eigen
INCLUDEPATH += $$PWD/third_party/alglib
INCLUDEPATH += $$PWD/third_party/lt_dmc
INCLUDEPATH += $$PWD/third_party/Nokov/include

NLOPT_DIR = $$PWD/third_party/nlopt/nlopt-2.4.2-dll64
INCLUDEPATH += $$NLOPT_DIR
win32-msvc*:LIBS += $$quote($$NLOPT_DIR/libnlopt-0.lib)
else:LIBS += -L$$NLOPT_DIR -llibnlopt-0
LIBS += -L$$PWD/third_party/lt_dmc -lLTDMC
LIBS += -L$$PWD/third_party/Nokov/lib -lnokov_sdk
