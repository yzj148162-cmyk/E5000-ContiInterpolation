QT += core gui widgets charts

CONFIG += c++17

TARGET = cdpr_control
TEMPLATE = app

# 独立工程；头文件、导入库和运行时 DLL 统一使用本工程自带的雷赛 SDK。
LEADSHINE_DIR = $$PWD/third_party/lt_dmc

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    cdpr/CdprConfiguration.cpp \
    cdpr/CdprCoordinator.cpp \
    control/PositionVelocityPid.cpp \
    control/TraceDelayCalibration.cpp \
    hardware/LeadshineMotionCard.cpp \
    hardware/MotionCardHardwareInterface.cpp \
    hardware/RuntimeTraceSlaveReader.cpp \
    hardware/MotionControlWorker.cpp \
    telemetry/TelemetryRecorder.cpp \
    planner/QuinticTrajectory.cpp \
    widgets/ZoomableChartView.cpp

HEADERS += \
    cdpr/CdprConfiguration.h \
    cdpr/CdprCoordinator.h \
    common/ContiTypes.h \
    control/PositionVelocityPid.h \
    control/TraceDelayCalibration.h \
    hardware/LeadshineMotionCard.h \
    hardware/MotionCardHardwareInterface.h \
    hardware/RuntimeTraceSlaveReader.h \
    hardware/MotionControlWorker.h \
    telemetry/TelemetryRecorder.h \
    mainwindow.h \
    planner/QuinticTrajectory.h \
    widgets/ZoomableChartView.h

FORMS += mainwindow.ui

INCLUDEPATH += \
    $$PWD \
    $$LEADSHINE_DIR

LIBS += -L$$LEADSHINE_DIR -lLTDMC
win32:LIBS += Version.lib

exists($$LEADSHINE_DIR/LTDMC.h) {
    message(Leadshine header found: $$LEADSHINE_DIR/LTDMC.h)
} else {
    error(Leadshine SDK header missing: $$LEADSHINE_DIR/LTDMC.h)
}

exists($$LEADSHINE_DIR/LTDMC.lib) {
    message(Leadshine library found: $$LEADSHINE_DIR/LTDMC.lib)
} else {
    error(Leadshine SDK library missing: $$LEADSHINE_DIR/LTDMC.lib)
}

LTDMC_DLL_WIN = $$replace(LEADSHINE_DIR, /, \\)\\LTDMC.dll
OUT_PWD_WIN = $$replace(OUT_PWD, /, \\)

win32:CONFIG(debug, debug|release) {
    QMAKE_POST_LINK += cmd /c "if not exist \"$$OUT_PWD_WIN\\debug\" mkdir \"$$OUT_PWD_WIN\\debug\" && copy /y \"$$LTDMC_DLL_WIN\" \"$$OUT_PWD_WIN\\debug\\LTDMC.dll\" > nul"
}

win32:CONFIG(release, debug|release) {
    QMAKE_POST_LINK += cmd /c "if not exist \"$$OUT_PWD_WIN\\release\" mkdir \"$$OUT_PWD_WIN\\release\" && copy /y \"$$LTDMC_DLL_WIN\" \"$$OUT_PWD_WIN\\release\\LTDMC.dll\" > nul"
}
