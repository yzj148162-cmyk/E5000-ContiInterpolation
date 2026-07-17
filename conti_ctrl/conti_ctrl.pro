QT += core gui widgets charts

CONFIG += c++17

TARGET = conti_ctrl
TEMPLATE = app

# 独立工程；暂时引用工作区内已提供的雷赛 SDK 二进制文件，不会修改 pos_ctrl。
LEADSHINE_DIR = $$PWD/../pos_ctrl/third_party/leadshine

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    hardware/E5000ContiInterface.cpp \
    hardware/E5000HardwareInterface.cpp \
    hardware/RuntimeTraceSlaveReader.cpp \
    hardware/ContiWorker.cpp \
    telemetry/TelemetryRecorder.cpp \
    planner/QuinticTrajectory.cpp

HEADERS += \
    common/ContiTypes.h \
    hardware/E5000ContiInterface.h \
    hardware/E5000HardwareInterface.h \
    hardware/RuntimeTraceSlaveReader.h \
    hardware/ContiWorker.h \
    telemetry/TelemetryRecorder.h \
    mainwindow.h \
    planner/QuinticTrajectory.h

FORMS += mainwindow.ui

INCLUDEPATH += \
    $$PWD \
    $$LEADSHINE_DIR

LIBS += -L$$LEADSHINE_DIR -lLTDMC

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
