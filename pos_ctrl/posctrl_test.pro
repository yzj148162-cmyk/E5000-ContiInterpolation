QT += core gui widgets

CONFIG += c++17

TARGET = posctrl_test
TEMPLATE = app

LEADSHINE_DIR = $$PWD/third_party/leadshine

SOURCES += \
    hardware/EthercatInterface.cpp \
    hardware/HardwareThread.cpp \
    hardware/PdoAccess.cpp \
    main.cpp \
    mainwindow.cpp \
    planner/PlannerThread.cpp \
    planner/QuinticPlanner.cpp \
    ui/UiRefreshController.cpp

HEADERS += \
    common/FeedbackData.h \
    common/MotionRequest.h \
    common/SharedContext.h \
    common/ThreadSafeQueue.h \
    common/TrajectoryPoint.h \
    config/MotionConfig.h \
    config/SystemConfig.h \
    hardware/EthercatInterface.h \
    hardware/HardwareThread.h \
    hardware/PdoAccess.h \
    mainwindow.h \
    planner/PlannerThread.h \
    planner/QuinticPlanner.h \
    planner/TrajectoryPlanner.h \
    ui/UiRefreshController.h

FORMS += \
    mainwindow.ui

INCLUDEPATH += \
    $$PWD \
    $$PWD/common \
    $$PWD/config \
    $$PWD/hardware \
    $$PWD/planner \
    $$PWD/ui \
    $$LEADSHINE_DIR
LIBS += -L$$LEADSHINE_DIR -lLTDMC

exists($$LEADSHINE_DIR/LTDMC.h) {
    message(Leadshine header found: $$LEADSHINE_DIR/LTDMC.h)
} else {
    warning(Leadshine header missing: $$LEADSHINE_DIR/LTDMC.h)
}

exists($$LEADSHINE_DIR/LTDMC.lib) {
    message(Leadshine lib found: $$LEADSHINE_DIR/LTDMC.lib)
} else {
    warning(Leadshine lib missing: $$LEADSHINE_DIR/LTDMC.lib)
}

LTDMC_DLL_WIN = $$replace(LEADSHINE_DIR, /, \\\\)\\\\LTDMC.dll
PVT_DLL_WIN = $$replace(LEADSHINE_DIR, /, \\\\)\\\\PVT.dll
OUT_PWD_WIN = $$replace(OUT_PWD, /, \\\\)

win32:CONFIG(debug, debug|release) {
    QMAKE_POST_LINK += cmd /c "if not exist \"$$OUT_PWD_WIN\\\\debug\" mkdir \"$$OUT_PWD_WIN\\\\debug\" && copy /y \"$$LTDMC_DLL_WIN\" \"$$OUT_PWD_WIN\\\\debug\\\\LTDMC.dll\" > nul && copy /y \"$$PVT_DLL_WIN\" \"$$OUT_PWD_WIN\\\\debug\\\\PVT.dll\" > nul"
}

win32:CONFIG(release, debug|release) {
    QMAKE_POST_LINK += cmd /c "if not exist \"$$OUT_PWD_WIN\\\\release\" mkdir \"$$OUT_PWD_WIN\\\\release\" && copy /y \"$$LTDMC_DLL_WIN\" \"$$OUT_PWD_WIN\\\\release\\\\LTDMC.dll\" > nul && copy /y \"$$PVT_DLL_WIN\" \"$$OUT_PWD_WIN\\\\release\\\\PVT.dll\" > nul"
}
