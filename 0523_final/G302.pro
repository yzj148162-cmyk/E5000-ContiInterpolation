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

SOURCES += \
    MatrixFun.cpp \
    NokovMinimalClient.cpp \
    third_party/alglib/alglibinternal.cpp \
    third_party/alglib/alglibmisc.cpp \
    third_party/alglib/ap.cpp \
    monitorthread.cpp \
    motortorquetestworker.cpp \
    cdprdynmodel.cpp \
    controlworker.cpp \
    forcesimulationmodel.cpp \
    hardwareinterface.cpp \
    infolabel.cpp \
    intbtn.cpp \
    kalmanhandler.cpp \
    linalg.cpp \
    main.cpp \
    mainwindow.cpp \
    motivelocalhandlerthread.cpp \
    nokovrigidbodyframe.cpp \
    optimization.cpp \
    pidhandler.cpp \
    pvtexecutionworker.cpp \
    positionsimulationmodel.cpp \
    simulationworker.cpp \
    solvers.cpp \
    trajectorygenerator.cpp \
    trajectoryplanner.cpp \
    qcustomplot.cpp \
    curvedrawer.cpp \
    barycenter.cpp \
    udpcommworker.cpp \
    safetymonitor.cpp \
    singlemotortrajectoryworker.cpp \



HEADERS += \
    third_party/lt_dmc/LTDMC.h \
    MatrixFun.h \
    NokovMinimalClient.h \
    third_party/alglib/alglibinternal.h \
    third_party/alglib/alglibmisc.h \
    third_party/alglib/ap.h \
    monitorthread.h \
    motortorquetestworker.h \
    cdprdynmodel.h \
    controlworker.h \
    forcesimulationmodel.h \
    hardwareinterface.h \
    infolabel.h \
    intbtn.h \
    kalmanhandler.h \
    linalg.h \
    macro.h \
    mainwindow.h \
    motivelocalhandlerthread.h \
    nokovrigidbodyframe.h \
    optimization.h \
    pidhandler.h \
    pvtexecutionworker.h \
    positionsimulationmodel.h \
    simulationworker.h \
    solvers.h \
    trajectorygenerator.h \
    trajectoryplanner.h \
    udppackettypes.h \
    udpcommworker.h \
    qcustomplot.h \
    curvedrawer.h \
    barycenter.h \
    safetymonitor.h \
    singlemotortrajectoryworker.h \
    udppackettypes.h \

FORMS += \
    mainwindow.ui

win32-msvc* {
    QMAKE_CXXFLAGS += /bigobj
}

DEFINES += USE_NOKOV
DEFINES += AE_NO_SSE2 AE_NO_AVX2 AE_NO_FMA

INCLUDEPATH += $$PWD/third_party/eigen
INCLUDEPATH += $$PWD/third_party/alglib
INCLUDEPATH += $$PWD/third_party/lt_dmc
INCLUDEPATH += $$PWD/third_party/nlopt/nlopt-2.4.2-dll64
INCLUDEPATH += $$PWD/third_party/Nokov/include

LIBS += -L$$PWD/third_party/nlopt/nlopt-2.4.2-dll64 -llibnlopt-0
LIBS += -L$$PWD/third_party/lt_dmc -lLTDMC
LIBS += -L$$PWD/third_party/Nokov/lib -lnokov_sdk
