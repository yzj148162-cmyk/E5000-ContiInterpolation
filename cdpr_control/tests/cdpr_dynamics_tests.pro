QT += core
QT -= gui

CONFIG += console c++17
CONFIG -= app_bundle

TARGET = cdpr_dynamics_tests
TEMPLATE = app

INCLUDEPATH += $$PWD/..

SOURCES += \
    CdprDynamicsTests.cpp \
    ../cdpr/CdprDynamics.cpp \
    ../cdpr/CdprForceInput.cpp

HEADERS += \
    ../cdpr/CdprConfiguration.h \
    ../cdpr/CdprControlTypes.h \
    ../cdpr/CdprDynamics.h \
    ../cdpr/CdprForceInput.h
