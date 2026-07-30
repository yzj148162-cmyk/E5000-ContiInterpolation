QT += core
QT -= gui

CONFIG += console c++17
CONFIG -= app_bundle

TARGET = cdpr_kinematics_tests
TEMPLATE = app

INCLUDEPATH += $$PWD/..

SOURCES += \
    CdprKinematicsTests.cpp \
    ../cdpr/CdprConfiguration.cpp \
    ../cdpr/CdprKinematics.cpp

HEADERS += \
    ../cdpr/CdprConfiguration.h \
    ../cdpr/CdprControlTypes.h \
    ../cdpr/CdprKinematics.h
