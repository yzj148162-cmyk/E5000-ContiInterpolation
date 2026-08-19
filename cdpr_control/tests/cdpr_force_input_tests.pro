QT += core
CONFIG += console c++17
CONFIG -= app_bundle

TEMPLATE = app
TARGET = cdpr_force_input_tests

INCLUDEPATH += $$PWD/..

SOURCES += \
    CdprForceInputTests.cpp \
    ../cdpr/CdprForceInput.cpp

HEADERS += \
    ../cdpr/CdprForceInput.h \
    ../cdpr/CdprControlTypes.h \
    ../cdpr/CdprConfiguration.h
