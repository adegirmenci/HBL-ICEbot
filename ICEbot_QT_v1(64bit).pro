#-------------------------------------------------
#
# Project created by QtCreator 2015-09-18T14:15:50
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ICEbot_QT_v1
TEMPLATE = app


SOURCES += main.cpp\
        icebot_gui.cpp \
    Point.cpp \
    omni.cpp \
    omnithread.cpp \
    epos2.cpp

HEADERS  += icebot_gui.h \
    Point.h \
    omni.h \
    omnithread.h \
    epos2.h

FORMS    += icebot_gui.ui \
    omni.ui \
    epos2.ui

win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/lib/x64/Release/ -lhd

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include

win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/lib/x64/Release/ -lhdu

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include


win32: LIBS += -L$$PWD/LabJackLibs/ -lLabJackUD

INCLUDEPATH += $$PWD/LabJackLibs/64bit
DEPENDPATH += $$PWD/LabJackLibs/64bit


win32: LIBS += -L$$PWD/MaxonLibs/ -lEposCmd64

INCLUDEPATH += $$PWD/MaxonLibs
DEPENDPATH += $$PWD/MaxonLibs
