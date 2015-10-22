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
    epos2.cpp \
    labjack.cpp \
    frmgrab.cpp

HEADERS  += icebot_gui.h \
    Point.h \
    omni.h \
    omnithread.h \
    epos2.h \
    labjack.h \
    frmgrab.h

FORMS    += icebot_gui.ui \
    omni.ui \
    epos2.ui \
    labjack.ui \
    frmgrab.ui

win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/lib/Win32/Release/ -lhd

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include

win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/lib/Win32/Release/ -lhdu

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include


win32: LIBS += -L$$PWD/LabJackLibs/ -lLabJackUD

INCLUDEPATH += $$PWD/LabJackLibs
DEPENDPATH += $$PWD/LabJackLibs


win32: LIBS += -L$$PWD/MaxonLibs/ -lEposCmd

INCLUDEPATH += $$PWD/MaxonLibs
DEPENDPATH += $$PWD/MaxonLibs
