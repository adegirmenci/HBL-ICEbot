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
    epos2.cpp \
    omni.cpp \
    omnithread.cpp \
    Point.cpp \
    SharedPoint.cpp

HEADERS  += icebot_gui.h \
    epos2.h \
    omni.h \
    omnithread.h \
    Point.h \
    SharedPoint.h

FORMS    += icebot_gui.ui \
    epos2.ui \
    omni.ui


win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/lib/x64/Release/ -lhd

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include/HD
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include/HD

win32: LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/lib/x64/Release/ -lhdu

INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include/HDU
DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include/HDU


win32: LIBS += -L$$PWD/Maxon Libs/ -lEposCmd64

INCLUDEPATH += $$PWD/Maxon Libs
DEPENDPATH += $$PWD/Maxon Libs

win32: LIBS += -L$$PWD/../../../../../Program Files (x86)/LabJack/Drivers/64bit/ -lLabJackUD

INCLUDEPATH += $$PWD/../../../../../Program Files (x86)/LabJack/Drivers
DEPENDPATH += $$PWD/../../../../../Program Files (x86)/LabJack/Drivers
