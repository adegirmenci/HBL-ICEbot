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
    omnithread.cpp

HEADERS  += icebot_gui.h \
    epos2.h \
    omni.h \
    omnithread.h

FORMS    += icebot_gui.ui \
    epos2.ui \
    omni.ui
