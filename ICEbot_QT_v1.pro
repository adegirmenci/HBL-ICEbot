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
    frmgrab.cpp \
    ascensionem.cpp

HEADERS  += icebot_gui.h \
    Point.h \
    omni.h \
    omnithread.h \
    epos2.h \
    labjack.h \
    frmgrab.h \
    ascensionem.h

FORMS    += icebot_gui.ui \
    omni.ui \
    epos2.ui \
    labjack.ui \
    frmgrab.ui \
    ascensionem.ui

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

win32 {
    INCLUDEPATH += "C:\\opencv\\build\\include" \

    CONFIG(debug,debug|release) {
        LIBS += -L"C:\\opencv\\build\\x86\\vc12\\lib" \
            -lopencv_core2411d \
            -lopencv_highgui2411d \
            -lopencv_imgproc2411d \
            -lopencv_features2d2411d \
            -lopencv_calib3d2411d
    }

    CONFIG(release,debug|release) {
        LIBS += -L"C:\\opencv\\build\\x86\\vc12\\lib" \
            -lopencv_core2411 \
            -lopencv_highgui2411 \
            -lopencv_imgproc2411 \
            -lopencv_features2d2411 \
            -lopencv_calib3d2411
    }
}

win32: LIBS += -L$$PWD/epiphan/frmgrab/lib/win/Win32/ -lfrmgrab

INCLUDEPATH += $$PWD/epiphan/include
INCLUDEPATH += $$PWD/epiphan/frmgrab/include

#win32: LIBS += -L"C:\\epiphan_sdk_32800009\\epiphan\\frmgrab\\lib\\win\\Win32" \
#             -lfrmgrab

#INCLUDEPATH += "C:\\epiphan_sdk_32800009\\epiphan\\include"
#INCLUDEPATH += "C:\\epiphan_sdk_32800009\\epiphan\\\frmgrab\\include"

win32: LIBS += -L$$PWD/3DGAPI/ -lATC3DG

INCLUDEPATH += $$PWD/3DGAPI
DEPENDPATH += $$PWD/3DGAPI
