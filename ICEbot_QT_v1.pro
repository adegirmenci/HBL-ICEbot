#-------------------------------------------------
#
# Project created by QtCreator 2015-09-18T14:15:50
#
#-------------------------------------------------

QT       += 3dcore 3drender 3dinput 3dextras core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = ICEbot_QT_v1
TEMPLATE = app

DEFINES += EIGEN_QT_SUPPORT

SOURCES += main.cpp\
        icebot_gui.cpp \
    omni.cpp \
    omnithread.cpp \
    ascensionthread.cpp \
    ascensionwidget.cpp \
    dataloggerthread.cpp \
    dataloggerwidget.cpp \
    epos2thread.cpp \
    epos2widget.cpp \
    frmgrabthread.cpp \
    frmgrabwidget.cpp \
    labjackthread.cpp \
    labjackwidget.cpp \
    qcustomplot.cpp \
    SceneVizWidget/extendedqt3dwindow.cpp \
    SceneVizWidget/scenemodifier.cpp \
    SceneVizWidget/scenevizwidget.cpp \
    SceneVizWidget/triadentity.cpp \
    frameclientthread.cpp \
    frameclientwidget.cpp \
    ControllerWidget/controllerthread.cpp \
    ControllerWidget/controllerwidget.cpp \
    ControllerWidget/kinematics_4dof.cpp \
    SceneVizWidget/usentity.cpp

HEADERS  += icebot_gui.h \
    omni.h \
    omnithread.h \
    ascensionthread.h \
    ascensionwidget.h \
    dataloggerthread.h \
    dataloggerwidget.h \
    epos2thread.h \
    epos2widget.h \
    frmgrabthread.h \
    frmgrabwidget.h \
    labjackthread.h \
    labjackwidget.h \
    icebot_definitions.h \
    qcustomplot.h \
    SceneVizWidget/extendedqt3dwindow.h \
    SceneVizWidget/scenemodifier.h \
    SceneVizWidget/scenevizwidget.h \
    SceneVizWidget/triadentity.h \
    frameclientthread.h \
    frameclientwidget.h \
    ControllerWidget/controllerthread.h \
    ControllerWidget/controllerwidget.h \
    ControllerWidget/kinematics_4dof.h \
    SceneVizWidget/usentity.h

FORMS    += icebot_gui.ui \
    omni.ui \
    ascensionwidget.ui \
    dataloggerwidget.ui \
    epos2widget.ui \
    frmgrabwidget.ui \
    labjackwidget.ui \
    SceneVizWidget/scenevizwidget.ui \
    frameclientwidget.ui \
    ControllerWidget/controllerwidget.ui

RC_FILE = ICEbotGUI.rc

# Eigen
INCLUDEPATH += "D:\\Eigen"

# Boost
INCLUDEPATH += "D:\\boost_1_61_0"

win32 {
    INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include
    DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/include

    INCLUDEPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include
    DEPENDPATH += $$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/include

    CONFIG(debug,debug|release) {
        LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/lib/Win32/Debug/ -lhd
        LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/lib/Win32/Debug/ -lhdu
    }

    CONFIG(release,debug|release) {
        LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/lib/Win32/Release/ -lhd
        LIBS += -L$$PWD/../../../../../OpenHaptics/Developer/3.4.0/utilities/lib/Win32/Release/ -lhdu
    }
}

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

#win32: LIBS += -L$$PWD/epiphan/frmgrab/lib/win/Win32/ -lfrmgrab

#INCLUDEPATH += $$PWD/epiphan/include
#INCLUDEPATH += $$PWD/epiphan/frmgrab/include

#win32: LIBS += -L"C:\\epiphan_sdk_32800009\\epiphan\\frmgrab\\lib\\win\\Win32" \
#             -lfrmgrab

#INCLUDEPATH += "C:\\epiphan_sdk_32800009\\epiphan\\include"
#INCLUDEPATH += "C:\\epiphan_sdk_32800009\\epiphan\\\frmgrab\\include"

win32: LIBS += -L$$PWD/3DGAPI/ -lATC3DG

INCLUDEPATH += $$PWD/3DGAPI
DEPENDPATH += $$PWD/3DGAPI

## Eigen
#INCLUDEPATH += "D:\\Eigen\\Eigen"
#INCLUDEPATH += "D:\\Eigen"
#DEPENDPATH += "D:\\Eigen\\Eigen"
