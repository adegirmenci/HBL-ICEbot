#-------------------------------------------------
#
# Project created by QtCreator 2015-09-18T14:15:50
#
#-------------------------------------------------

QT += 3dcore 3drender 3dinput 3dextras core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = ICEbot_QT_v1
TEMPLATE = app

CONFIG += c++11

#DEFINES += EIGEN_QT_SUPPORT
#DEFINES *= QT_USE_QSTRINGBUILDER

SOURCES += main.cpp\
        icebot_gui.cpp \
    OmniWidget/omni.cpp \
    OmniWidget/omnithread.cpp \
    AscensionWidget/ascensionthread.cpp \
    AscensionWidget/ascensionwidget.cpp \
    DataLoggerWidget/dataloggerthread.cpp \
    DataLoggerWidget/dataloggerwidget.cpp \
    EPOS2Widget/epos2thread.cpp \
    EPOS2Widget/epos2widget.cpp \
    FrmGrabWidget/frmgrabthread.cpp \
    FrmGrabWidget/frmgrabwidget.cpp \
    LabJackWidget/labjackthread.cpp \
    LabJackWidget/labjackwidget.cpp \
    qcustomplot.cpp \
    SceneVizWidget/extendedqt3dwindow.cpp \
    SceneVizWidget/scenemodifier.cpp \
    SceneVizWidget/scenevizwidget.cpp \
    SceneVizWidget/triadentity.cpp \
    FrameClientWidget/frameclientthread.cpp \
    FrameClientWidget/frameclientwidget.cpp \
    ControllerWidget/controllerthread.cpp \
    ControllerWidget/controllerwidget.cpp \
    ControllerWidget/kinematics_4dof.cpp \
    SceneVizWidget/usentity.cpp

HEADERS  += icebot_gui.h \
    OmniWidget/omni.h \
    OmniWidget/omnithread.h \
    AscensionWidget/ascensionthread.h \
    AscensionWidget/ascensionwidget.h \
    DataLoggerWidget/dataloggerthread.h \
    DataLoggerWidget/dataloggerwidget.h \
    EPOS2Widget/epos2thread.h \
    EPOS2Widget/epos2widget.h \
    FrmGrabWidget/frmgrabthread.h \
    FrmGrabWidget/frmgrabwidget.h \
    LabJackWidget/labjackthread.h \
    LabJackWidget/labjackwidget.h \
    icebot_definitions.h \
    qcustomplot.h \
    SceneVizWidget/extendedqt3dwindow.h \
    SceneVizWidget/scenemodifier.h \
    SceneVizWidget/scenevizwidget.h \
    SceneVizWidget/triadentity.h \
    FrameClientWidget/frameclientthread.h \
    FrameClientWidget/frameclientwidget.h \
    ControllerWidget/controllerthread.h \
    ControllerWidget/controllerwidget.h \
    ControllerWidget/kinematics_4dof.h \
    SceneVizWidget/usentity.h

FORMS    += icebot_gui.ui \
    OmniWidget/omni.ui \
    AscensionWidget/ascensionwidget.ui \
    DataLoggerWidget/dataloggerwidget.ui \
    EPOS2Widget/epos2widget.ui \
    FrmGrabWidget/frmgrabwidget.ui \
    LabJackWidget/labjackwidget.ui \
    SceneVizWidget/scenevizwidget.ui \
    FrameClientWidget/frameclientwidget.ui \
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

win32: LIBS += -L$$PWD/LabJackWidget/LabJackLibs/ -lLabJackUD

INCLUDEPATH += $$PWD/LabJackWidget/LabJackLibs
DEPENDPATH += $$PWD/LabJackWidget/LabJackLibs


win32: LIBS += -L$$PWD/EPOS2Widget/MaxonLibs/ -lEposCmd

INCLUDEPATH += $$PWD/EPOS2Widget/MaxonLibs
DEPENDPATH += $$PWD/EPOS2Widget/MaxonLibs

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

win32: LIBS += -L$$PWD/AscensionWidget/3DGAPI/ -lATC3DG

INCLUDEPATH += $$PWD/AscensionWidget/3DGAPI
DEPENDPATH += $$PWD/AscensionWidget/3DGAPI

## Eigen
#INCLUDEPATH += "D:\\Eigen\\Eigen"
#INCLUDEPATH += "D:\\Eigen"
#DEPENDPATH += "D:\\Eigen\\Eigen"
