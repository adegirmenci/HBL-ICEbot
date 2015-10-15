#ifndef OMNITHREAD_H
#define OMNITHREAD_H

#include <QThread>
#include <QTimer>
#include <QElapsedTimer>

#include <stdio.h>
#include <assert.h>
#include <windows.h>
#include <conio.h>

//#include <HD/hd.h>
//#include <HD/hdDefines.h>
//#include <HD/hdScheduler.h>
//#include <HDU/hduError.h>
//#include <HDU/hduVector.h>
#include <hd.h>
#include <hdDefines.h>
#include <hdScheduler.h>
#include <hduError.h>
#include <hduVector.h>

/* Holds data retrieved from HDAPI. */
typedef struct
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    HDdouble m_transform[16]; // Transformation matrix
    HDErrorInfo m_error;
    hduVector3Dd m_deviceRotation;
    hduVector3Dd m_stylusHeading;
} DeviceData;

class omniThread : public QThread
{
    Q_OBJECT
public:
    explicit omniThread(QObject *parent = 0);
    
    DeviceData m_currentData;
    DeviceData m_prevData;
    DeviceData gServoDeviceData;

    void open();
    void init();
    void close();

    void getDevicePosRot(hduVector3Dd &pos, hduVector3Dd &rot);
    double getSphereRad() {return m_sphereRadius;}
    void setSphereRad(double radius);

signals:
    void button1pressed(bool pressed);

public slots:
    void run();

private:
    HDSchedulerHandle m_hUpdateHandle;
    HHD m_hHD;

    double m_sphereRadius;

    FILE *pFile;

    //QTimer *updateTimer;

};

HDCallbackCode HDCALLBACK updateDeviceCallback(void *ptr);
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *ptr);

#endif // OMNITHREAD_H
