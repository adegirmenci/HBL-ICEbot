#include "omnithread.h"

omniThread::omniThread(QObject *parent) :
    QThread(parent)
{
//    moveToThread(this);
    m_hUpdateHandle = 0;
    HDErrorInfo error;

    /* Initialize the device, must be done before attempting to call any hd
       functions. */
    m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        //throw exception
    }

    m_sphereRadius = 40.0;

    /* Schedule the main scheduler callback that updates the device state. */
    m_hUpdateHandle = hdScheduleAsynchronous(updateDeviceCallback, this, HD_MAX_SCHEDULER_PRIORITY);

    open();
    init();

//    updateTimer = new QTimer();
//    updateTimer->setInterval(1);
//    connect(updateTimer, SIGNAL(timeout()), this, SLOT(run()));
//    updateTimer->start();
}

void omniThread::getDevicePosRot(hduVector3Dd &pos, hduVector3Dd &rot)
{
    pos = m_currentData.m_devicePosition;
    //rot = m_currentData.m_deviceRotation;
    rot = m_currentData.m_stylusHeading;
}

void omniThread::open()
{
    HDErrorInfo error;
    /* Start the servo loop scheduler. */
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        //throw exception
    }

    if(fopen_s(&pFile, "omniPositionRec.txt","w") == 0)
        fprintf(stdout, "Opened log file.\n");
}

void omniThread::init()
{
    /* Perform a synchronous call to copy the most current device state. */
    hdScheduleSynchronous(copyDeviceDataCallback, this, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&m_prevData, &m_currentData, sizeof(DeviceData));
}

void omniThread::close()
{
    /* For cleanup, unschedule callbacks and stop the servo loop. */
    hdStopScheduler();
    hdUnschedule(m_hUpdateHandle);
    hdDisableDevice(m_hHD);

    fclose(pFile);
}

void omniThread::setSphereRad(double radius)
{
    if((1.0 < radius) && (radius < 80.0))
        m_sphereRadius = radius;
}

void omniThread::run()
{
//    QElapsedTimer elTimer;
//    elTimer.start();
//    qint64 ctr = 0;
//    qint64 elapsedT = 0;
//    qint64 avgTime = 0;

    while(isRunning())
    {
        /* Perform a synchronous call to copy the most current device state.
           This synchronous scheduler call ensures that the device state
           is obtained in a thread-safe manner. */
        hdScheduleSynchronous(copyDeviceDataCallback,
                              this,
                              HD_MIN_SCHEDULER_PRIORITY);

//        // Filter noise
//        hduVector3Dd diffPos, diffRot;

//        for(int i = 0; i < 3; i++)
//        {
//            diffPos[i] = fabs(m_currentData.m_devicePosition[i] - m_prevData.m_devicePosition[i]);
//            diffRot[i] = fabs(m_currentData.m_deviceRotation[i] - m_prevData.m_deviceRotation[i]);
//            if(diffPos[i] < 0.001)
//                m_currentData.m_devicePosition[i] = m_prevData.m_devicePosition[i];
//            if(diffRot[i] < 0.001)
//                m_currentData.m_deviceRotation[i] = m_prevData.m_deviceRotation[i];
//        }
        /* If the user depresses the gimbal button, display the current
           location information. */
        if (m_currentData.m_buttonState && !m_prevData.m_buttonState)
        {
            /*fprintf(stdout, "Current position: (%g, %g, %g)\n",
                m_currentData.m_devicePosition[0],
                m_currentData.m_devicePosition[1],
                m_currentData.m_devicePosition[2]);*/
            //fprintf(stdout, "Roll, pitch, yaw: (%g, %g, %g)\n", roll_, pitch_, yaw_);
            //fprintf(stdout, "Send A3200 to: (%g, %g, %g)\n", axis01, axis02, axis00);

            //close gripper

            //*************** FIX THIS - need to use signals?
            //ui->outputText->append(QString("Current position: (%1, %2, %3)").arg(ui->xLCD->value()).arg(ui->yLCD->value()).arg(ui->zLCD->value()));
            emit button1pressed(true);
        }
        else if (m_currentData.m_buttonState && m_prevData.m_buttonState)
        {
            /* Button is held pressed down. */

            // Gripper remains closed

            // write other code
        }
        else if (!m_currentData.m_buttonState && m_prevData.m_buttonState)
        {
            /* Button released. */
            // Open gripper
            emit button1pressed(false);
        }

        /* Check if an error occurred. */
        if (HD_DEVICE_ERROR(m_currentData.m_error))
        {
            hduPrintError(stderr, &m_currentData.m_error, "Device error detected");

            if (hduIsSchedulerError(&m_currentData.m_error))
            {
                /* Quit, since communication with the device was disrupted. */
                fprintf(stderr, "Communication with the device was disrupted..\n");
            }
        }

        /* Store off the current data for the next loop. */
        memcpy(&m_prevData, &m_currentData, sizeof(DeviceData));

//        ctr++;
//        elapsedT += elTimer.restart();
//        avgTime = elapsedT;
//        if( (ctr%1000) == 0)
//            printf("Elapsed Omni time: %d msec in %d counts.\n", int(avgTime), int(ctr));

        fprintf(pFile,"%lf %lf %lf %lf %lf %lf\n",
                m_currentData.m_devicePosition[0],m_currentData.m_devicePosition[1],m_currentData.m_devicePosition[2],
                m_currentData.m_stylusHeading[0],m_currentData.m_stylusHeading[1],m_currentData.m_stylusHeading[2]);


    }
    exec();
}

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *ptr) //pUserData
{
    omniThread* omniPtr = (omniThread *) ptr;

    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);

    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    omniPtr->gServoDeviceData.m_buttonState =
        (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, omniPtr->gServoDeviceData.m_devicePosition);

    /* Get the transformation matrix (HD_CURRENT_TRANSFORM)
       We declare a vector of three doubles since hdGetDoublev returns
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omniPtr->gServoDeviceData.m_deviceRotation);
    hduVector3Dd temp = omniPtr->gServoDeviceData.m_deviceRotation;
    omniPtr->gServoDeviceData.m_deviceRotation[0] = temp[2];
    omniPtr->gServoDeviceData.m_deviceRotation[2] = temp[0];
    hdGetDoublev(HD_CURRENT_TRANSFORM, omniPtr->gServoDeviceData.m_transform);

//    // calculate Roll [=0], Pitch [=1], Yaw [=2]
//    omniPtr->gServoDeviceData.m_deviceRotation[0] =
//        atan2(omniPtr->gServoDeviceData.m_transform[4], omniPtr->gServoDeviceData.m_transform[0]);
//    omniPtr->gServoDeviceData.m_deviceRotation[2] =
//        atan2(-1.0*omniPtr->gServoDeviceData.m_transform[8],
//            sqrt(pow(omniPtr->gServoDeviceData.m_transform[9],2) + pow(omniPtr->gServoDeviceData.m_transform[10],2)) );
//    omniPtr->gServoDeviceData.m_deviceRotation[1] =
//        atan2(omniPtr->gServoDeviceData.m_transform[9], omniPtr->gServoDeviceData.m_transform[10]);

    // calculate Roll [=0], Pitch [=1], Yaw [=2]
    // omni stores transformation matix in a column fashion (0,1,2,3 first column)
    omniPtr->gServoDeviceData.m_stylusHeading[0] = -omniPtr->gServoDeviceData.m_transform[8];//2
    omniPtr->gServoDeviceData.m_stylusHeading[1] = -omniPtr->gServoDeviceData.m_transform[9];//6
    omniPtr->gServoDeviceData.m_stylusHeading[2] = -omniPtr->gServoDeviceData.m_transform[10];

    /* Also check the error state of HDAPI. */
    omniPtr->gServoDeviceData.m_error = hdGetError();

    const hduVector3Dd spherePosition(0,0,0);
    const double sphereStiffness = .25;

    // Find the distance between the device and the center of the
    // sphere.
    double distance = (omniPtr->gServoDeviceData.m_devicePosition - spherePosition).magnitude();

    // If the user is outside the sphere -- i.e. if the distance from the user to
    // the center of the sphere is greater than the sphere radius -- then the user
    // is penetrating the sphere and a force should be commanded to repel him
    // towards the center.
    if (distance > omniPtr->getSphereRad())
    {
        // Calculate the penetration distance.
        double penetrationDistance = distance - omniPtr->getSphereRad();

        // Create a unit vector in the direction of the force, this will always
        // be inward to the center of the sphere through the user's
        // position.
        hduVector3Dd forceDirection = (spherePosition - omniPtr->gServoDeviceData.m_devicePosition)/distance;

        // Use F=kx to create a force vector that is towards the center of
        // the sphere and proportional to the penetration distance, and scaled
        // by the object stiffness.
        // Hooke's law explicitly:
        double k = sphereStiffness;
        hduVector3Dd x = penetrationDistance*forceDirection;
        hduVector3Dd f = k*x;
        hdSetDoublev(HD_CURRENT_FORCE, f);
    }


    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *ptr) //pUserData
{
    omniThread* omniPtr = (omniThread *) ptr;

    DeviceData *pDeviceData = &(omniPtr->m_currentData); //(DeviceData *) pUserData;

    memcpy(pDeviceData, &(omniPtr->gServoDeviceData), sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}
