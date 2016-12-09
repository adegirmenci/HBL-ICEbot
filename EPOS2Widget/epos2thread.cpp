#include "epos2thread.h"

EPOS2Thread::EPOS2Thread(QObject *parent) : QObject(parent)
{
    qRegisterMetaType< std::vector<long> >("std::vector<long>");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepServoing = false;
    m_abort = false;

    m_KeyHandle = 0;

    m_mutex = new QMutex(QMutex::Recursive);

    qDebug() << "Initizializing pointers to eposMotor";

    for(size_t i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        m_motors.push_back(std::shared_ptr<eposMotor>(new eposMotor));
        m_motors[i]->m_nodeID = EPOS_MOTOR_IDS[i];

        m_motors[i]->m_minQC = EPOS_MOTOR_LIMITS[i][0]; // lower limit
        m_motors[i]->m_maxQC = EPOS_MOTOR_LIMITS[i][1]; // upper limit

        m_motors[i]->m_enabled = false;
    }

}

EPOS2Thread::~EPOS2Thread()
{
    disconnectEPOS();

    m_mutex->lock();
    m_abort = true;
    qDebug() << "Ending EPOS2Thread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
    m_mutex->unlock();

    delete m_mutex;

    emit finished();
}

bool EPOS2Thread::initializeEPOS()
{
    QMutexLocker locker(m_mutex);

    emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_INITIALIZE_BEGIN);
    emit statusChanged(EPOS_INITIALIZE_BEGIN);

    qDebug() << "Opening connection to EPOS...";

    if(m_KeyHandle)
    {
        //Close Previous Device
        if(!VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode))
            ShowErrorInformation(m_ulErrorCode);
        m_KeyHandle = 0;
    }
    else
        m_KeyHandle = 0;

    //Settings
    m_oImmediately = true; // don't wait until end of last move
    m_oUpdateActive = false;

    HANDLE hNewKeyHandle;

    hNewKeyHandle = VCS_OpenDeviceDlg(&m_ulErrorCode);
    if(hNewKeyHandle)
        m_KeyHandle = hNewKeyHandle;
    else
    {
        ShowErrorInformation(m_ulErrorCode);
        emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_INITIALIZE_FAILED);
        emit statusChanged(EPOS_INITIALIZE_FAILED);
        return false;
    }

    // Set properties of each motor
    bool status = true;

    //locker.unlock();
    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        if(!initializeMotor( EPOS_AXIS_IDS[i]) )
        {
            status = false;
            ShowErrorInformation(m_ulErrorCode);
        }
        else
            status = status && true;
    }
    //locker.relock();

    emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_INITIALIZED);
    emit statusChanged(EPOS_INITIALIZED);

    return status;
}

void EPOS2Thread::startServoing()
{
    QMutexLocker locker(m_mutex);

    m_timer = new QTimer(this);
    m_timer->start(0);
    connect(m_timer,SIGNAL(timeout()),this,SLOT(servoToPosition()));

    m_keepServoing = true;

//    QElapsedTimer elTimer;
//    elTimer.start();

//    QTime::currentTime();

//    qint64 elNsec = elTimer.nsecsElapsed();
//    qDebug() << "Nsec elapsed:" << elNsec;


    emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_SERVO_LOOP_STARTED);
    emit statusChanged(EPOS_SERVO_LOOP_STARTED);

    qDebug() << "Servo loop started.";
}

void EPOS2Thread::setServoTargetPos(const int axisID, long targetPos, bool moveAbsOrRel)
{
    QMutexLocker locker(m_mutex);

    // check axis limits
    if(checkMotorID(axisID))
    {
        if(moveAbsOrRel)
            targetPos = clampToMotorLimits(axisID, targetPos); // abs
        else
            targetPos = clampToMotorLimits(axisID, m_motors[axisID]->m_lActualValue + targetPos); // rel

        // compensate for roll
        if(ROLL_AXIS_ID == axisID)
        {
            // #relativeRoll = #currentIncrement - #oldPosition
            long relativeRoll = targetPos - m_motors[ROLL_AXIS_ID]->m_lActualValue;

            // this works because roll is the last to update
            // if roll is not the last to get updated, then these target values
            // would get overwritten
            m_motors[PITCH_AXIS_ID]->m_lTargetPosition += relativeRoll; //pitch
            m_motors[YAW_AXIS_ID]->m_lTargetPosition   -= relativeRoll; //yaw

            //update limits
//            m_motors[PITCH_AXIS_ID]->m_minQC += relativeRoll;
//            m_motors[PITCH_AXIS_ID]->m_maxQC += relativeRoll;
//            m_motors[YAW_AXIS_ID]->m_minQC -= relativeRoll;
//            m_motors[YAW_AXIS_ID]->m_maxQC -= relativeRoll;
            m_motors[PITCH_AXIS_ID]->m_minQC = EPOS_PITCH_MIN + targetPos;
            m_motors[PITCH_AXIS_ID]->m_maxQC = EPOS_PITCH_MAX + targetPos;
            m_motors[YAW_AXIS_ID]->m_minQC = EPOS_YAW_MIN - targetPos;
            m_motors[YAW_AXIS_ID]->m_maxQC = EPOS_YAW_MAX - targetPos;

        }

        // finally, update the target
        m_motors[axisID]->m_lTargetPosition = targetPos;
    }
}

void EPOS2Thread::setServoTargetPos(std::vector<long> targetPos, bool moveAbsOrRel)
{
    QMutexLocker locker(m_mutex);

    if(targetPos.size() != EPOS_NUM_MOTORS)
        qDebug() << "Wrong vector size in setServoTargetPos.";

    //updateMotorQCs();

    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
//        if(moveAbsOrRel)
//            m_motors[i]->m_lTargetPosition = targetPos[i];
//        else
//            m_motors[i]->m_lTargetPosition += targetPos[i];
        setServoTargetPos(i, targetPos[i], moveAbsOrRel);
    }

    //qDebug() << "New servo pos received.";
}

int EPOS2Thread::checkMotorLimits(const int axisID, const long targetPos)
{
    QMutexLocker locker(m_mutex);

    if(targetPos < m_motors[axisID]->m_minQC)      // too small
        return -1;
    else if(targetPos > m_motors[axisID]->m_maxQC) // too large
        return 1;
    else                                           // just right
        return 0;
}

long EPOS2Thread::clampToMotorLimits(const int axisID, const long targetPos)
{
    QMutexLocker locker(m_mutex);

    if(targetPos < m_motors[axisID]->m_minQC)      // too small
        return m_motors[axisID]->m_minQC;
    else if(targetPos > m_motors[axisID]->m_maxQC) // too large
        return m_motors[axisID]->m_maxQC;
    else                                           // just right
        return targetPos;
}


// Calls to the motors will ALWAYS be executed SEQUENTIALLY over a single USB line, because data can only travel sequentially
// over a serial port.
// If we need to REDUCE the LATENCY, then we should consider using two USB cables.
// If the main concern is SYNCING the MOTION COMMANDS, then we can have the master (root) EPOS send a trigger signal to the
// slave boards AFTER all the motor target positions have been updated.
void EPOS2Thread::servoToPosition()
{
    QMutexLocker locker(m_mutex);

    updateMotorQCs();

//  QElapsedTimer elTimer;
//  qDebug() << "Using clock type " << elTimer.clockType();
//  elTimer.start();

    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        servoToPosition(i);
    }

//  qDebug() << "Elapsed Time: " << elTimer.nsecsElapsed()/1000000. << " ms";
}

void EPOS2Thread::servoToPosition(const int axisID)
{
    QMutexLocker locker(m_mutex);

    if(checkMotorID(axisID))
    {
        WORD usNodeId = m_motors[axisID]->m_nodeID;

        if(m_motors[axisID]->m_enabled)
        {
            //updateMotorQC(axisID); // this may not be the best place to put this

            emit logData(QTime::currentTime(), EPOS_COMMANDED, axisID, m_motors[axisID]->m_lTargetPosition);

            if(!VCS_MoveToPosition(m_KeyHandle, usNodeId, m_motors[axisID]->m_lTargetPosition, true, m_oImmediately, &m_ulErrorCode))
            {
                ShowErrorInformation(m_ulErrorCode);
                qDebug() << QString("Servo to pos failed! Axis %1, Node %2").arg(axisID).arg(usNodeId);

                emit statusChanged(EPOS_SERVO_TO_POS_FAILED);
            }
        }
    }
}

void EPOS2Thread::stopServoing()
{
    QMutexLocker locker(m_mutex);

    if ( m_keepServoing )
    {
        m_keepServoing = false;
        m_timer->stop();

        disconnect(m_timer,SIGNAL(timeout()),0,0);

        delete m_timer;

        emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_SERVO_LOOP_STOPPED);
        emit statusChanged(EPOS_SERVO_LOOP_STOPPED);
        qDebug() << "Servo loop stopped.";
    }
}

void EPOS2Thread::updateMotorQC(const int axisID)
{
    QMutexLocker locker(m_mutex);

    if(checkMotorID(axisID))
    {
        //Read Actual Position
        if(VCS_GetPositionIs(m_KeyHandle, m_motors[axisID]->m_nodeID,
                             &(m_motors[axisID]->m_lActualValue),
                             &m_ulErrorCode))
        {
            emit logData(QTime::currentTime(), EPOS_READ, axisID, m_motors[axisID]->m_lActualValue);
            emit motorQcChanged(axisID, m_motors[axisID]->m_lActualValue);
        }
        else
        {
            ShowErrorInformation(m_ulErrorCode);

            emit statusChanged(EPOS_UPDATE_QC_FAILED);
        }
    }
}

void EPOS2Thread::updateMotorQCs()
{
    QMutexLocker locker(m_mutex);

    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        if(m_motors[i]->m_enabled)
           updateMotorQC(i);
    }
}

void EPOS2Thread::haltMotor(const int axisID)
{
    QMutexLocker locker(m_mutex);

    if(checkMotorID(axisID))
    {

        if(VCS_HaltPositionMovement(m_KeyHandle, m_motors[axisID]->m_nodeID, &m_ulErrorCode))
        {
            //Read Actual Position
            updateMotorQC(axisID);
        }
        else
        {
            ShowErrorInformation(m_ulErrorCode);
            qDebug() << QString("Halt failed! Node %1").arg(m_motors[axisID]->m_nodeID);

            emit statusChanged(EPOS_HALT_FAILED);
        }
    }
}


void EPOS2Thread::homeAxis(const int axisID)
{
    QMutexLocker locker(m_mutex);

    if(checkMotorID(axisID))
    {
        // set a lower speed
        if(m_motors[axisID]->m_enabled)
        {
            WORD nodeID = m_motors[axisID]->m_nodeID;
            if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                       1400,2000,2000,&m_ulErrorCode))
            {
                ShowErrorInformation(m_ulErrorCode);
            }

            m_motors[axisID]->m_minQC = EPOS_MOTOR_LIMITS[axisID][0]; // reset lower limit
            m_motors[axisID]->m_maxQC = EPOS_MOTOR_LIMITS[axisID][1]; // reset upper limit

            setServoTargetPos(axisID, 0, true); // set target position to 0

            // TODO: this should be a blocking call, otherwise speed gets reset during motion
            servoToPosition(axisID); // servo motor

            // reset speed
            if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                       EPOS_VELOCITY[axisID],EPOS_ACCEL[axisID],EPOS_DECEL[axisID],&m_ulErrorCode))
            {
                ShowErrorInformation(m_ulErrorCode);
            }
        }
    }
}

void EPOS2Thread::homeAllAxes()
{
    QMutexLocker locker(m_mutex);

    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        if(m_motors[i]->m_enabled)
        {
            // set a lower speed
            WORD nodeID = m_motors[i]->m_nodeID;
            if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                       1400,2000,2000,&m_ulErrorCode))
            {
                ShowErrorInformation(m_ulErrorCode);
            }

            m_motors[i]->m_minQC = EPOS_MOTOR_LIMITS[i][0]; // reset lower limit
            m_motors[i]->m_maxQC = EPOS_MOTOR_LIMITS[i][1]; // reset upper limit

            // don't use setServoPos here, since that will compensate for roll
            // self comment: setServoPos(vector) would probably work though?
            m_motors[i]->m_lTargetPosition = 0;

            // TODO: a better strategy is probably to get the catheter straight first, then home
        }
    }

    // TODO: this should be a blocking call, otherwise speed gets reset during motion
    servoToPosition(); // servo motors

    // reset speed
    for(int i = 0; i < EPOS_NUM_MOTORS; i++)
    {
        if(m_motors[i]->m_enabled)
        {
            WORD nodeID = m_motors[i]->m_nodeID;
            if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                       EPOS_VELOCITY[i],EPOS_ACCEL[i],EPOS_DECEL[i],&m_ulErrorCode))
            {
                ShowErrorInformation(m_ulErrorCode);
            }
         }
    }
}

bool EPOS2Thread::disconnectEPOS()
{
    bool status = true;

    QMutexLocker locker(m_mutex);

    stopServoing();

    if(m_KeyHandle)
    {
        //locker.unlock();
        for(int i = 0; i < EPOS_NUM_MOTORS; i++)
        {
            status = status && disableMotor( EPOS_AXIS_IDS[i] );
        }
        //locker.relock();
        if(status)
            qDebug() << "Motors disabled.";

        status = VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
        if(status)
        {
            m_KeyHandle = 0;

            qDebug() << "EPOS closed successfully.";

            emit statusChanged(EPOS_DISCONNECTED);
        }
        else
        {
            ShowErrorInformation(m_ulErrorCode);
            emit statusChanged(EPOS_DISCONNECT_FAILED);
        }
    }
    else
    {
        qDebug() << ("EPOS is already closed.");
    }

    return status;
}

void EPOS2Thread::setEpoch(const QDateTime &datetime)
{
    QMutexLocker locker(m_mutex);

    if(!m_keepServoing)
    {
        m_epoch = datetime;
        m_isEpochSet = true;

        emit logEventWithMessage(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_EPOCH_SET,
                                 m_epoch.toString("yyyy/MM/dd - hh:mm:ss.zzz"));
    }
    else
        emit logEvent(SRC_EPOS, LOG_INFO, QTime::currentTime(), EPOS_EPOCH_SET_FAILED);
}

// ----------------
// HELPER FUNCTIONS
// ----------------

bool EPOS2Thread::initializeMotor(const int motID)
{
    QMutexLocker locker(m_mutex);

    bool result = checkMotorID(motID);

    if(result && !m_motors[motID]->m_enabled)
    {
        std::shared_ptr<eposMotor> mot = m_motors[motID];

        mot->m_bMode = 0;
        mot->m_lActualValue = 0;
        mot->m_lStartPosition = 0;
        mot->m_lTargetPosition = 0;

        WORD usNodeID = mot->m_nodeID;

        qDebug() << QString("Connecting to Node %1...").arg(usNodeID);

        mot->m_enabled = false;

        if(m_KeyHandle)
        {
            //Clear Error History
            if(VCS_ClearFault(m_KeyHandle, usNodeID, &m_ulErrorCode))
            {
                //Enable
                if( VCS_SetEnableState(m_KeyHandle, usNodeID, &m_ulErrorCode) )
                {
                    //Read Operation Mode
                    if(VCS_GetOperationMode(m_KeyHandle, usNodeID,
                                            &(mot->m_bMode),
                                            &m_ulErrorCode))
                    {
                        //Read Position Profile Objects
                        if(VCS_GetPositionProfile(m_KeyHandle, usNodeID,
                                                  &(mot->m_ulProfileVelocity),
                                                  &(mot->m_ulProfileAcceleration),
                                                  &(mot->m_ulProfileDeceleration),
                                                  &m_ulErrorCode))
                        {
                            //Write Profile Position Mode
                            if(VCS_SetOperationMode(m_KeyHandle, usNodeID,
                                                    OMD_PROFILE_POSITION_MODE,
                                                    &m_ulErrorCode))
                            {
                                //Write Profile Position Objects
                                if(VCS_SetPositionProfile(m_KeyHandle, usNodeID,
                                                          EPOS_VELOCITY[motID],
                                                          EPOS_ACCEL[motID],
                                                          EPOS_DECEL[motID],
                                                          &m_ulErrorCode))
                                {
                                    mot->m_ulProfileVelocity = EPOS_VELOCITY[motID];
                                    mot->m_ulProfileAcceleration = EPOS_ACCEL[motID];
                                    mot->m_ulProfileDeceleration = EPOS_DECEL[motID];

                                    //Read Actual Position
                                    if(VCS_GetPositionIs(m_KeyHandle, usNodeID,
                                                         &(mot->m_lStartPosition),
                                                         &m_ulErrorCode))
                                    {
                                        qDebug() << "DONE!";

                                        mot->m_lActualValue = mot->m_lStartPosition;
                                        mot->m_lTargetPosition = mot->m_lStartPosition;

                                        mot->m_enabled = true;
                                        mot->m_motorStatus = EPOS_MOTOR_ENABLED;
                                        result = true;
                                        emit motorStatusChanged(motID, EPOS_MOTOR_ENABLED);
                                        emit motorQcChanged(motID, mot->m_lActualValue);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(!mot->m_enabled)
            {
                qDebug() << "Can't connect to motor!";
                ShowErrorInformation(m_ulErrorCode);
                mot->m_motorStatus = EPOS_MOTOR_CANT_CONNECT;
                emit motorStatusChanged(motID, EPOS_MOTOR_CANT_CONNECT);
                result = false;
            }
        }
        else
        {
            qDebug() << "Can't open device!";
            mot->m_enabled = false;
            mot->m_motorStatus = EPOS_MOTOR_CANT_CONNECT;
            result = false;
            emit statusChanged(EPOS_DEVICE_CANT_CONNECT);
        }
    }

    return result;
}

bool EPOS2Thread::disableMotor(const int motID)
{
    QMutexLocker locker(m_mutex);

    bool result = checkMotorID(motID);

    if(result && m_motors[motID]->m_enabled)
    {
        std::shared_ptr<eposMotor> mot = m_motors[motID];

        WORD usNodeID = mot->m_nodeID; //get motor ID
        mot->m_enabled = false; // set flag to false
        mot->m_motorStatus = EPOS_MOTOR_DISABLED;

        // disable motor
        result = VCS_SetDisableState(m_KeyHandle, usNodeID, &m_ulErrorCode);
        if(result)
        {
            qDebug() <<"Motor " << usNodeID << " disabled.";
            emit motorStatusChanged(motID, EPOS_MOTOR_DISABLED);
        }
        else
            ShowErrorInformation(m_ulErrorCode);
    }

    return result;
}

const int EPOS2Thread::getMotorStatus(const int axisID)
{
    QMutexLocker locker(m_mutex);

    bool result = checkMotorID(axisID);

    if(result)
        return m_motors[axisID]->m_motorStatus;
    else
        return EPOS_INVALID_MOTOR_ID;
}

bool EPOS2Thread::ShowErrorInformation(DWORD p_ulErrorCode)
{
    char* pStrErrorInfo;
    const char* strDescription;
    QString msg(QString("Error Code %1: ").arg(QString::number(p_ulErrorCode)));

    if((pStrErrorInfo = (char*)malloc(100)) == NULL)
    {
        msg.append("Not enough memory to allocate buffer for error information string.");
        qDebug() << "Maxon: " << msg;

        return false;
    }

    if(VCS_GetErrorInfo(p_ulErrorCode, pStrErrorInfo, 100))
    {
        strDescription = pStrErrorInfo;
        msg.append(strDescription);

        qDebug() << "Maxon: " << msg;
        emit logError(SRC_EPOS, LOG_ERROR, QTime::currentTime(), EPOS_FAIL, msg);

        free(pStrErrorInfo);

        return true;
    }
    else
    {
        msg.append("Error information can't be read!");

        free(pStrErrorInfo);

        qDebug() << "Maxon: " << msg;
        emit logError(SRC_EPOS, LOG_ERROR, QTime::currentTime(), EPOS_FAIL, msg);

        return false;
    }
}

bool EPOS2Thread::checkMotorID(const int motID)
{
    if( (motID >= EPOS_NUM_MOTORS) || (motID < 0) )
    {
        QString msg = QString("Motor ID (%1) out of bounds").arg(motID);

        emit logEventWithMessage(SRC_EPOS, LOG_ERROR,
                      QTime::currentTime(),
                      EPOS_DISABLE_MOTOR_FAILED,
                      msg);
        qDebug() << msg;
        return false;
    }
    return true;
}

QString EPOS2Thread::formatOutput(QTime &timeStamp,
                     const int motorID,
                     long pos)
{
    QString output;
    output.append(timeStamp.toString("HH.mm.ss.zzz\n"));
    output.append(QString("[Motor %1] - %2 QCs").arg(motorID).arg(pos));

    return QString();
}

inline const QString getCurrTimeStr()
{
    return QTime::currentTime().toString("HH.mm.ss.zzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("yyyy/MM/dd - hh:mm:ss.zzz");
}
