#ifndef EPOS2THREAD_H
#define EPOS2THREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QDateTime>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
#include <QSharedPointer>

#include <vector>
#include <memory>

#include "../AscensionWidget/icebot_definitions.h"

#include "MaxonLibs/Definitions.h"
#include <stdio.h>
#include <Windows.h>

//Q_DECLARE_METATYPE(std::vector<long>)
//Q_DECLARE_METATYPE(std::vector<int>)

struct eposMotor
{
    __int8 m_bMode;
    WORD m_nodeID; // motor ID
    DWORD m_ulProfileAcceleration; // acceleration value
    DWORD m_ulProfileDeceleration; // deceleration value
    DWORD m_ulProfileVelocity; // velocity value
    bool m_enabled;
    EPOS_MOTOR_STATUS m_motorStatus;

    long m_lActualValue; // volatile?
    long m_lStartPosition; // volatile?
    long m_lTargetPosition; // volatile?

    long m_maxQC; // upper limit
    long m_minQC; // lower limit
};

class EPOS2Thread : public QObject
{
    Q_OBJECT
public:
    explicit EPOS2Thread(QObject *parent = 0);
    ~EPOS2Thread();

    bool isInServoLoop(){QMutexLocker locker(m_mutex); return m_keepServoing;}

signals:
    void statusChanged(int event);
    void motorStatusChanged(const int motID, const int status);
    void motorStatusChanged(std::vector<int> status);
    void motorQcChanged(const int motID, const long QC);
    void motorQcChanged(std::vector<long> QCs);
    //void EPOS_Ready(bool status); // tells the widget that the EPOS is ready
    void logData(QTime timeStamp,
                 int dataType, // EPOS_DATA_IDS
                 const int motID,
                 long data);
    void logData(QTime timeStamp,
                 int dataType, // EPOS_DATA_IDS
                 std::vector<long> data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPE
                  QTime timeStamp,
                  int eventID); // EPOS_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType, // LOG_TYPE
                  QTime timeStamp,
                  int eventID,  // EPOS_EVENT_IDS
                  QString &message);
    void logError(int source, // LOG_SOURCE
                  int logType, // LOG_TYPE
                  QTime timeStamp,
                  int errCode, // EPOS_ERROR_CODES
                  QString message);
    void sendDataToGUI(const int id, const QString &output);
    void finished(); // emit upon termination

public slots:
    bool initializeEPOS(); // open connection to EPOS
    void startServoing(); // start timer
    void setServoTargetPos(const int axisID, long targetPos, bool moveAbsOrRel); // update target for one motor
    void setServoTargetPos(std::vector<long> targetPos, bool moveAbsOrRel); // update target for all motors
    void servoToPosition(); // called by timer
    void servoToPosition(const int axisID);
    void haltMotor(const int axisID); // immediately stop all motors
    void stopServoing(); // stop timer
    bool disconnectEPOS(); // disconnect from EPOS
    void setEpoch(const QDateTime &datetime); // set Epoch
    void updateMotorQC(const int axisID);
    void updateMotorQCs();
    const int getMotorStatus(const int axisID);
    bool initializeMotor(const int motID);
    bool disableMotor(const int motID);
    void homeAllAxes();

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling servoToPosition every xxx msecs
    QTimer *m_timer;

    // Epoch for time stamps
    // During initializeEPOS(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if EPOS tracker is ready
    // True if InitializeEPOS was successful
    bool m_isReady;

    // Flag to tell that we are still servoing
    bool m_keepServoing;
    // Flag to abort actions (e.g. initialize, acquire, etc.)
    bool m_abort;

    // EPOS variables
    BOOL m_oImmediately;
    BOOL m_oInitialisation;
    BOOL m_oUpdateActive;
    DWORD m_ulErrorCode;
    HANDLE m_KeyHandle;

    // list of motors
    std::vector< std::shared_ptr<eposMotor> > m_motors;

    const int m_prec = 4; // precision for print operations

    // error handler
    bool ShowErrorInformation(DWORD p_ulErrorCode);

    QString formatOutput(QTime &timeStamp,
                         const int motorID,
                         long pos);

    // -----

    long checkMotorLimits(const int axisID, const long targetPos);
    bool checkMotorID(const int motID);

};

// ----------------
// HELPER FUNCTIONS
// ----------------

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();

#endif // EPOS2THREAD_H
