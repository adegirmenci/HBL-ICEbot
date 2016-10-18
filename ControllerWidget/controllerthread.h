#ifndef CONTROLLERTHREAD_H
#define CONTROLLERTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
#include <QSharedPointer>
#include <QAtomicInt>
#include <atomic>

#include <vector>
#include <memory>

//#include <chrono>

// included in kinematics_4dof.h
//#include <Eigen/Dense>
//#include <Eigen/SVD>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include <iostream>
//#include <fstream>
//#include <iterator>

#include "kinematics_4dof.h"

#include "../AscensionWidget/icebot_definitions.h"
#include "../AscensionWidget/3DGAPI/ATC3DG.h"

class ControllerThread : public QObject
{
    Q_OBJECT
public:
    explicit ControllerThread(QObject *parent = 0);
    ~ControllerThread();

signals:
    void statusChanged(int event);
    void logData(QTime timeStamp,
                 int frameIdx,
                 QString &data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID); // FRMGRAB_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID, // FRMGRAB_EVENT_IDS
                  QString &message);
    void logError(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int errCode, // FRMGRAB_ERROR_CODES
                  QString message);
    void finished(); // emit upon termination

public slots:
    void setEpoch(const QDateTime &epoch);
    void printThreadID();
    void receiveEMdata(QTime timeStamp,
                      int sensorID,
                      DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD data);

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling controlLoop every xxx msecs
    QTimer *m_timer;

    // Epoch for time stamps
    // During InitializeController(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if Controller is ready
    // True if InitializeController was successful
    bool m_isReady;

    // Flag to tell that we are still controlling
    bool m_keepControlling;
    // Flag to abort actions
    bool m_abort;

    const int m_prec = 4; // precision for print operations

    // latest reading
    std::vector<DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD> m_latestReading;

    // transforms
    Eigen::Transform<double,3,Eigen::Affine> m_BB_Box,
                                             m_STm_BT,
                                             m_BT_CT,
                                             m_BB_CT_curTipPos,
                                             m_BB_SBm,
                                             m_BBfixed_BBmobile,
                                             m_BBmobile_CT,
                                             m_curTipPos,
                                             m_ISm_INSTR,
                                             m_basTipPos_mobile,
                                             m_Box_BBmobile,
                                             m_targetPos,
                                             m_BB_targetPos,
                                             m_currChest;

    // keep track of number of control cycles
    // an atomic variable alleviates the need to use mutexes during mutation
    std::atomic<unsigned int> m_numCycles;

    Kinematics_4DOF m_cathKin;

    std::shared_ptr< std::vector<double> > cycle_recalculate(std::vector<double> &inputs);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ----------------
// HELPER FUNCTIONS
// ----------------

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();
static void Transform_From_EMreading(DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD &input, Eigen::Transform<double,3,Eigen::Affine> &output);

#endif // CONTROLLERTHREAD_H
