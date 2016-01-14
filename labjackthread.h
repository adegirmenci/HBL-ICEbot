#ifndef LABJACKTHREAD_H
#define LABJACKTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <QSharedPointer>

#include <vector>
#include <memory>

#include "../AscensionWidget/icebot_definitions.h"

#include "LabJackUD.h"

class LabJackThread : public QObject
{
    Q_OBJECT
public:
    explicit LabJackThread(QObject *parent = 0);
    ~LabJackThread();

signals:
    void statusChanged(int status);
    //void EM_Ready(bool status); // tells the widget that the EM tracker is ready
    void logData(QTime timeStamp,
                 double data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID); // LABJACK_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID, // LABJACK_EVENT_IDS
                  QString &message);
    void logError(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  long errorCode, // LJ_ERROR
                  QString &message);
    //void sendDataToGUI(int sensorID, const QString &output);
    void finished(); // emit upon termination

public slots:
    void connectLabJack(); // open connection
    void initializeLabJack(const unsigned int samplesPerSec); // initialize settings
    void startAcquisition(); // start timer
    void stopAcquisition(); // stop timer
    void disconnectLabJack(); // disconnect
    void setEpoch(const QTime &epoch);

private slots:
    void ReadStream();

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling acquireData every xxx msecs
    QTimer *m_timer;

    // Epoch for time stamps
    // During initializeLabJack(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if LabJack is ready
    // True if InitializeLabJack was successful
    bool m_isReady;

    // Flag to tell that we are still acquiring data
    bool m_keepAcquiring;
    // Flag to abort actions (e.g. initialize, acquire, etc.)
    bool m_abort;

    // LabJack specific variables
    LJ_ERROR m_lngErrorcode;
    LJ_HANDLE m_lngHandle;
    long m_lngGetNextIteration;
    unsigned long long int m_i, m_k;
    long m_lngIOType, m_lngChannel;
    double m_dblValue, m_dblCommBacklog, m_dblUDBacklog;
    double m_scanRate; //scan rate = sample rate / #channels
    int m_delayms;
    double m_numScans; //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    double m_numScansRequested;
    double *m_adblData;
    long m_padblData;
    // LabJack specific variables \\

    bool ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration);

};

#endif // LABJACKTHREAD_H
