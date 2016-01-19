#ifndef ASCENSIONTHREAD_H
#define ASCENSIONTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QWaitCondition>
#include <QString>
#include <QDateTime>
#include <QTimer>
#include <QDebug>

#include <vector>

#include "../AscensionWidget/icebot_definitions.h"

#include "3DGAPI/ATC3DG.h"

// ##############################
// # ATC3DG (Ascension) classes #
// ##############################
// System class
class CSystem
{
public:
    SYSTEM_CONFIGURATION m_config;
};
// Sensor class
class CSensor
{
public:
    SENSOR_CONFIGURATION m_config;
};
// Transmitter class
class CXmtr
{
public:
    TRANSMITTER_CONFIGURATION m_config;
};
// ------------------------------

Q_DECLARE_METATYPE(DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD)

// ##################################
// # Ascension Thread Class Members #
// ##################################
class AscensionThread : public QObject
{
    Q_OBJECT
public:
    explicit AscensionThread(QObject *parent = 0);
    ~AscensionThread();

    //bool isEMready();
    //bool isRecording();

    int getSampleRate();
    int getNumSensors();

signals:
    void statusChanged(int status);
    //void EM_Ready(bool status); // tells the widget that the EM tracker is ready
    void logData(QTime timeStamp,
                 int sensorID,
                 DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID); // EM_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType,  // LOG_TYPES
                  QTime timeStamp,
                  int eventID, // EM_EVENT_IDS
                  QString &message);
    void logError(int source,  // LOG_SOURCE
                  int logType,   // LOG_TYPES
                  QTime timeStamp,
                  int errCode, // EM_ERROR_CODES
                  QString message);
    void sendDataToGUI(int sensorID, const QString &output);
    void finished(); // emit upon termination

public slots:
    bool initializeEM(); // open connection to EM
    void startAcquisition(); // start timer
    void stopAcquisition(); // stop timer
    bool disconnectEM(); // disconnect from EM
    void setEpoch(const QDateTime &datetime); // set Epoch
    void setSampleRate(int freq); // set freq
    void getLatestReading(const int sensorID, DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD &dataContainer);
    void getLatestReadingsAll(std::vector<DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD> &dataContainer);

private slots:
    void getSample(); // called by timer

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling acquire data every 1/m_samplingFreq
    QTimer *m_timer;

    // Epoch for time stamps
    // During initializeEM(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if EM tracker is ready for data collection
    // True if InitializeEM was successful
    bool m_isReady;

    // Flag to tell the run() loop to keep acquiring readings from EM
    bool m_keepRecording;
    // Fllag to abort actions (e.g. initialize, acquire, etc.)
    bool m_abort;

    // EM tracker variables
    CSystem m_ATC3DG;
    CSensor *m_pSensor;
    CXmtr *m_pXmtr;
    int m_errorCode;
    int m_i;
    int m_numSensorsAttached;
    int m_sensorID;
    short m_id;
    int m_samplingFreq;
    int m_records;
    int m_numberBytes;
    // latest reading
    std::vector<DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD> m_latestReading;

    const int m_prec = 4; // precision for print operations

    // EM tracker error handler function
    void errorHandler_(int error);

    QString formatOutput(QTime &timeStamp,
                         int sensorID,
                         DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD &record);

};

// ----------------
// HELPER FUNCTIONS
// ----------------

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();

#endif // ASCENSIONTHREAD_H
