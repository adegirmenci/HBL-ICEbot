#ifndef DATALOGGERTHREAD_H
#define DATALOGGERTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
//#include <QWaitCondition>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <vector>
#include <memory>

#include "../AscensionWidget/icebot_definitions.h"
#include "3DGAPI/ATC3DG.h"

//Q_DECLARE_METATYPE(DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD)

Q_DECLARE_METATYPE(std::vector<int>)

class DataLoggerThread : public QObject
{
    Q_OBJECT
public:
    explicit DataLoggerThread(QObject *parent = 0);
    ~DataLoggerThread();

signals:
    void statusChanged(int event);
    void fileStatusChanged(const unsigned short fileID, int status);

    void finished(); // emit upon termination

public slots:
    // Widget slots
    void setRootDirectory(QString &dir);
    void initializeDataLogger(std::vector<int> enableMask, std::vector<QString> fileNames);
    void startLogging();
    void stopLogging();
    void closeLogFile(const unsigned short fileID);
    void closeLogFiles();
    void logNote(QTime timeStamp, QString note);
    // EM slots
    void logEMdata(QTime timeStamp,
                 int sensorID,
                 DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD data);
//    void logEvent(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EM_EVENT_IDS eventID);
//    void logEventWithMessage(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EM_EVENT_IDS eventID,
//                  QString &message);
//    void logError(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EM_ERROR_CODES,
//                  QString &message);
//    void logData(QTime timeStamp,
//                 EPOS_DATA_IDS dataType,
//                 const int motID,
//                 long data);
//    void logData(QTime timeStamp,
//                 EPOS_DATA_IDS dataType,
//                 std::vector<long> data);
//    void logEvent(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EPOS_EVENT_IDS eventID);
//    void logEventWithMessage(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EPOS_EVENT_IDS eventID,
//                  QString &message);
//    void logError(LOG_TYPES logType,
//                  QTime timeStamp,
//                  EPOS_ERROR_CODES,
//                  QString &message);

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Epoch for time stamps
    // During initializeDataLogger(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if DataLogger is ready for data logging
    // True if InitializeDataLogger was successful
    bool m_isReady;

    // Flag to tell the run() loop to keep logging data
    bool m_keepLogging;

    // Root directory for creating files
    QString m_rootDirectory;

//    QFile m_EMfile;
//    QFile m_ECGfile;
//    QFile m_EPOSfile;
//    QFile m_FrameFile;
//    QFile m_LogFile;
//    QFile m_ErrorFile;
//    QFile m_NoteFile;

//    QTextStream m_textStream;

    const int m_prec = 4; // precision for print operations

    std::vector< std::shared_ptr<QFile> > m_files;
    std::vector< std::shared_ptr<QTextStream> > m_streams;


};

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();

#endif // DATALOGGERTHREAD_H
