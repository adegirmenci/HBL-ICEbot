#ifndef LABJACK_H
#define LABJACK_H

#include <QWidget>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include "LabJackUD.h"

// ------------------------------------------- \\
// -------------- LabJackWorker -------------- \\
// ------------------------------------------- \\

class LabJackWorker : public QObject
{
    Q_OBJECT

public:
    LabJackWorker();
    LabJackWorker(int samplesPsec);
    ~LabJackWorker();

    bool ConnectToLabJack(); // helper function to connect to LabJack
    bool SetFileName(const QString &fileName);
    bool ConfigureStream();
    bool StartStream();
    bool StopStream();
    bool DisconnectFromLabJack(); // helper function to disconnect from LabJack
    void setEpoch(const QTime &epoch) { m_epoch = epoch; }

    bool workerIsReady() { return m_LabJackReady; }
    bool workerIsRecording() { return m_eventTimer->isActive(); }

private slots:
    bool ReadStream();

private:
    QTimer *m_eventTimer; // timer for event loop
    QFile m_outputFile; // output file
    QTextStream m_textStream;
    int m_timeCurr;
    QTime m_epoch; // when was the GUI started - can be set externally

    // Flag to keep track of connection
    bool m_LabJackReady;

    // LabJack specific variables
    LJ_ERROR m_lngErrorcode;
    LJ_HANDLE m_lngHandle;
    long m_lngGetNextIteration;
    unsigned long long int m_i, m_k;
    long m_lngIOType, m_lngChannel;
    double m_dblValue, m_dblCommBacklog, m_dblUDBacklog;
    double m_scanRate; //scan rate = sample rate / #channels
    double m_numScans; //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    double m_numScansRequested;
    double *m_adblData;
    long m_padblData;
    // LabJack specific variables \\

    bool ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration);
};

// ------------------------------------- \\
// -------------- LabJack -------------- \\
// ------------------------------------- \\

namespace Ui {
class LabJack;
}

class LabJack : public QWidget
{
    Q_OBJECT
    QThread* workerThread;

public:
    explicit LabJack(QWidget *parent = 0);
    ~LabJack();

    bool isReady();
    bool isRecording();

    bool setWorkerEpoch(const QTime &epoch);
    bool setWorkerFileName(const QString &fileName);

public slots:

signals:

private slots:
    void on_connectButton_clicked();

    void on_disconnectButton_clicked();

    void on_initializeLJbutton_clicked();

    void on_startRecordButton_clicked();

    void on_stopRecordButton_clicked();

private:
    Ui::LabJack *ui;

    LabJackWorker* worker;

    bool ConfigureWorker(int samplesPsec);

};

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();

#endif // LABJACK_H
