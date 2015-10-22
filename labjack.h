#ifndef LABJACK_H
#define LABJACK_H

#include <QWidget>
#include <QFileDevice>
#include <QDebug>
#include <QThread>
#include "LabJackUD.h"

// ------------------------------------------- \\
// -------------- LabJackWorker -------------- \\
// ------------------------------------------- \\

class LabJackWorker : public QObject
{
    Q_OBJECT

public slots:
    bool configStream();
    void doWork(const QString &parameter);

    bool workerIsReady() { return m_LabJackReady; }
    bool workerIsRecording() { return m_keepRecording; }

signals:
    void resultReady(const QString &result);

public:
    LabJackWorker();
    LabJackWorker(int samplesPsec);
    ~LabJackWorker();

    LJ_ERROR m_lngErrorcode;
    LJ_HANDLE m_lngHandle;
    long m_lngGetNextIteration;
    unsigned long long int m_i, m_k;
    long m_lngIOType, m_lngChannel;
    double m_dblValue, m_dblCommBacklog, m_dblUDBacklog;
    double m_scanRate; //scan rate = sample rate / #channels
    //long m_delayms;
    double m_numScans; //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    double m_numScansRequested;
    double *m_adblData;
    long m_padblData;

    bool m_LabJackReady;

    double m_timeCurr;

    bool m_keepRecording;

    QFileDevice* m_outputFile;

private:
    bool ConnectToLabJack();

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

    void StartStream(const char* fileName, const double timeStamp);
    int ReadData();
    void StopStream();

    bool isReady();
    bool isRecording();

public slots:
    void handleResults(const QString &str);

signals:
    void operate(const QString &);

private slots:
    void on_connectButton_clicked();

private:
    Ui::LabJack *ui;

    LabJackWorker* worker;

    void ConfigureStream(int samplesPsec);

};

#endif // LABJACK_H
