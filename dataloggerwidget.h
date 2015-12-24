#ifndef DATALOGGERWIDGET_H
#define DATALOGGERWIDGET_H

#include <QWidget>
#include <QThread>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

#include "dataloggerthread.h"

namespace Ui {
class DataLoggerWidget;
}

class DataLoggerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DataLoggerWidget(QWidget *parent = 0);
    ~DataLoggerWidget();

    DataLoggerThread *m_worker;

signals:
    void initializeDataLogger(std::vector<int> enableMask, std::vector<QString> fileNames);
    void closeLogFiles();
    void startLogging();
    void stopLogging();
    void logNote(QTime timeStamp, QString note);

private slots:
    void workerStatusChanged(int status);
    void workerFileStatusChanged(const unsigned short fileID, int status);

    void on_dataDirPushButton_clicked();

    void on_initFilesButton_clicked();

    void on_closeFilesButton_clicked();

    void on_startLoggingButton_clicked();

    void on_stopLoggingButton_clicked();

    void on_logNoteButton_clicked();

private:
    Ui::DataLoggerWidget *ui;

    QThread m_thread; // Data Logger Thread will live in here

    std::vector<QLabel*> m_statusList;
    std::vector<QLineEdit*> m_fileNameList;
    std::vector<QCheckBox*> m_checkBoxList;
    std::vector<unsigned int> m_writeCount;

};

#endif // DATALOGGERWIDGET_H
