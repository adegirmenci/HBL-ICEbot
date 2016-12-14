#ifndef FRMGRABWIDGET_H
#define FRMGRABWIDGET_H

#include <QWidget>

#include "frmgrabthread.h"

namespace Ui {
class FrmGrabWidget;
}

class FrmGrabWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FrmGrabWidget(QWidget *parent = 0);
    ~FrmGrabWidget();

    FrmGrabThread *m_worker;

signals:
    void frmGrabConnect();
    void frmGrabInitialize(int width, int height, double fps);
    void startStream();
    void stopStream();
    void addSaveRequest(unsigned short numFrames);
    void frmGrabDisconnect();
    void toggleLiveFeed();
    void startSaving();
    void stopSaving();

private slots:
    void workerStatusChanged(int status);

    void on_connectButton_clicked();

    void on_initButton_clicked();

    void on_disconnectButton_clicked();

    void on_startStreamButton_clicked();

    void on_liveFeedButton_clicked();

    void on_saveFrameButton_clicked();

    void on_stopStreamButton_clicked();

    void on_saveFramesButton_clicked();

    void controlStarted();
    void controlStopped();

private:
    Ui::FrmGrabWidget *ui;

    QThread m_thread; // FrmGrab Thread will live in here

    bool mKeepSavingFrames;

};

#endif // FRMGRABWIDGET_H
