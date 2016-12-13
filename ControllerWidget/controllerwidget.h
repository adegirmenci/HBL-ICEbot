#ifndef CONTROLLERWIDGET_H
#define CONTROLLERWIDGET_H

#include <QWidget>
#include <QThread>

#include "controllerthread.h"
#include "gainswidget.h"
#include "respmodelwidget.h"

namespace Ui {
class ControllerWidget;
}

class ControllerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControllerWidget(QWidget *parent = 0);
    ~ControllerWidget();

    ControllerThread *m_worker;

signals:
    void tellWorkerToPrintThreadID();
    void updateJointSpaceCommand(double pitch, double yaw, double roll, double trans);
    void updateConfigSpaceCommand(double alpha, double theta, double gamma, double d);
    void updateTaskSpaceCommand(double x, double y, double z, double delPsi, bool isAbsolute);
    void tellWorkerToResetBB();
    void startControlCycle(); // start control loop in worker
    void stopControlCycle(); // stop control loop in worker
    void updateModeFlags(ModeFlags flags);
    void updateUSangle(double usAngle);

private slots:
    void workerStatusChanged(int status);
    void receiveMsgFromWorker(QString msg);

    void on_testButton_clicked();

    void on_taskSpaceGroupBox_toggled(bool arg1);

    void on_configSpaceGroupBox_toggled(bool arg1);

    void on_jointSpaceGroupBox_toggled(bool arg1);

    void on_updateJointSpaceButton_clicked();

    void on_updateConfigSpaceButton_clicked();

    void on_updateTaskSpaceButton_clicked();

    void on_controllerToggleButton_clicked();

    void on_resetBB_Button_clicked();

    void on_adjustGainsButton_clicked();

    void on_relativeRadiobutton_clicked();

    void on_absoluteRadiobutton_clicked();

    void on_updateFlagsButton_clicked();

    void on_setUSangleButton_clicked();

    void on_respModelButton_clicked();

private:
    Ui::ControllerWidget *ui;

    gainsWidget *gainWidget;
    respModelWidget *m_respModelWidget;

    QThread m_thread; // Controller Thread will live in here
};

#endif // CONTROLLERWIDGET_H
