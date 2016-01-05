#ifndef EPOS2WIDGET_H
#define EPOS2WIDGET_H

#include <QWidget>
#include <QThread>
#include <QLabel>
#include <QLCDNumber>

#include "epos2thread.h"

namespace Ui {
class EPOS2Widget;
}

class EPOS2Widget : public QWidget
{
    Q_OBJECT

public:
    explicit EPOS2Widget(QWidget *parent = 0);
    ~EPOS2Widget();

    EPOS2Thread *m_worker;

signals:
    void setServoTargetPos(const int axisID, long targetPos, bool moveAbsOrRel);
    void servoToPos();
    void servoToPos(const int axisID);
    void haltMotor(const int axisID);
    void enableMotor(const int axisID);
    void disableMotor(const int axisID);
    void startServoLoop();
    void stopServoLoop();
    void homeAllAxes();

private slots:
    void workerStatusChanged(int status);
    void workerMotorStatusChanged(const int motID, const int status);
    void workerMotorStatusChanged(std::vector<int> status);
    void workerMotorQcChanged(const int motID, const long QC);
    void workerMotorQcChanged(std::vector<long> QCs);
    void receiveDataFromWorker(int motorID, const QString &data);

    void on_moveAbsButton_clicked();

    void on_moveRelButton_clicked();

    void on_homingButton_clicked();

    void on_haltButton_clicked();

    void on_nodeIDcomboBox_currentIndexChanged(int index);

    void on_enableNodeButton_clicked();

    void on_disableNodeButton_clicked();

    void on_enableServoLoopButton_clicked();

    void on_disableServoLoopButton_clicked();

    void on_homeAllButton_clicked();

private:
    Ui::EPOS2Widget *ui;

    QThread m_thread; // EPOS Thread will live in here

    std::vector<QLabel*> motLabels;
    std::vector<QLCDNumber*> motQCs;

    void updateManualControls(const int motorStatus);

};

#endif // EPOS2WIDGET_H
