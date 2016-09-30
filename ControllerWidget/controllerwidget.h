#ifndef CONTROLLERWIDGET_H
#define CONTROLLERWIDGET_H

#include <QWidget>
#include <QThread>

#include "controllerthread.h"

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

private slots:
    void on_testButton_clicked();

private:
    Ui::ControllerWidget *ui;

    QThread m_thread; // Controller Thread will live in here
};

#endif // CONTROLLERWIDGET_H
