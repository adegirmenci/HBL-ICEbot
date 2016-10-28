#ifndef FRAMECLIENTWIDGET_H
#define FRAMECLIENTWIDGET_H

#include <QWidget>
#include <QTimer>

#include "frameclientthread.h"

namespace Ui {
class FrameClientWidget;
}

class FrameClientWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FrameClientWidget(QWidget *parent = 0);
    ~FrameClientWidget();

    FrameClientThread *m_worker;

signals:
    void sendFrame();

private slots:
    void workerStatusChanged(int status);

    void on_sendFrameButton_clicked();

    void on_toggleAutoButton_clicked();

private:
    Ui::FrameClientWidget *ui;

    bool m_keepTransmitting;
    QTimer *m_transmitTimer;

    QThread m_thread; // FrameClientThread will live in here
};

#endif // FRAMECLIENTWIDGET_H
