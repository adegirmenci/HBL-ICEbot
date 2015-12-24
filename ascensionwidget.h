#ifndef ASCENSIONWIDGET_H
#define ASCENSIONWIDGET_H

#include <QWidget>
#include <QThread>

#include "ascensionthread.h"

namespace Ui {
class AscensionWidget;
}

class AscensionWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AscensionWidget(QWidget *parent = 0);
    ~AscensionWidget();

    AscensionThread *m_worker;

private slots:
    void workerStatusChanged(int status);
    void on_initButton_clicked();
    void receiveDataFromWorker(int sensorID, const QString &data);


private:
    Ui::AscensionWidget *ui;

    QThread m_thread; // Ascension Thread will live in here
};

#endif // ASCENSIONWIDGET_H
