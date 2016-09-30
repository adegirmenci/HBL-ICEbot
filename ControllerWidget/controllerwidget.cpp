#include "controllerwidget.h"
#include "ui_controllerwidget.h"

ControllerWidget::ControllerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerWidget)
{
    ui->setupUi(this);

    m_worker = new ControllerThread();
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    connect(this, SIGNAL(tellWorkerToPrintThreadID()),m_worker,SLOT(printThreadID()));
}

ControllerWidget::~ControllerWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "Controller thread quit.";

    delete ui;
}

void ControllerWidget::on_testButton_clicked()
{
    qDebug() << QTime::currentTime() << "Widget Thread ID: " << QThread::currentThreadId();

    emit tellWorkerToPrintThreadID();
}
