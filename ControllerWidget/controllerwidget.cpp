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

    connect(m_worker, SIGNAL(sendMsgToWidget(QString)),
            ui->statusTextEdit, SLOT(insertPlainText(QString)));
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

void ControllerWidget::on_taskSpaceGroupBox_toggled(bool arg1)
{
    if(arg1 && ui->configSpaceGroupBox->isChecked())
        ui->configSpaceGroupBox->setChecked(false);
    if(arg1 && ui->jointSpaceGroupBox->isChecked())
        ui->jointSpaceGroupBox->setChecked(false);
}

void ControllerWidget::on_configSpaceGroupBox_toggled(bool arg1)
{
    if(arg1 && ui->taskSpaceGroupBox->isChecked())
        ui->taskSpaceGroupBox->setChecked(false);
    if(arg1 && ui->jointSpaceGroupBox->isChecked())
        ui->jointSpaceGroupBox->setChecked(false);
}

void ControllerWidget::on_jointSpaceGroupBox_toggled(bool arg1)
{
    if(arg1 && ui->configSpaceGroupBox->isChecked())
        ui->configSpaceGroupBox->setChecked(false);
    if(arg1 && ui->taskSpaceGroupBox->isChecked())
        ui->taskSpaceGroupBox->setChecked(false);
}
