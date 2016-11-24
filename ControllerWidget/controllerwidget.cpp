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

    // updates
    connect(this, SIGNAL(updateJointSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateJointSpaceCommand(double,double,double,double)));
    connect(this, SIGNAL(updateConfigSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateConfigSpaceCommand(double,double,double,double)));
    connect(this, SIGNAL(updateTaskSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateTaskSpaceCommand(double,double,double,double)));

    ui->jointSpaceGroupBox->setChecked(false);
    ui->configSpaceGroupBox->setChecked(false);
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

void ControllerWidget::on_updateJointSpaceButton_clicked()
{
    double pitch = ui->pitchSpinbox->value();
    double yaw   = ui->yawSpinbox->value();
    double roll  = ui->rollSpinbox->value();
    double trans = ui->transSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("Joint Space Command: %1 %2 %3 %4").arg(pitch).arg(yaw).arg(roll).arg(trans));

    // TODO : update target in controller thread
}

void ControllerWidget::on_updateConfigSpaceButton_clicked()
{
    double alpha = ui->alphaSpinbox->value();
    double theta = ui->thetaSpinbox->value();
    double gamma = ui->gammaSpinbox->value();
    double d     = ui->dSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("Config Space Command: %1 %2 %3 %4").arg(alpha).arg(theta).arg(gamma).arg(d));

    // TODO : update target in controller thread
}

void ControllerWidget::on_updateTaskSpaceButton_clicked()
{
    double x      = ui->xSpinbox->value();
    double y      = ui->ySpinbox->value();
    double z      = ui->zSpinbox->value();
    double delPsi = ui->delPsiSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("Task Space Command: %1 %2 %3 %4").arg(x).arg(y).arg(z).arg(delPsi));

    // TODO : update target in controller thread
}
