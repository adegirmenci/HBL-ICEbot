#include "controllerwidget.h"
#include "ui_controllerwidget.h"

ControllerWidget::ControllerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerWidget),
    gainWidget(new gainsWidget)
{
    ui->setupUi(this);

    m_worker = new ControllerThread();
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    connect(this, SIGNAL(tellWorkerToPrintThreadID()), m_worker, SLOT(printThreadID()));

    connect(m_worker, SIGNAL(sendMsgToWidget(QString)),
            this, SLOT(receiveMsgFromWorker(QString)));

    connect(m_worker, SIGNAL(statusChanged(int)),
            this, SLOT(workerStatusChanged(int)));

    // updates
    connect(this, SIGNAL(updateJointSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateJointSpaceCommand(double,double,double,double)));
    connect(this, SIGNAL(updateConfigSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateConfigSpaceCommand(double,double,double,double)));
    connect(this, SIGNAL(updateTaskSpaceCommand(double,double,double,double)),
            m_worker, SLOT(updateTaskSpaceCommand(double,double,double,double)));

    // control cycle signals
    connect(this, SIGNAL(startControlCycle()), m_worker, SLOT(startControlCycle()));
    connect(this, SIGNAL(stopControlCycle()), m_worker, SLOT(stopControlCycle()));

    // reset BB
    connect(this, SIGNAL(tellWorkerToResetBB()), m_worker, SLOT(resetBB()));

    ui->jointSpaceGroupBox->setChecked(false);
    ui->configSpaceGroupBox->setChecked(false);

}

ControllerWidget::~ControllerWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "Controller thread quit.";

    delete gainWidget;
    delete ui;
}

void ControllerWidget::workerStatusChanged(int status)
{
    switch(status)
    {
    case CONTROLLER_INITIALIZED:
        ui->controllerToggleButton->setEnabled(true);
        ui->statusLineEdit->setText("Initialized.");
        break;
    case CONTROLLER_LOOP_STARTED:
        ui->controllerToggleButton->setText( QStringLiteral("Stop Controller") );
        ui->controllerToggleButton->setEnabled(true);
        ui->statusLineEdit->setText("Running.");
        break;
    case CONTROLLER_LOOP_STOPPED:
        ui->controllerToggleButton->setText( QStringLiteral("Start Controller") );
        ui->controllerToggleButton->setEnabled(true);
        ui->statusLineEdit->setText("Stopped.");
        break;
    case CONTROLLER_EPOCH_SET:
        ui->statusTextEdit->appendPlainText("Epoch set.");
        break;
    case CONTROLLER_EPOCH_SET_FAILED:
        ui->statusTextEdit->appendPlainText("Epoch set failed!");
        break;
    case CONTROLLER_RESET:
        ui->statusTextEdit->appendPlainText("Controller reset.");
        break;
    case CONTROLLER_RESET_FAILED:
        ui->statusTextEdit->appendPlainText("Controller reset failed!");
        break;
    default:
        ui->statusTextEdit->appendPlainText("Unknown state!");
        break;
    }
}

void ControllerWidget::receiveMsgFromWorker(QString msg)
{
    ui->statusTextEdit->clear();
    ui->statusTextEdit->appendPlainText(msg);
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

    ui->statusTextEdit->appendPlainText(QString("[%5] Joint Space Command: %1 %2 %3 %4").arg(pitch).arg(yaw).arg(roll).arg(trans).arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateJointSpaceCommand(pitch, yaw, roll, trans);
}

void ControllerWidget::on_updateConfigSpaceButton_clicked()
{
    double alpha = ui->alphaSpinbox->value();
    double theta = ui->thetaSpinbox->value();
    double gamma = ui->gammaSpinbox->value();
    double d     = ui->dSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("[%5] Config Space Command: %1 %2 %3 %4").arg(alpha).arg(theta).arg(gamma).arg(d).arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateConfigSpaceCommand(alpha, theta, gamma, d);
}

void ControllerWidget::on_updateTaskSpaceButton_clicked()
{
    double x      = ui->xSpinbox->value();
    double y      = ui->ySpinbox->value();
    double z      = ui->zSpinbox->value();
    double delPsi = ui->delPsiSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("[%5] Task Space Command: %1 %2 %3 %4").arg(x).arg(y).arg(z).arg(delPsi).arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateTaskSpaceCommand(x, y, z, delPsi);
}

void ControllerWidget::on_controllerToggleButton_clicked()
{
    ui->controllerToggleButton->setEnabled(false); // disable button

    if(m_worker->isControlling())
    {
        // TODO: this should stop LabJack collection
        emit stopControlCycle();
    }
    else
    {
        // TODO: this should trigger LabJack collection
        emit startControlCycle();
    }
}

void ControllerWidget::on_resetBB_Button_clicked()
{
    emit tellWorkerToResetBB();
}

void ControllerWidget::on_adjustGainsButton_clicked()
{
    if(gainWidget->isHidden())
    {
        ui->adjustGainsButton->setText("Hide Gains");
        gainWidget->show();
        gainWidget->raise();
    }
    else
    {
        ui->adjustGainsButton->setText("Adjust Gains");
        //gainWidget->hide();
        gainWidget->close();
    }
}
