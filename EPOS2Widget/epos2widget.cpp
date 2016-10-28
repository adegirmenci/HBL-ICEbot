#include "epos2widget.h"
#include "ui_epos2widget.h"

EPOS2Widget::EPOS2Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EPOS2Widget)
{
    ui->setupUi(this);

    m_worker = new EPOS2Thread;
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    connect(ui->connectButton, SIGNAL(clicked(bool)), m_worker, SLOT(initializeEPOS()));
//    connect(ui->acquireButton, SIGNAL(clicked(bool)), m_worker, SLOT(startAcquisition()));
//    connect(ui->stopButton, SIGNAL(clicked(bool)), m_worker, SLOT(stopAcquisition()));
    connect(ui->disconnectButton, SIGNAL(clicked(bool)), m_worker, SLOT(disconnectEPOS()));

    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));
    connect(m_worker, SIGNAL(motorStatusChanged(int,int)), this, SLOT(workerMotorStatusChanged(int,int)));
    connect(m_worker, SIGNAL(motorStatusChanged(std::vector<int>)), this, SLOT(workerMotorStatusChanged(std::vector<int>)));
    connect(m_worker, SIGNAL(motorQcChanged(int,long)), this, SLOT(workerMotorQcChanged(int,long)));
    connect(m_worker, SIGNAL(motorQcChanged(std::vector<long>)), this, SLOT(workerMotorQcChanged(std::vector<long>)));
    connect(m_worker, SIGNAL(sendDataToGUI(int,QString)),
                             this, SLOT(receiveDataFromWorker(int,QString)));

    connect(this, SIGNAL(setServoTargetPos(int,long,bool)), m_worker, SLOT(setServoTargetPos(int,long,bool)));
    connect(this, SIGNAL(servoToPos()), m_worker, SLOT(servoToPosition()));
    connect(this, SIGNAL(servoToPos(int)), m_worker, SLOT(servoToPosition(int)));
    connect(this, SIGNAL(haltMotor(int)), m_worker, SLOT(haltMotor(int)));
    connect(this, SIGNAL(enableMotor(int)), m_worker, SLOT(initializeMotor(int)));
    connect(this, SIGNAL(disableMotor(int)), m_worker, SLOT(disableMotor(int)));
    connect(this, SIGNAL(startServoLoop()), m_worker, SLOT(startServoing()));
    connect(this, SIGNAL(stopServoLoop()), m_worker, SLOT(stopServoing()));
    connect(this, SIGNAL(homeAllAxes()), m_worker, SLOT(homeAllAxes()));

    // status labels
    motLabels.push_back(ui->transStatusLabel);
    motLabels.push_back(ui->pitchStatusLabel);
    motLabels.push_back(ui->yawStatusLabel);
    motLabels.push_back(ui->rollStatusLabel);
    // LCD numbers
    motQCs.push_back(ui->transQC_LCD);
    motQCs.push_back(ui->pitchQC_LCD);
    motQCs.push_back(ui->yawQC_LCD);
    motQCs.push_back(ui->rollQC_LCD);
}

EPOS2Widget::~EPOS2Widget()
{
    //motLabels.clear();

    m_thread.quit();
    m_thread.wait();
    qDebug() << "EPOS thread quit.";

    delete ui;
}

void EPOS2Widget::workerStatusChanged(int status)
{
    // TODO: consider putting this in a truth table and reading the entry based on the status code
    // then we will get rid of the entire switch/case and just have a few lines of code

    switch(status)
    {
    case EPOS_INITIALIZE_BEGIN:
        ui->connectButton->setEnabled(false);
        ui->outputTextEdit->appendPlainText("Connecting to EPOS...");
        break;
    case EPOS_INITIALIZE_FAILED:
        ui->connectButton->setEnabled(true);
        ui->outputTextEdit->appendPlainText("Connection failed!");
        break;
    case EPOS_INITIALIZED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->nodeIDcomboBox->setEnabled(true);
        ui->enableServoLoopButton->setEnabled(true);
        ui->disableServoLoopButton->setEnabled(false);
        ui->outputTextEdit->appendPlainText("Connected to EPOS.");
//        ui->outputTextEdit->appendPlainText(QString("%1 motor(s) plugged in.")
//                                            .arg(m_worker->getNumMotors()));
        break;
    case EPOS_DISCONNECTED:
        ui->connectButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->enableNodeButton->setEnabled(false);
        ui->nodeIDcomboBox->setEnabled(false);
        ui->enableServoLoopButton->setEnabled(false);
        ui->disableServoLoopButton->setEnabled(false);
        ui->outputTextEdit->appendPlainText("Disconnected from EPOS.");
        break;
    case EPOS_DISCONNECT_FAILED:
        ui->outputTextEdit->appendPlainText("Disconnection from EPOS failed!");
        break;
    case EPOS_SERVO_LOOP_STARTED:
        ui->enableServoLoopButton->setEnabled(false);
        ui->disableServoLoopButton->setEnabled(true);
        ui->haltButton->setEnabled(false); // disable halt button, use 'Disable Servo Loop' button to halt motors
        ui->outputTextEdit->appendPlainText("Servo loop started.");
        break;
    case EPOS_SERVO_LOOP_STOPPED:
        ui->enableServoLoopButton->setEnabled(true);
        ui->disableServoLoopButton->setEnabled(false);
        //ui->haltButton->setEnabled(true);
        updateManualControls(m_worker->getMotorStatus( ui->nodeIDcomboBox->currentIndex() ));
        ui->outputTextEdit->appendPlainText("Servo loop stopped.");
        break;
    case EPOS_SERVO_TO_POS_FAILED:
        ui->outputTextEdit->appendPlainText("Servo to position failed!");
        break;
    case EPOS_EPOCH_SET:
        ui->outputTextEdit->appendPlainText("Epoch set.");
        break;
    case EPOS_EPOCH_SET_FAILED:
        ui->outputTextEdit->appendPlainText("Epoch set failed!");
        break;
    case EPOS_DISABLE_MOTOR_FAILED:
        ui->outputTextEdit->appendPlainText("Motor disabled.");
        break;
    case EPOS_DISABLE_MOTOR_SUCCESS:
        ui->outputTextEdit->appendPlainText("Disable motor failed!");
        break;
    case EPOS_DEVICE_CANT_CONNECT:
        ui->outputTextEdit->appendPlainText("Can't connect to device!");
        break;
    case EPOS_UPDATE_QC_FAILED:
        ui->outputTextEdit->appendPlainText("Update QC failed!");
        break;
    case EPOS_HALT_FAILED:
        ui->outputTextEdit->appendPlainText("Halt failed!");
        break;
    default:
        ui->outputTextEdit->appendPlainText("Unknown state!");
        break;
    }
}

void EPOS2Widget::workerMotorStatusChanged(const int motID, const int status)
{
    int currID = ui->nodeIDcomboBox->currentIndex();
    if(currID == motID)
        updateManualControls(status);


    if( status == EPOS_MOTOR_ENABLED )
    {
        motLabels[motID]->setText("Enabled");
        motLabels[motID]->setStyleSheet("QLabel { background-color : green;}");
        ui->outputTextEdit->appendPlainText(QString("Motor %1 enabled.").arg(motID+1));
    }
    else if( status == EPOS_MOTOR_DISABLED )
    {
        motLabels[motID]->setText("Disabled");
        motLabels[motID]->setStyleSheet("QLabel { background-color : yellow;}");
        ui->outputTextEdit->appendPlainText(QString("Motor %1 disabled.").arg(motID+1));
    }
    else if(status == EPOS_MOTOR_FAULT)
    {
        motLabels[motID]->setText("FAULT!");
        motLabels[motID]->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui->outputTextEdit->appendPlainText(QString("Motor %1 fault!").arg(motID+1));
    }
    else if(status == EPOS_MOTOR_CANT_CONNECT)
    {
        motLabels[motID]->setText("Can't connect!");
        motLabels[motID]->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui->outputTextEdit->appendPlainText(QString("Cannot connect to Motor %1.").arg(motID+1));
    }
    else
    {
        motLabels[motID]->setText("Unknown state!");// UNKNOWN STATE!
        motLabels[motID]->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui->outputTextEdit->appendPlainText(QString("Motor %1 in inknown state.").arg(motID+1));
    }

}

void EPOS2Widget::workerMotorStatusChanged(std::vector<int> status)
{
    for(size_t i = 0; i < status.size(); i++)
        workerMotorStatusChanged(i, status[i]);
}

void EPOS2Widget::workerMotorQcChanged(const int motID, const long QC)
{
    if( motID < static_cast<int>(motQCs.size()) )
        motQCs[motID]->display(QC);
    else
        qDebug() << "MotorID out of bounds!";
}

void EPOS2Widget::workerMotorQcChanged(std::vector<long> QCs)
{
    for(size_t i = 0; i < QCs.size(); i++)
        workerMotorQcChanged(i, QCs[i]);
}

void EPOS2Widget::receiveDataFromWorker(int motorID, const QString &data)
{
    if(motorID == 0)
        ui->outputTextEdit->clear();

    ui->outputTextEdit->appendPlainText(data);
}

void EPOS2Widget::on_moveAbsButton_clicked()
{
    int axisID = ui->nodeIDcomboBox->currentIndex();
    long targetPos = ui->targetQCspinBox->value();

    emit setServoTargetPos(axisID, targetPos, true);
    if(!m_worker->isInServoLoop())
        emit servoToPos(axisID);
}

void EPOS2Widget::on_moveRelButton_clicked()
{
    int axisID = ui->nodeIDcomboBox->currentIndex();
    long targetPos = ui->targetQCspinBox->value();

    emit setServoTargetPos(axisID, targetPos, false);
    if(!m_worker->isInServoLoop())
        emit servoToPos(axisID);
}

void EPOS2Widget::on_homingButton_clicked()
{

    int axisID = ui->nodeIDcomboBox->currentIndex();
    long targetPos = 0;
    qDebug() << "Homing axis " << axisID;

    //if(!m_worker->isInServoLoop())
    emit stopServoLoop(); // stop servo loop

    emit setServoTargetPos(axisID, targetPos, true); // set target position

    emit servoToPos(axisID); // servo to position
}

void EPOS2Widget::on_haltButton_clicked()
{
    int axisID = ui->nodeIDcomboBox->currentIndex();

    emit haltMotor(axisID);
}

void EPOS2Widget::updateManualControls(const int motorStatus)
{
    if( motorStatus == EPOS_MOTOR_ENABLED )
    {
        ui->enableNodeButton->setEnabled(false);
        ui->disableNodeButton->setEnabled(true);
        ui->homingButton->setEnabled(true);
        ui->moveAbsButton->setEnabled(true);
        ui->moveRelButton->setEnabled(true);
        if(!m_worker->isInServoLoop())
            ui->haltButton->setEnabled(true);
        else
            ui->haltButton->setEnabled(false);
        ui->homeAllButton->setEnabled(true);
    }
    else if( motorStatus == EPOS_MOTOR_DISABLED )
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
        ui->homingButton->setEnabled(false);
        ui->moveAbsButton->setEnabled(false);
        ui->moveRelButton->setEnabled(false);
        ui->haltButton->setEnabled(false);
        ui->homeAllButton->setEnabled(false);
    }
    else if(motorStatus == EPOS_MOTOR_FAULT)
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
        ui->homingButton->setEnabled(false);
        ui->moveAbsButton->setEnabled(false);
        ui->moveRelButton->setEnabled(false);
        ui->haltButton->setEnabled(false);
        ui->homeAllButton->setEnabled(false);
    }
    else if(motorStatus == EPOS_MOTOR_CANT_CONNECT)
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
        ui->homingButton->setEnabled(false);
        ui->moveAbsButton->setEnabled(false);
        ui->moveRelButton->setEnabled(false);
        ui->haltButton->setEnabled(false);
        ui->homeAllButton->setEnabled(false);
    }
    else
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
        ui->homingButton->setEnabled(false);
        ui->moveAbsButton->setEnabled(false);
        ui->moveRelButton->setEnabled(false);
        ui->haltButton->setEnabled(false);
        ui->homeAllButton->setEnabled(false);
    }
}

void EPOS2Widget::on_nodeIDcomboBox_currentIndexChanged(int index)
{
    const int motorStatus = m_worker->getMotorStatus(index);

    if(motorStatus != EPOS_INVALID_MOTOR_ID)
        updateManualControls(motorStatus);
    else
        qDebug() << "Invalid motorID!";

}

void EPOS2Widget::on_enableNodeButton_clicked()
{
    const int axisID = ui->nodeIDcomboBox->currentIndex();

    emit enableMotor(axisID);
}

void EPOS2Widget::on_disableNodeButton_clicked()
{
    const int axisID = ui->nodeIDcomboBox->currentIndex();

    emit disableMotor(axisID);
}

void EPOS2Widget::on_enableServoLoopButton_clicked()
{
    emit startServoLoop();
}

void EPOS2Widget::on_disableServoLoopButton_clicked()
{
    emit stopServoLoop();
}

void EPOS2Widget::on_homeAllButton_clicked()
{
    // stop servo loop
    emit stopServoLoop();

    emit homeAllAxes();
}
