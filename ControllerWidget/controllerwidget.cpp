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
    connect(this, SIGNAL(updateTaskSpaceCommand(double,double,double,double,bool)),
            m_worker, SLOT(updateTaskSpaceCommand(double,double,double,double,bool)));

    // control cycle signals
    connect(this, SIGNAL(startControlCycle()), m_worker, SLOT(startControlCycle()));
    connect(this, SIGNAL(stopControlCycle()), m_worker, SLOT(stopControlCycle()));

    // reset BB
    connect(this, SIGNAL(tellWorkerToResetBB()), m_worker, SLOT(resetBB()));

    // Gains Widget
    connect(gainWidget, SIGNAL(closeGainsWindow()), this, SLOT(on_adjustGainsButton_clicked()));
    connect(gainWidget, SIGNAL(setGains(GainsPYRT)), m_worker, SLOT(setGains(GainsPYRT)));
    connect(gainWidget, SIGNAL(setLimits(ConvergenceLimits)), m_worker, SLOT(setLimits(ConvergenceLimits)));

    // Initalize gains and limits to defaults
    gainWidget->on_setGainsButton_clicked();
    gainWidget->on_setLimitsButton_clicked();

    // Mode Flags
    connect(this, SIGNAL(updateModeFlags(ModeFlags)), m_worker, SLOT(setModeFlags(ModeFlags)));

    // US angle
    connect(this, SIGNAL(updateUSangle(double)), m_worker, SLOT(setUSangle(double)));

    ui->jointSpaceGroupBox->setChecked(false);
    ui->configSpaceGroupBox->setChecked(false);

}

ControllerWidget::~ControllerWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "Controller thread quit.";

    gainWidget->close();

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
    qDebug() << QTime::currentTime() << "Widget Thread ID: " << reinterpret_cast<int>(QThread::currentThreadId());

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
    double pitch = ui->pitchSpinbox->value() * piOverDeg180;
    double yaw   = ui->yawSpinbox->value() * piOverDeg180;
    double roll  = ui->rollSpinbox->value() * piOverDeg180;
    double trans = ui->transSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("[%5] Joint Space Command: %1 %2 %3 %4")
                                        .arg(pitch)
                                        .arg(yaw)
                                        .arg(roll)
                                        .arg(trans)
                                        .arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateJointSpaceCommand(pitch, yaw, roll, trans);
}

void ControllerWidget::on_updateConfigSpaceButton_clicked()
{
    double alpha = ui->alphaSpinbox->value() * piOverDeg180;
    double theta = ui->thetaSpinbox->value() * piOverDeg180;
    double gamma = ui->gammaSpinbox->value() * piOverDeg180;
    double d     = ui->dSpinbox->value();

    ui->statusTextEdit->appendPlainText(QString("[%5] Config Space Command: %1 %2 %3 %4")
                                        .arg(alpha)
                                        .arg(theta)
                                        .arg(gamma)
                                        .arg(d)
                                        .arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateConfigSpaceCommand(alpha, theta, gamma, d);
}

void ControllerWidget::on_updateTaskSpaceButton_clicked()
{
    double x      = ui->xSpinbox->value();
    double y      = ui->ySpinbox->value();
    double z      = ui->zSpinbox->value();
    double delPsi = ui->delPsiSpinbox->value() * piOverDeg180;
    bool isAbsolute = ui->absoluteRadiobutton->isChecked();

    ui->statusTextEdit->appendPlainText(QString("[%6] Task Space Command: %1 %2 %3 %4 %5")
                                        .arg(x)
                                        .arg(y)
                                        .arg(z)
                                        .arg(delPsi)
                                        .arg(isAbsolute ? "Abs" : "Rel") // ? : ternary operator
                                        .arg(QTime::currentTime().toString("HH:mm:ss")));

    emit updateTaskSpaceCommand(x, y, z, delPsi, isAbsolute);
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

void ControllerWidget::on_relativeRadiobutton_clicked()
{
    ui->xSpinbox->setValue(0.0);
    ui->ySpinbox->setValue(0.0);
    ui->zSpinbox->setValue(0.0);
    ui->delPsiSpinbox->setValue(0.0);
}

void ControllerWidget::on_absoluteRadiobutton_clicked()
{
    ui->xSpinbox->setValue(0.0);
    ui->ySpinbox->setValue(0.0);
    ui->zSpinbox->setValue(70.0);
    ui->delPsiSpinbox->setValue(0.0);
}

void ControllerWidget::on_updateFlagsButton_clicked()
{
    ModeFlags flags;

//    int coordFrame;
//    int tethered;
//    int instTrackState;
//    int instTrackMode;
//    int EKFstate;
//    int inVivoMode;


    if(ui->mobileRadioButton->isChecked())
        flags.coordFrame = COORD_FRAME_MOBILE;
    else if(ui->worldRadioButton->isChecked())
        flags.coordFrame = COORD_FRAME_WORLD;
    else{
        qDebug() << "coordFrame: State not recognized!"; return; }

    if(ui->relativeModeRadioButton->isChecked())
        flags.tethered = MODE_RELATIVE;
    else if(ui->tetheredModeRadioButton->isChecked())
        flags.tethered = MODE_TETHETERED;
    else{
        qDebug() << "tethered: State not recognized!"; return; }

    if(ui->ITonRadioButton->isChecked())
        flags.instTrackState = INST_TRACK_ON;
    else if(ui->IToffRadioButton->isChecked())
        flags.instTrackState = INST_TRACK_OFF;
    else{
        qDebug() << "instTrackState: State not recognized!"; return; }

    if(ui->imagerRadioButton->isChecked())
        flags.instTrackMode = INST_TRACK_IMAGER;
    else if(ui->positionRadioButton->isChecked())
        flags.instTrackMode = INST_TRACK_POSITION;
    else{
        qDebug() << "instTrackMode: State not recognized!"; return; }

    if(ui->EKFonRadioButton->isChecked())
        flags.EKFstate = EKF_ON;
    else if(ui->EKFoffRadioButton->isChecked())
        flags.EKFstate = EKF_OFF;
    else{
        qDebug() << "EKFstate: State not recognized!"; return; }

    if(ui->InVivoOnRadioButton->isChecked())
        flags.inVivoMode = IN_VIVO_ON;
    else if(ui->InVivoOffRadioButton->isChecked())
        flags.inVivoMode = IN_VIVO_OFF;
    else{
        qDebug() << "inVivoMode: State not recognized!"; return; }

    emit updateModeFlags(flags);
}

void ControllerWidget::on_setUSangleButton_clicked()
{
    double usAngle = ui->usAngleSpinBox->value();

    emit updateUSangle(usAngle);
}
