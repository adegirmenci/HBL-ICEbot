#include "controllerwidget.h"
#include "ui_controllerwidget.h"

ControllerWidget::ControllerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerWidget),
    gainWidget(new gainsWidget),
    m_respModelWidget(new respModelWidget)
{
    ui->setupUi(this);

    m_worker = new ControllerThread();
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    connect(this, SIGNAL(tellWorkerToPrintThreadID()), m_worker, SLOT(printThreadID()));

    connect(m_worker, SIGNAL(sendMsgToWidget(QString,int)),
            this, SLOT(receiveMsgFromWorker(QString,int)));

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

    // Resp Model Widget
    connect(m_respModelWidget, SIGNAL(closeRespModelWindow()), this, SLOT(on_respModelButton_clicked()));
    connect(m_respModelWidget, SIGNAL(initializeRespModel()), m_worker, SLOT(initializeRespModel()));
    connect(m_respModelWidget, SIGNAL(re_initializeRespModel()), m_worker, SLOT(re_initializeRespModel()));
    connect(m_respModelWidget, SIGNAL(stopRespModel()), m_worker, SLOT(stopRespModel()));
    connect(m_respModelWidget, SIGNAL(newFutureSamplesValue(int)), m_worker, SLOT(updateFutureSamples(int)));
//    connect(m_worker, SIGNAL(sendDataToRespModelWidget(int,bool,bool,double,EigenVectorFiltered,EigenVectorFiltered,EigenVectorFiltered)),
//            m_respModelWidget, SLOT(receiveDataFromRespModel(int,bool,bool,double,EigenVectorFiltered,EigenVectorFiltered,EigenVectorFiltered)));
    connect(m_worker, SIGNAL(sendDataToRespModelWidget(int,bool,bool,double)),
            m_respModelWidget, SLOT(receiveDataFromRespModel(int,bool,bool,double)));

    connect(&(m_worker->m_respModel), SIGNAL(sendToPlotBird4(unsigned int, double, double)),
            m_respModelWidget, SLOT(plotBird4(unsigned int,double,double)));

    // Sweep
    connect(this, SIGNAL(workerStartSweep(uint,double,double,qint64)),
            m_worker, SLOT(startSweep(uint,double,double,qint64)));
    connect(this, SIGNAL(workerAbortSweep()),
            m_worker, SLOT(abortSweep()));

    // Initalize gains and limits to defaults
    gainWidget->on_setGainsButton_clicked();
    gainWidget->on_setLimitsButton_clicked();

    // Mode Flags
    connect(this, SIGNAL(updateModeFlags(ModeFlags)), m_worker, SLOT(setModeFlags(ModeFlags)));

    // US angle
    connect(this, SIGNAL(updateUSangle(double)), m_worker, SLOT(setUSangle(double)));

    ui->jointSpaceGroupBox->setChecked(false);
    ui->configSpaceGroupBox->setChecked(false);

    // trajectory
    m_keepDriving = false;
    m_currTrajIdx = 0;

    m_ctr = 0;

    connect(m_worker, SIGNAL(reportCurrentXYZPSI(XYZPSI)), this, SLOT(receiveCurrentXYZPSI(XYZPSI)));

}

ControllerWidget::~ControllerWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "Controller thread quit.";

    gainWidget->close();
    //m_respModelWidget->close();
    emit m_respModelWidget->closeRespModelWindow();

    delete gainWidget;
    delete m_respModelWidget;
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

void ControllerWidget::receiveMsgFromWorker(QString msg, int destination)
{
    if(destination == 0)
    {
        ui->statusTextEdit->clear();
        ui->statusTextEdit->appendPlainText(msg);
    }
    else
    {
        ui->statusLineEdit->setText(msg);
    }
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

void ControllerWidget::on_respModelButton_clicked()
{
    if(m_respModelWidget->isHidden())
    {
        ui->respModelButton->setText("Close RespModel Win");
        m_respModelWidget->show();
        m_respModelWidget->raise();
    }
    else
    {
        ui->respModelButton->setText("Respiration Model");
        m_respModelWidget->close();
    }
}

void ControllerWidget::receiveCurrentXYZPSI(XYZPSI currXYZPSI)
{
    m_currXYZPSI = currXYZPSI;

    if( (m_ctr % 5) == 0 )
    {
        ui->currXspinbox->setValue(currXYZPSI.x);
        ui->currYspinbox->setValue(currXYZPSI.y);
        ui->currZspinbox->setValue(currXYZPSI.z);
        ui->currDelPsiSpinbox->setValue(currXYZPSI.psi);
    }

    m_ctr++;
}

void ControllerWidget::on_trajOpenFileButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open CSV File"),
                                                    "../ICEbot_QT_v1/LoggedData",
                                                    tr("CSV File (*.csv)"));

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << file.errorString();
        return;
    }

    QTextStream in(&file);

    m_XYZPSIs.clear();

    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList split = line.split(','); // files are generated using dlmwrite in MATLAB
        if(split.size() == 4)
        {
            XYZPSI xyzpsi;
            xyzpsi.x = split[0].toDouble();
            xyzpsi.y = split[1].toDouble();
            xyzpsi.z = split[2].toDouble();
            xyzpsi.psi = split[3].toDouble() * piOverDeg180; // convert to radians
            double signedPI = boost::math::copysign(pi, xyzpsi.psi);
            xyzpsi.psi = fmod(xyzpsi.psi + signedPI,(2*pi)) - signedPI;
            m_XYZPSIs.push_back(xyzpsi);
        }
        else
        {
            qDebug() << "Error reading CSV - number of elements in line is not equal to 4!";
            break;
        }
    }

    qDebug() << "Read" << m_XYZPSIs.size() << "setpoints from trajectory file.";

    if(m_XYZPSIs.size() > 0){
        ui->trajDriveButton->setEnabled(true);
        ui->trajStepLineEdit->setText(QString("%1 points loaded.").arg(m_XYZPSIs.size()));
    }
    else{
        ui->trajDriveButton->setEnabled(false);
        ui->trajStepLineEdit->setText("Error reading file.");
    }
}

void ControllerWidget::on_trajDriveButton_clicked()
{
    if(m_keepDriving)
    {
        m_keepDriving = false;
        ui->trajDriveButton->setText("Drive");

        // stop timer
        m_trajTimer->stop();
        disconnect(m_trajTimer, SIGNAL(timeout()), 0, 0);
        delete m_trajTimer;
    }
    else
    {
        m_keepDriving = true;
        ui->trajDriveButton->setText("Stop");

        m_currTrajIdx = 0;
        ui->trajStepLineEdit->setText(QString("%1 of %2.").arg(m_currTrajIdx).arg(m_XYZPSIs.size()));

        XYZPSI targetXYZPSI = m_XYZPSIs[m_currTrajIdx];

        // send to worker
        emit updateTaskSpaceCommand(targetXYZPSI.x, targetXYZPSI.y, targetXYZPSI.z, targetXYZPSI.psi, true);

        // update spinbox in UI
        ui->xSpinbox->setValue(targetXYZPSI.x);
        ui->ySpinbox->setValue(targetXYZPSI.y);
        ui->zSpinbox->setValue(targetXYZPSI.z);
        ui->delPsiSpinbox->setValue(targetXYZPSI.psi * deg180overPi);

        // start timer
        m_trajTimer = new QTimer(this);
        connect(m_trajTimer, SIGNAL(timeout()), this, SLOT(driveTrajectory()));
        m_trajTimer->start(10);
    }
}

void ControllerWidget::driveTrajectory()
{
    XYZPSI targetXYZPSI = m_XYZPSIs[m_currTrajIdx];
    if( (abs(m_currXYZPSI.x - targetXYZPSI.x) < 0.75) &&
        (abs(m_currXYZPSI.y - targetXYZPSI.y) < 0.75) &&
        (abs(m_currXYZPSI.z - targetXYZPSI.z) < 0.75) && m_keepDriving)
    {
        m_currTrajIdx++;
        ui->trajStepLineEdit->setText(QString("%1 of %2.").arg(m_currTrajIdx).arg(m_XYZPSIs.size()));
        if(m_currTrajIdx < m_XYZPSIs.size())
        {
            // get next traget
            targetXYZPSI = m_XYZPSIs[m_currTrajIdx];

            // send to worker
            emit updateTaskSpaceCommand(targetXYZPSI.x, targetXYZPSI.y, targetXYZPSI.z, targetXYZPSI.psi, true);

            // update spinbox in UI
            ui->xSpinbox->setValue(targetXYZPSI.x);
            ui->ySpinbox->setValue(targetXYZPSI.y);
            ui->zSpinbox->setValue(targetXYZPSI.z);
            ui->delPsiSpinbox->setValue(targetXYZPSI.psi * deg180overPi);

            qDebug() << "Drive to" << targetXYZPSI.x << targetXYZPSI.y << targetXYZPSI.z << targetXYZPSI.psi;
        }
        else
        {
            m_keepDriving = false;
            ui->trajDriveButton->setText("Drive");
        }
    }

}

void ControllerWidget::on_sweepButton_clicked()
{
    unsigned int nSteps = ui->numSweepsSpinBox->value();
    double stepAngle = ui->sweepStepDblSpinBox->value() * piOverDeg180;
    double convLimit = ui->sweepThreshDblSpinBox->value() * piOverDeg180;
    qint64 imgDuration = ui->acqTimeMsSpinBox->value();

    emit workerStartSweep(nSteps, stepAngle, convLimit, imgDuration);
}

void ControllerWidget::on_abortSweepButton_clicked()
{
    emit workerAbortSweep();
}
