#include "controllerthread.h"

ControllerThread::ControllerThread(QObject *parent) :
    QObject(parent)
{
    qRegisterMetaType<ModeFlags>("ModeFlags");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepControlling = false;
    m_abort = false;
    m_latestReading.resize(4);

    m_numCycles = 0;

    m_cathKin = Kinematics_4DOF(0.05*1000.0, 0.00135*1000.0, 0.90*0.0254*1000.0);
//    m_respModel = CyclicModel();

    loadConstants();

    // zero initialize
    m_targetPos = m_targetPos.Identity();
    m_deltaXYZPsiToTarget << 0.0, 0.0, 0.0, 0.0;
    m_input_AbsXYZ << 0.0, 0.0, 0.0;
    m_input_RelXYZ << 0.0, 0.0, 0.0;
    m_input_delPsi = 0.0;
    m_dXYZPsi << 0.0, 0.0, 0.0, 0.0;

    m_mutex = new QMutex(QMutex::Recursive);

    m_isReady = true;

    emit statusChanged(CONTROLLER_INITIALIZED);
}

ControllerThread::~ControllerThread()
{
//    std::cout << "FwdKin\n" << m_cathKin.forwardKinematics(1,0.1,0.1,0).matrix() << std::endl;

//    std::cout << "Jacobian\n" << m_cathKin.JacobianNumeric(1,0.1,0.1,0).matrix() << std::endl;

//    std::cout << "InvKin3D\n" << m_cathKin.inverseKinematics3D(5,5,12,0.1) << std::endl;

//    Eigen::Transform<double, 3, Eigen::Affine> T_in(Eigen::Matrix<double, 4, 4>::Identity());
//    std::cout << "T_in\n" << T_in.matrix() << std::endl;
//    Eigen::Vector4d configIn(0.0010, 0.0001, 0.0001, 0);
//    std::cout << "control_icra2016\n" << m_cathKin.control_icra2016(T_in, configIn, 0.0) << std::endl;

    qDebug() << "Ending ControllerThread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";

    // stop controller
    stopControlCycle();

    m_mutex->lock();
    m_abort = true;
    m_mutex->unlock();

    delete m_mutex;

    emit finished();
}

// ------------------------------
//     SLOTS IMPLEMENTATION
// ------------------------------
void ControllerThread::setEpoch(const QDateTime &epoch)
{
    QMutexLocker locker(m_mutex);

    if(!m_keepControlling)
    {
        m_epoch = epoch;
        m_isEpochSet = true;

        emit logEventWithMessage(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_EPOCH_SET,
                                 m_epoch.toString("dd/MM/yyyy - hh:mm:ss.zzz"));
    }
    else
        emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_EPOCH_SET_FAILED);
}

void ControllerThread::printThreadID()
{
    qDebug() << QTime::currentTime() << "Worker Thread ID: " << reinterpret_cast<int>(QThread::currentThreadId());

    m_respModel.testLPF();
}

// THIS FUNCTION SHOULD NOT BE USED AT ALL
void ControllerThread::receiveEMdata(QTime timeStamp, int sensorID, DOUBLE_POSITION_MATRIX_TIME_Q_RECORD data)
{
    // Data Format
    // | Sensor ID | Time Stamp | x | y | z | q1 | q2 | q3 | q4 |
    // |    int    |   double   |    ...    double     ...      |

    QMutexLocker locker(m_mutex);

    m_prevReading[sensorID] = m_latestReading[sensorID];
    m_latestReading[sensorID] = data;

    QElapsedTimer elTimer;
    elTimer.start();

    // TODO: emit signal to log receiveEMdata event
    //emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_EM_RECEIVED);

    switch(sensorID)
    {
    case EM_SENSOR_BB:
        Transform_From_EMreading(data, m_basTipPos_mobile);
        m_Box_BBmobile = m_basTipPos_mobile * m_BB_SBm.inverse();
        m_BBfixed_BBmobile = m_BB_Box * m_Box_BBmobile;
        m_BBmobile_CT = m_Box_BBmobile.inverse()*m_curTipPos*m_STm_BT*m_BT_CT;
        break;
    case EM_SENSOR_BT:
        // process CT point
        Transform_From_EMreading(data, m_curTipPos);
        m_BB_CT_curTipPos = m_BB_Box*m_curTipPos*m_STm_BT*m_BT_CT; // convert to CT in terms of BBfixed
        break;
    case EM_SENSOR_INST:
        Transform_From_EMreading(data, m_targetPos);
        m_targetPos = m_targetPos * m_ISm_INSTR;
        m_BB_targetPos = m_BB_Box * m_targetPos;
        break;
    case EM_SENSOR_CHEST:
        Transform_From_EMreading(data, m_currChest);
        break;
    default:
        qDebug() << "SensorID not recognized!";
        break;
    }

    qint64 elNsec = elTimer.nsecsElapsed();

    qDebug() << "Nsec elapsed:" << elNsec;
}

void ControllerThread::receiveLatestEMreading(std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> readings)
{
    QMutexLocker locker(m_mutex);

//    QElapsedTimer elTimer;
//    elTimer.start();

    Q_ASSERT(4 == readings.size());

    m_prevReading = m_latestReading;
    m_latestReading = readings;

    // EM_SENSOR_BB:
    Transform_From_EMreading(readings[EM_SENSOR_BB], m_Box_SBm);
    // EM_SENSOR_BT
    Transform_From_EMreading(readings[EM_SENSOR_BT], m_curTipPos);
    // EM_SENSOR_INST:
    Transform_From_EMreading(readings[EM_SENSOR_INST], m_targetPos);
    // EM_SENSOR_CHEST:
    Transform_From_EMreading(readings[EM_SENSOR_CHEST], m_currChest);

    // Calculate T_BBmobile_CT
    m_basTipPos_mobile = m_Box_SBm;
    m_Box_BBmobile = m_Box_SBm * m_BB_SBm.inverse();
    m_BBfixed_BBmobile = m_BB_Box * m_Box_BBmobile;
    // this is from MATLAB >>> Get BT w.r.t. mobile BB, rather than fixed BB
    // this is from MATLAB >>> m_BBmobile_BT = m_BBfixed_BBmobile.inverse() * m_BB_CT_curTipPos * m_BT_CT.inverse();
    m_BBmobile_BT = m_Box_BBmobile.inverse() * m_curTipPos * m_STm_BT;
    // this is the original C++ >>> m_BBmobile_CT = m_Box_BBmobile.inverse() * m_curTipPos * m_STm_BT * m_BT_CT;
    m_BBmobile_CT = m_BBmobile_BT * m_BT_CT;

    //Calculate T_BB_CT_curTipPos:
    // process CT point
    m_BB_CT_curTipPos = m_BB_Box * m_curTipPos * m_STm_BT * m_BT_CT; // convert to CT in terms of BBfixed

    //Calculate T_BB_targetPos
    m_targetPos = m_targetPos * m_ISm_INSTR; // the tip of the instrument in EM coord
    m_BB_targetPos = m_BB_Box * m_targetPos; // the tip of the instr in BBfixed coord

    controlCycle();

//    qint64 elNsec = elTimer.nsecsElapsed();
//    qDebug() << "Nsec elapsed:" << elNsec;

    //std::cout << m_BB_CT_curTipPos.matrix() << std::endl;
    // display in GUI
    QString msg = QString("T_BBfixed_CT:\n");
    msg += QString::number(m_BB_CT_curTipPos(0,0), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(0,1), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(0,2), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(0,3), 'f', 2) + "\n"
       + QString::number(m_BB_CT_curTipPos(1,0), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(1,1), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(1,2), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(1,3), 'f', 2) + "\n"
       + QString::number(m_BB_CT_curTipPos(2,0), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(2,1), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(2,2), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(2,3), 'f', 2) + "\n"
       + QString::number(m_BB_CT_curTipPos(3,0), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(3,1), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(3,2), 'f', 3) + "  "
       + QString::number(m_BB_CT_curTipPos(3,3), 'f', 2) + "\n";

    msg += QString("\nT_BBmobile_CT:\n")
       + QString::number(m_BBmobile_CT(0,0), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(0,1), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(0,2), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(0,3), 'f', 2) + "\n"
       + QString::number(m_BBmobile_CT(1,0), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(1,1), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(1,2), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(1,3), 'f', 2) + "\n"
       + QString::number(m_BBmobile_CT(2,0), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(2,1), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(2,2), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(2,3), 'f', 2) + "\n"
       + QString::number(m_BBmobile_CT(3,0), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(3,1), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(3,2), 'f', 3) + "  "
       + QString::number(m_BBmobile_CT(3,3), 'f', 2) + "\n";

    msg += QString("\nT_BBfixed_Instr:\n")
       + QString::number(m_BB_targetPos(0,0), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(0,1), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(0,2), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(0,3), 'f', 2) + "\n"
       + QString::number(m_BB_targetPos(1,0), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(1,1), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(1,2), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(1,3), 'f', 2) + "\n"
       + QString::number(m_BB_targetPos(2,0), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(2,1), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(2,2), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(2,3), 'f', 2) + "\n"
       + QString::number(m_BB_targetPos(3,0), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(3,1), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(3,2), 'f', 3) + "  "
       + QString::number(m_BB_targetPos(3,3), 'f', 2);

    emit sendMsgToWidget(msg);

    //emit logEventWithMessage(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), 0, msg);

}

void ControllerThread::updateJointSpaceCommand(double pitch, double yaw, double roll, double trans)
{
    // qDebug() << "New Joint Target Received";
    QMutexLocker locker(m_mutex);

    // check limits

    // update motor QCs

    // emit event to DataLogger
}

void ControllerThread::updateConfigSpaceCommand(double alpha, double theta, double gamma, double d)
{
    // this is not implemented at the moment, just test code

    // qDebug() << "New Config Target Received";
    QMutexLocker locker(m_mutex);

    // run through kinematics
    Eigen::Transform<double,3,Eigen::Affine> tempT;
    tempT = m_cathKin.forwardKinematics(gamma, theta, alpha, d);

    // check limits
    double norm = (m_targetPos.matrix().col(3) - tempT.matrix().col(3)).norm();
    qDebug() << "Norm:" << norm;
    if(norm < 500.0) // less than 50 cm
    {
        // update target
        ;
    }

    // emit event to DataLogger
}

void ControllerThread::updateTaskSpaceCommand(double x, double y, double z, double delPsi, bool isAbsolute)
{
    // qDebug() << "New Task Target Received";
    QMutexLocker locker(m_mutex);

    // Store the current tip pose as the new origin
    m_BBfixed_CTorig = m_BB_CT_curTipPos;

    // update target
    if(isAbsolute)
    {
        m_input_AbsXYZ(0) = x;
        m_input_AbsXYZ(1) = y;
        m_input_AbsXYZ(2) = z;
        m_input_RelXYZ(0) = 0.0; // TODO: this may be problematic, might want to chage the GUI to have both abs and rel editable at the same time
        m_input_RelXYZ(1) = 0.0;
        m_input_RelXYZ(2) = 0.0;

//        m_deltaXYZPsiToTarget(0) = x - m_BBfixed_CTorig(0,3);
//        m_deltaXYZPsiToTarget(1) = y - m_BBfixed_CTorig(1,3);
//        m_deltaXYZPsiToTarget(2) = z - m_BBfixed_CTorig(2,3);
    }
    else
    {
        m_input_RelXYZ(0) = x;
        m_input_RelXYZ(1) = y;
        m_input_RelXYZ(2) = z;
        m_input_AbsXYZ(0) = 0.0; // TODO: this may be problematic, might want to chage the GUI to have both abs and rel editable at the same time
        m_input_AbsXYZ(1) = 0.0;
        m_input_AbsXYZ(2) = 0.0;

//        m_deltaXYZPsiToTarget(0) = x;
//        m_deltaXYZPsiToTarget(1) = y;
//        m_deltaXYZPsiToTarget(2) = z;
    }
    m_input_delPsi = delPsi;
//    m_deltaXYZPsiToTarget(3) = delPsi;

    // check limits

    // emit event to DataLogger
    //std::cout << "New deltaXYZPsi : " << m_deltaXYZPsiToTarget << std::endl;
    std::cout << "New AbsXYZ RelXYZ DelPsi: " << m_input_AbsXYZ << m_input_RelXYZ << m_input_delPsi << std::endl;
}

void ControllerThread::resetBB()
{
    QMutexLocker locker(m_mutex);

    // get latest BB reading, and put it into m_basTipPos_fixed ( = T_Box_SBm_fixed )
    // m_basTipPos_fixed is the same as T_Box_SBm_fixed
    Transform_From_EMreading(m_latestReading[EM_SENSOR_BB], m_basTipPos_fixed);

    // update m_BB_Box
    m_BB_Box = m_BB_SBm * m_basTipPos_fixed.inverse(); // T_BB_Box = T_BB_SBm * T_SBm_Box

    // Save the fixed BB position, T_Box_BBfixed = T_Box_SBm_fixed * inv(T_BB_SBm)
    m_Box_BBfixed = m_basTipPos_fixed * m_BB_SBm.inverse();

    // TODO: Log event
    // TODO: Save Transform to file
}

void ControllerThread::startControlCycle()
{
    QMutexLocker locker(m_mutex);

    if(m_isReady && !m_keepControlling) // ready to control
    {
        //Start the cycle
        m_keepControlling = true;

//        m_timer = new QTimer(this);
//        connect(m_timer, SIGNAL(timeout()), this, SLOT(controlCycle()));

//        m_timer->start(6); // every 6ms

//        if(m_timer->isActive())
//        {
//            qDebug() << "Timer started.";
//            emit statusChanged(CONTROLLER_LOOP_STARTED);
//            emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_LOOP_STARTED);
//        }
//        else
//        {
//            qDebug() << "Timer is not active.";
//            emit statusChanged(CONTROLLER_LOOP_STOPPED);
//            emit logError(SRC_CONTROLLER, LOG_ERROR, QTime::currentTime(), CONTROLLER_INITIALIZE_FAILED, QString("Timer is not active."));
//        }
        emit statusChanged(CONTROLLER_LOOP_STARTED);
    }
    else
    {
        if(m_keepControlling)
            qDebug() << "Controller is already running.";
        else
        {
            qDebug() << "Controller is not ready.";
            emit statusChanged(CONTROLLER_INITIALIZE_FAILED);
            emit logError(SRC_CONTROLLER, LOG_ERROR, QTime::currentTime(), CONTROLLER_INITIALIZE_FAILED, QString("LabJack is not ready."));
        }
    }
}

void ControllerThread::stopControlCycle()
{
    QMutexLocker locker(m_mutex);

    if(m_keepControlling)
    {
        m_keepControlling = false;

//        m_timer->stop();

//        disconnect(m_timer, SIGNAL(timeout()), 0, 0);

//        delete m_timer;

        emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_LOOP_STOPPED);
        emit statusChanged(CONTROLLER_LOOP_STOPPED);

        qDebug() << "Timer stopped.";
    }
    else
    {
        qDebug() << "Timer already stopped.";
    }
}

void ControllerThread::setGains(GainsPYRT gains)
{
    QMutexLocker locker(m_mutex);

    m_gains = gains;

    locker.unlock();

    qDebug() << "Gains set:\nPitch:" << gains.kPitchMin << gains.kPitchMax
             << "\nYaw:" << gains.kYawMin << gains.kYawMax
             << "\nRoll:" << gains.kRollMin << gains.kRollMax
             << "\nTrans:" << gains.kTransMin << gains.kTransMax;
}

void ControllerThread::setLimits(ConvergenceLimits limits)
{
    QMutexLocker locker(m_mutex);

    m_convLimits = limits;

    locker.unlock();

    qDebug() << "Convergence limits set:\nPosition:" << limits.posMin << limits.posMax
             << "\nAngle:" << limits.angleMin << limits.angleMax;
}

void ControllerThread::setModeFlags(ModeFlags flags)
{
    QMutexLocker locker(m_mutex);

    m_modeFlags = flags;

    locker.unlock();

    qDebug() << "Mode flags set: " << flags.coordFrame
             << flags.tethered << flags.instTrackState
             << flags.instTrackMode << flags.EKFstate
             << flags.inVivoMode;
}

void ControllerThread::setUSangle(double usAngle)
{
    QMutexLocker locker(m_mutex);

    usAngle *= piOverDeg180;
    m_USangle = usAngle;

    m_BT_CT = m_BT_CT.Identity();
    m_BT_CT(0,0) = cos(usAngle);
    m_BT_CT(1,1) = m_BT_CT(0,0);
    m_BT_CT(0,1) = -sin(usAngle);
    m_BT_CT(1,0) = -m_BT_CT(0,1);
    m_BT_CT(2,3) = 21.7;

//    m_BT_CT << cos(usAngle), -sin(usAngle), 0,    0,
//               sin(usAngle),  cos(usAngle), 0,    0,
//                         0,              0, 1, 21.7;

    std::cout << "New m_BT_CT:\n" << m_BT_CT.matrix() << std::endl;
    qInfo() << "US angle updated to" << m_USangle*deg180overPi << "degrees.";
}

void ControllerThread::controlCycle()
{
    //QMutexLocker locker(m_mutex); // already locked by calling function

    // TODO : should we check if the resp model is collecting training data here?
    // this way, all other control code will be bypassed

    if(m_isReady && m_keepControlling)
    {
        switch(m_modeFlags.coordFrame)
        {
        case COORD_FRAME_WORLD:
            computeCoordFrameWorld();
            break;
        case COORD_FRAME_MOBILE:
            computeCoordFrameMobile();
            break;
        default:
            computeCoordFrameWorld();
            qCritical() << "Unknown Coord Frame Mode! Defaulting to World";
            break;
        }

        // calculate gains
        updateGains();

        // feed into control_icra2016
        Eigen::Matrix<double, 4, 2> jointsCurrAndTarget;
        jointsCurrAndTarget = m_cathKin.control_icra2016(m_BB_CT_curTipPos, m_dXYZPsi, m_currGamma);

        // get relative motor counts
        Eigen::Vector4d relQCs;  // knob_tgt - knob_curr
        relQCs(0) = (jointsCurrAndTarget(0,1) - jointsCurrAndTarget(0,0)) * m_gains.kTrans * 0.001 * EPOS_TRANS_RAD2QC;
        relQCs(1) = (jointsCurrAndTarget(1,1) - jointsCurrAndTarget(1,0)) * m_gains.kPitch * EPOS_PITCH_RAD2QC;
        relQCs(2) = (jointsCurrAndTarget(2,1) - jointsCurrAndTarget(2,0)) * m_gains.kYaw * EPOS_YAW_RAD2QC;
        relQCs(3) = (jointsCurrAndTarget(3,1) - jointsCurrAndTarget(3,0)) * m_gains.kRoll * EPOS_ROLL_RAD2QC;

        // check if QCs are finite
        if( !(isfinite(relQCs(0)) && isfinite(relQCs(1)) && isfinite(relQCs(2)) && isfinite(relQCs(3))) )
        {
            relQCs = relQCs.Zero();
        }

        std::vector<long> targetPos;
        targetPos.push_back((long)relQCs(0));
        targetPos.push_back((long)relQCs(1));
        targetPos.push_back((long)relQCs(2));
        targetPos.push_back((long)relQCs(3));

        // limit QCs to EPOS velocity limits
        // TODO : velocity is in RPMs, so we should convert this velocity to QCs/control cycle (= Ascension time = ~6ms)
        if(abs(targetPos[0]) > EPOS_VELOCITY[0])
            targetPos[0] = boost::math::copysign(EPOS_VELOCITY[0], targetPos[0]);
        if(abs(targetPos[1]) > EPOS_VELOCITY[1])
            targetPos[1] = boost::math::copysign(EPOS_VELOCITY[1], targetPos[1]);
        if(abs(targetPos[2]) > EPOS_VELOCITY[2])
            targetPos[2] = boost::math::copysign(EPOS_VELOCITY[2], targetPos[2]);
        if(abs(targetPos[3]) > EPOS_VELOCITY[3])
            targetPos[3] = boost::math::copysign(EPOS_VELOCITY[3], targetPos[3]);

        printf("QCs - T: %ld P: %ld Y: %ld R: %ld\n", targetPos[0], targetPos[1], targetPos[2], targetPos[3]);

        // TODO : check if the EPOS Servo Loop is active, if not, don't send commands
        emit setEPOSservoTargetPos(targetPos, false); //relative

        // emit logData(QTime::currentTime(), newData);

        qDebug() << QTime::currentTime() << "Cycle:" << m_numCycles;

        m_numCycles++;
    }
}

void ControllerThread::computeCoordFrameWorld()
{
    // Calculate the angle between the new x-axis and the original x-axis
    Eigen::Transform<double,3,Eigen::Affine> T_CTorig_CT = m_BBfixed_CTorig.inverse()*m_BB_CT_curTipPos;
    // This is the total amount of psy that has occurred at the tip since we started
    double total_psy = atan2(T_CTorig_CT(1,0), T_CTorig_CT(0,0));

    // Existing roll in the catheter handle
    // Assuming the BB point has rotated about its Z axis, the amount of roll in
    // the handle is calculated as the angle of rotation about the base z-axis.
    // Here we calculate the angle between the new x-axis and the original
    // x-axis.
    double m_currGamma = atan2(m_BBfixed_BBmobile(1,0), m_BBfixed_BBmobile(0,0));

    printf("Psy : %.3f Gamma : %.3f\n", total_psy * deg180overPi, m_currGamma * deg180overPi);

    switch(m_modeFlags.instTrackState)
    {
    case INST_TRACK_OFF:

        // How much does the cath still need to move?
        // calculate delta x,y,z,psi
        // MATLAB code cycleinput5.m > lines 112-126

        switch(m_modeFlags.tethered)
        {
        case MODE_TETHETERED:
            m_deltaXYZPsiToTarget(0) = m_input_AbsXYZ(0) - m_BBfixed_CTorig(0,3);
            m_deltaXYZPsiToTarget(1) = m_input_AbsXYZ(1) - m_BBfixed_CTorig(1,3);
            m_deltaXYZPsiToTarget(2) = m_input_AbsXYZ(2) - m_BBfixed_CTorig(2,3);
            std::cout << "Running this." << std::endl;
            break;
        case MODE_RELATIVE:
            m_deltaXYZPsiToTarget(0) = m_input_RelXYZ(0);
            m_deltaXYZPsiToTarget(1) = m_input_RelXYZ(1);
            m_deltaXYZPsiToTarget(2) = m_input_RelXYZ(2);
            break;
        default:
            m_deltaXYZPsiToTarget(0) = m_input_AbsXYZ(0) - m_BBfixed_CTorig(0,3);
            m_deltaXYZPsiToTarget(1) = m_input_AbsXYZ(1) - m_BBfixed_CTorig(1,3);
            m_deltaXYZPsiToTarget(2) = m_input_AbsXYZ(2) - m_BBfixed_CTorig(2,3);
            qCritical() << "Unknown Mode! Defaulting to tethered.";
            break;
        }
        m_deltaXYZPsiToTarget(3) = m_input_delPsi;

        m_dXYZPsi(0) = m_deltaXYZPsiToTarget(0) - (m_BB_CT_curTipPos(0,3) - m_BBfixed_CTorig(0,3));
        m_dXYZPsi(1) = m_deltaXYZPsiToTarget(1) - (m_BB_CT_curTipPos(1,3) - m_BBfixed_CTorig(1,3));
        m_dXYZPsi(2) = m_deltaXYZPsiToTarget(2) - (m_BB_CT_curTipPos(2,3) - m_BBfixed_CTorig(2,3));
        m_dXYZPsi(3) = m_deltaXYZPsiToTarget(3) - total_psy;

        break;
    case INST_TRACK_ON:
        // World, IT
        switch(m_modeFlags.instTrackMode)
        {
        case INST_TRACK_POSITION:
        // World, IT, Position
            switch(m_modeFlags.EKFstate)
            {
            case EKF_OFF: // World, IT, Position
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - m_BB_CT_curTipPos(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - m_BB_CT_curTipPos(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - m_BB_CT_curTipPos(2,3);
                m_dXYZPsi(3) = 0;
                break;
            case EKF_ON: // World, IT, Position, EKF
//                dx = T_BBfixed_Instr_EKF_x - T_BBfixed_CT(1,4);
//                dy = T_BBfixed_Instr_EKF_y - T_BBfixed_CT(2,4);
//                dz = T_BBfixed_Instr_EKF_z - T_BBfixed_CT(3,4);
//                dpsi = 0;
                // TODO : add EKF info here
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - m_BB_CT_curTipPos(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - m_BB_CT_curTipPos(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - m_BB_CT_curTipPos(2,3);
                m_dXYZPsi(3) = 0;
                break;
            default:
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - m_BB_CT_curTipPos(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - m_BB_CT_curTipPos(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - m_BB_CT_curTipPos(2,3);
                m_dXYZPsi(3) = 0;
                qCritical() << "Unknown EKFstate! Defaulting to EKF Off.";
                break;
            }
            break;
        // World, IT, Position ends
        case INST_TRACK_IMAGER:
        // World, IT, Imager
        {
            Eigen::Vector3d objectXYZ;

            switch(m_modeFlags.EKFstate)
            {
            case EKF_OFF: // World, IT, Imager
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                break;
            case EKF_ON: // World, IT, Imager, EKF
//                object_loc = [T_BBfixed_Instr_EKF_x;
//                              T_BBfixed_Instr_EKF_y;
//                              T_BBfixed_Instr_EKF_z];
                // TODO : add EKF info here
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                break;
            default:
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                qCritical() << "Unknown EKFstate! Defaulting to EKF Off.";
                break;
            }

            m_dXYZPsi(0) = m_input_AbsXYZ(0) - m_BB_CT_curTipPos(0,3);
            m_dXYZPsi(1) = m_input_AbsXYZ(1) - m_BB_CT_curTipPos(1,3);
            m_dXYZPsi(2) = m_input_AbsXYZ(2) - m_BB_CT_curTipPos(2,3);
            m_dXYZPsi(3) = computeSweep(m_BB_CT_curTipPos, objectXYZ);
        }
            break;
        // World, IT, Imager ends
        default:
            break;
        }
        break;
        // World, IT ends
    default:
        break;
    }

    printf("dx : %.3f dy : %.3f dz : %.3f dpsi : %.3f\n", m_dXYZPsi(0), m_dXYZPsi(1), m_dXYZPsi(2), m_dXYZPsi(3)* deg180overPi);
}

void ControllerThread::computeCoordFrameMobile()
{

}

double ControllerThread::computeSweep(const Eigen::Transform<double,3,Eigen::Affine> &currT, const Eigen::Vector3d &objXYZ)
{
    Eigen::Vector3d currLoc = currT.matrix().col(3).segment(0,3); // current location catheter xyz
    Eigen::Vector3d y_1 = currT.matrix().col(1).segment(0,3); // normal to the catheter's XZ plane
    // distance from the current point to the obj
    double d_obj_cur = (objXYZ - currLoc).norm();
    // distance from object to the catheter's XZ plane
    double d_obj_plane = y_1.dot(objXYZ - currLoc) / y_1.norm();

    double ratio = d_obj_plane / d_obj_cur;
    if(abs(ratio) > 1.0)
        ratio = ratio / abs(ratio); // otherwise, asin will be invalid

    double psy = 0.0;

    if(d_obj_cur > 0.0001)
        psy = asin(ratio); // angle between object and the catheter's XZ plane

    return psy;
}

void ControllerThread::updateGains()
{
    // Calculate distance from target
    double distError = m_dXYZPsi.segment(0,3).norm();
    double angleError = m_dXYZPsi(3);

    // Update gains
    double kPitch = 0.0, kYaw = 0.0, kRoll = 0.0, kTrans = 0.0;

    if(distError < m_convLimits.posMin)
    {
        kPitch = m_gains.kPitchMin;
        kYaw = m_gains.kYawMin;
        kTrans = m_gains.kTransMin;
    }
    else if(distError > m_convLimits.posMax)
    {
        kPitch = m_gains.kPitchMax;
        kYaw = m_gains.kYawMax;
        kTrans = m_gains.kTransMax;
    }
    else
    {
        double tPos = (distError - m_convLimits.posMin)/(m_convLimits.posMax - m_convLimits.posMin);
        kPitch = lerp(m_gains.kPitchMin, m_gains.kPitchMax, tPos);
        kYaw = lerp(m_gains.kYawMin, m_gains.kYawMax, tPos);
        kTrans = lerp(m_gains.kTransMin, m_gains.kTransMax, tPos);
    }

    if(angleError < m_convLimits.angleMin)
    {
        kRoll = m_gains.kRollMin;
    }
    else if(angleError > m_convLimits.angleMax)
    {
        kRoll = m_gains.kRollMax;
    }
    else
    {
        double tAng = (angleError - m_convLimits.angleMin)/(m_convLimits.angleMax - m_convLimits.angleMin);
        kRoll = lerp(m_gains.kRollMin, m_gains.kRollMax, tAng);
    }

    m_gains.kRoll = kRoll;
    m_gains.kPitch = kPitch;
    m_gains.kYaw = kYaw;
    m_gains.kTrans = kTrans;

    std::cout << "Gains - kR: " << kRoll << " kP: " << kPitch << " kY: " << kYaw << " kT: " << kTrans << std::endl;
}

// ----------------
//    ACCESSORS
// ----------------




// ----------------
// HELPER FUNCTIONS
// ----------------
static void Transform_From_EMreading(DOUBLE_POSITION_MATRIX_TIME_Q_RECORD &input, Eigen::Transform<double,3,Eigen::Affine> &output)
{
//    QElapsedTimer elTimer;
//    elTimer.start();

    Eigen::Translation3d trans(input.x, input.y, input.z);
    Eigen::Matrix3d rot;
    rot << input.s[0][0], input.s[0][1], input.s[0][2],
           input.s[1][0], input.s[1][1], input.s[1][2],
           input.s[2][0], input.s[2][1], input.s[2][2];
//    Eigen::Matrix3d rot;
//    rot << input.s[0][0], input.s[1][0], input.s[2][0],
//           input.s[0][1], input.s[1][1], input.s[2][1],
//           input.s[0][2], input.s[1][2], input.s[2][2];

    output = trans * rot;

    // std::cout << output.matrix() << std::endl;

//    qint64 elNsec = elTimer.nsecsElapsed();

//    qDebug() << "Nsec elapsed:" << elNsec;
}

Eigen::Transform<double,3,Eigen::Affine> ControllerThread::readTransformFromTxtFile(const QString &path)
{
    Eigen::Transform<double,3,Eigen::Affine> tform;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "File doesn't exist:" << path << " !!!";
        return tform;
    }

    QTextStream in(&file);
    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 4; j++)
        {
            if(!in.atEnd())
            {
                in >> tform(i,j);
            }
            else
            {
                qDebug() << "File ended prematurely at element (" << i << "," << j << "):" << path << " !!!";
                return tform;
            }
        }
    }

    // convert from meters to mm
    tform.translation() *= 1000.0;

    std::cout << "Loaded:" << path.toStdString().c_str() << std::endl;
    std::cout << tform.matrix() << std::endl;

    return tform;
}

void ControllerThread::loadConstants()
{
    m_BB_Box = readTransformFromTxtFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\Constants\\C_T_BB_Box.txt"));
    m_STm_BT = readTransformFromTxtFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\Constants\\C_T_STm_BT.txt"));
    m_BT_CT = readTransformFromTxtFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\Constants\\C_T_BT_CT.txt"));
    m_BB_SBm = readTransformFromTxtFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\Constants\\C_T_BB_SBm.txt"));
    m_ISm_INSTR = readTransformFromTxtFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\Constants\\C_T_ISm_INSTR.txt"));
}


inline const QString getCurrTimeStr()
{
    return QTime::currentTime().toString("HH.mm.ss.zzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("dd/MM/yyyy - hh:mm:ss.zzz");
}
