#include "controllerthread.h"

ControllerThread::ControllerThread(QObject *parent) :
    QObject(parent)
{
    qRegisterMetaType<ModeFlags>("ModeFlags");
    qRegisterMetaType<EigenVectorFiltered>("EigenVectorFiltered");
    qRegisterMetaType<XYZPSI>("XYZPSI");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepControlling = false;
    m_abort = false;
    m_latestReading.resize(4);

    m_respModelInitializing = false;

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

    m_mutex = new QMutex(QMutex::NonRecursive);

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

    QElapsedTimer elTimer;
    elTimer.start();

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

    std::vector<double> T_BB_CT_curTipPos(m_BB_CT_curTipPos.matrix().data(), m_BB_CT_curTipPos.matrix().data() + m_BB_CT_curTipPos.matrix().size());

    if(T_BB_CT_curTipPos.size() == 16)
    {
        QDateTime t; t.setMSecsSinceEpoch(readings[EM_SENSOR_BT].time*1000.0);
        emit logData(t.time(), m_numCycles, CONTROLLER_T_BB_CT_curTipPos, T_BB_CT_curTipPos);

        emit send_CT_toFrameClient(T_BB_CT_curTipPos, readings[EM_SENSOR_BT].time);
    }

    if(m_respModelInitializing)
    {
        // send training data
        //const EigenAffineTransform3d &T_BB_CT_curTipPos,
        //const EigenAffineTransform3d &T_BB_targetPos,
        //const EigenAffineTransform3d &T_Box_BBmobile,
        //const EigenAffineTransform3d &T_BB_Box,
        //const EigenAffineTransform3d &T_Bird4,
        m_respModel.addTrainingObservation(m_BB_CT_curTipPos, m_BB_targetPos, m_Box_BBmobile, m_BB_Box, m_currChest, readings[EM_SENSOR_CHEST].time);

        // check how many samples were sent and turn off flag if trained

        if(m_respModel.getNumSamples() == N_SAMPLES)
        {
            std::cout << "Train model.\n" << std::endl;
            m_respModel.trainModel();

            if(m_respModel.isTrained())
            {
                m_respModelInitializing = false;
                std::cout << "Model trained.\n" << std::endl;

                emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESP_MODEL_INITIALIZED);
            }
            else
            {
                // ERROR!
                std::cout << "Error in training resp model!" << std::endl;

                emit logEvent(SRC_CONTROLLER, LOG_ERROR, QTime::currentTime(), CONTROLLER_RESP_MODEL_INIT_FAILED);
            }
        }
    }
    else if(m_respModel.isTrained())
    {
        // send current data
        m_respModel.addObservation(m_currChest, readings[EM_SENSOR_CHEST].time);

        std::vector<double> omega = {m_respModel.getOmega()};
        emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_PERIOD, omega);
    }

    if(!m_respModelInitializing)
    {
        controlCycle();
    }

    // TODO : send data to resp model widget
    // size_t numSamples,
    //    bool isTrained,
    //    bool inVivoMode,
    //    double omega0,
    //    EigenVectorFiltered Bird4_filtered,
    //    EigenVectorFiltered Bird4_filtered_new,
    //    EigenVectorFiltered breathSignalFromModel);
    emit sendDataToRespModelWidget(m_respModel.getNumSamples(),
                                   m_respModel.isTrained(),
                                   m_respModel.isInVivoMode(),
                                   m_respModel.getOmega());//,
//                                   m_respModel.get_Bird4_filtered(),
//                                   m_respModel.get_Bird4_filtered_new(),
//                                   m_respModel.get_breathSignalFromModel());

    qint64 elNsec = elTimer.nsecsElapsed();
    std::cout << "EM receive Nsec elapsed: " << elNsec << std::endl;

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

    emit sendMsgToWidget(msg, 0);

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

    std::vector<double> xyzdxyzpsi = {m_input_AbsXYZ(0), m_input_AbsXYZ(1), m_input_AbsXYZ(2),
                                      m_input_RelXYZ(0), m_input_RelXYZ(1), m_input_RelXYZ(2), delPsi*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_USER_XYZDXYZPSI, xyzdxyzpsi);

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

    std::vector<double> T_basTipPos_fixed(m_basTipPos_fixed.matrix().data(), m_basTipPos_fixed.matrix().data() + m_basTipPos_fixed.matrix().size());

    emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESETBB_SUCCESS);
    if(T_basTipPos_fixed.size() == 16)
    {
        emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_RESETBB, T_basTipPos_fixed);
    }
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
        emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_LOOP_STARTED);
    }
    else
    {
        if(m_keepControlling)
            qDebug() << "Controller is already running.";
        else
        {
            qDebug() << "Controller is not ready.";
            emit statusChanged(CONTROLLER_INITIALIZE_FAILED);
            emit logError(SRC_CONTROLLER, LOG_ERROR, QTime::currentTime(), CONTROLLER_INITIALIZE_FAILED, QString("Controller is not ready."));
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

    if(flags.inVivoMode)
    {
        if(!m_modeFlags.inVivoMode) // we're turning on in vivo mode
        {
            m_respModel.setInVivo(true);
        }
    }
    else
    {
        if(m_modeFlags.inVivoMode) // we're turning off in vivo mode
        {
            m_respModel.setInVivo(false);
        }
    }

    m_modeFlags = flags;

    unsigned int numcyc = m_numCycles;

    locker.unlock();

//    qDebug() << "Mode flags set: " << flags.coordFrame
//             << flags.tethered << flags.instTrackState
//             << flags.instTrackMode << flags.EKFstate
//             << flags.inVivoMode;
    //emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESETBB_SUCCESS);

    std::vector<double> modes(6);
    modes.push_back(flags.coordFrame);
    modes.push_back(flags.tethered);
    modes.push_back(flags.instTrackState);
    modes.push_back(flags.instTrackMode);
    modes.push_back(flags.EKFstate);
    modes.push_back(flags.inVivoMode);
    emit logData(QTime::currentTime(), numcyc, CONTROLLER_MODES, modes);
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
    //qInfo() << "US angle updated to" << m_USangle*deg180overPi << "degrees.";

    std::vector<double> usang = {m_USangle*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_USANGLE, usang);
}

void ControllerThread::initializeRespModel()
{
    QMutexLocker locker(m_mutex);
    // TODO : start sending data to m_respModel
    m_respModelInitializing = true;

    emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESP_MODEL_INIT_BEGIN);
}

void ControllerThread::re_initializeRespModel()
{
    QMutexLocker locker(m_mutex);
    m_respModel.resetModel();
    m_respModelInitializing = true;

    emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESP_MODEL_INIT_BEGIN);
}

void ControllerThread::stopRespModel()
{
    QMutexLocker locker(m_mutex);
    m_respModel.resetModel();
    m_respModelInitializing = false;

    emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_RESP_MODEL_STOPPED);
}

void ControllerThread::updateFutureSamples(int n)
{
    if(n >= EDGE_EFFECT)
        m_respModel.updateNfutureSamples(n);
    else
        qDebug() << "updateFutureSamples: n < EDGE_EFFECT";
}

void ControllerThread::startSweep(unsigned int nSteps_, double stepAngle_, double convLimit_, qint64 imgDuration_)
{
    if(!m_sweep.getIsActive())
    {
        m_sweep = Sweep(nSteps_, stepAngle_, convLimit_, imgDuration_);
        m_sweep.activate();

        std::vector<double> sweepParams = {(double)nSteps_,
                                           stepAngle_*deg180overPi,
                                           convLimit_*deg180overPi,
                                           (double)imgDuration_};
        emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_SWEEP_START, sweepParams);
        emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_SWEEP_STARTED);
    }
    else
        qDebug() << "Already sweeping!";
}

void ControllerThread::abortSweep()
{
    m_sweep.abort();
    emit logEvent(SRC_CONTROLLER, LOG_INFO, QTime::currentTime(), CONTROLLER_SWEEP_ABORTED);
}

void ControllerThread::controlCycle()
{
    //QMutexLocker locker(m_mutex); // already locked by calling function

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

        // handle sweep
        if(m_sweep.getIsActive())
        {
            if(m_sweep.update( m_dXYZPsi(3) ))
            {
                // if done collecting
                m_input_delPsi += m_sweep.getStepAngle();

                QString msg = QString("%1 sweeps remaining.").arg(m_sweep.getRemainingSteps());
                emit sendMsgToWidget(msg, 1); // update status text
            }
        }

        // calculate gains
        updateGains();

        // feed into control_icra2016
        Eigen::Matrix<double, 4, 2> jointsCurrAndTarget;
        jointsCurrAndTarget = m_cathKin.control_icra2016(m_BB_CT_curTipPos, m_dXYZPsi, m_currGamma);
        //jointsCurrAndTarget = m_cathKin.control_icra2016(m_BBmobile_CT, m_dXYZPsi, m_currGamma);

        // get relative motor counts
        Eigen::Vector4d relQCs;  // knob_tgt - knob_curr
        relQCs(0) = (jointsCurrAndTarget(0,1) - jointsCurrAndTarget(0,0)) * m_gains.kTrans * 0.001 * EPOS_TRANS_RAD2QC;
        relQCs(1) = (jointsCurrAndTarget(1,1) - jointsCurrAndTarget(1,0)) * m_gains.kPitch * EPOS_PITCH_RAD2QC;
        relQCs(2) = (jointsCurrAndTarget(2,1) - jointsCurrAndTarget(2,0)) * m_gains.kYaw * EPOS_YAW_RAD2QC;
        relQCs(3) = (jointsCurrAndTarget(3,1) - jointsCurrAndTarget(3,0)) * m_gains.kRoll * EPOS_ROLL_RAD2QC;

        // Jacobian based
//        Eigen::Vector4d configCurr = m_cathKin.inverseKinematics(m_BB_CT_curTipPos, m_currGamma);
//        Eigen::Matrix<double, 6, 4> J = m_cathKin.JacobianNumeric(configCurr(0), configCurr(1), configCurr(2), configCurr(3));
//        m_cathKin.dampedLeastSquaresStep(J, m_dXYZPsi);

//        // *****
        Eigen::Vector4d configCurr = m_cathKin.inverseKinematics(m_BB_CT_curTipPos, m_currGamma);
        Eigen::Vector4d currTask = m_cathKin.configToTaskSpace(configCurr);
//        Eigen::Vector4d targetTask(m_input_AbsXYZ(0), m_input_AbsXYZ(1), m_input_AbsXYZ(2), m_input_delPsi);
//        Eigen::Vector4d targetConfig = m_cathKin.JacobianStep(currTask, targetTask, configCurr);
//        Eigen::Vector4d currJoint = m_cathKin.configToJointSpace(configCurr);
//        Eigen::Vector4d targetJoint = m_cathKin.configToJointSpace(targetConfig);

        std::vector<double> task = {currTask(0),currTask(1),currTask(2),currTask(3)};
        emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_CURR_TASK, task);

        emit reportCurrentXYZPSI(XYZPSI(currTask(0),currTask(1),currTask(2),currTask(3)));

//        // *****

        // FIXME: 1/20/2017: Trying to fix BBmobile discrepancy
//        // figure out distance from BBcurr to xy plane of BBfixed
//        double ratio = m_BBfixed_BBmobile(2,3)/m_BBfixed_BBmobile(2,2);
//        //Eigen::Vector4d offset_ =  m_BBfixed_BBmobile.matrix().col(2)*ratio; // extend along z axis
//        Eigen::Transform<double,3,Eigen::Affine> T_BBtrans_CT(m_BBmobile_CT);
//        //T_BBtrans_CT.matrix().col(3) += offset_;
//        T_BBtrans_CT(2,3) += ratio;

//        Eigen::Vector4d configCurr = m_cathKin.inverseKinematics(T_BBtrans_CT, m_currGamma);
//        Eigen::Vector4d currTask = m_cathKin.configToTaskSpace(configCurr);
////        Eigen::Vector4d tmp = {currTask(0),currTask(1),currTask(2),1.0};
////        tmp = (m_BBfixed_BBmobile * tmp).eval();
////        currTask.segment<3>(0) = tmp.segment<3>(0);
//        currTask.segment<3>(0) = m_BB_CT_curTipPos.matrix().col(3).segment<3>(0);
////        Eigen::Vector4d tmp(m_input_AbsXYZ(0), m_input_AbsXYZ(1), m_input_AbsXYZ(2),1.0);
////        tmp = (m_BBfixed_BBmobile.inverse() * tmp).eval();
////        Eigen::Vector4d targetTask(tmp(0), tmp(1), tmp(2)+ratio, m_input_delPsi);
//        Eigen::Vector4d targetTask(m_input_AbsXYZ(0), m_input_AbsXYZ(1), m_input_AbsXYZ(2), m_input_delPsi);
//        Eigen::Vector4d targetConfig = m_cathKin.JacobianStep(currTask, targetTask, configCurr);
//        Eigen::Vector4d currJoint = m_cathKin.configToJointSpace(configCurr);
//        Eigen::Vector4d targetJoint = m_cathKin.configToJointSpace(targetConfig);
////        Eigen::Vector4d deltaConfig = m_cathKin.JacobianStepSingle(currTask, targetTask, configCurr);
////        Eigen::Vector4d currJoint = m_cathKin.configToJointSpace(Eigen::Vector4d::Zero());
////        Eigen::Vector4d targetJoint = m_cathKin.configToJointSpace(deltaConfig);



//        // get relative motor counts
//        Eigen::Vector4d relQCs;  // knob_tgt - knob_curr
//        relQCs(0) = (targetJoint(0) - currJoint(0)) * m_gains.kTrans * 0.001 * EPOS_TRANS_RAD2QC;
//        relQCs(1) = (targetJoint(1) - currJoint(1)) * m_gains.kPitch * EPOS_PITCH_RAD2QC;
//        relQCs(2) = (targetJoint(2) - currJoint(2)) * m_gains.kYaw * EPOS_YAW_RAD2QC;
//        relQCs(3) = (targetJoint(3) - currJoint(3)) * m_gains.kRoll * EPOS_ROLL_RAD2QC;

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

        std::cout << QTime::currentTime().toString().toStdString() << " Cycle:" << m_numCycles << std::endl;

        m_numCycles++;
    }
    else
    {
        m_currGamma = atan2(m_BBfixed_BBmobile(1,0), m_BBfixed_BBmobile(0,0));
        Eigen::Vector4d configCurr = m_cathKin.inverseKinematics(m_BB_CT_curTipPos, m_currGamma);
        Eigen::Vector4d currTask = m_cathKin.configToTaskSpace(configCurr);

        std::vector<double> task = {currTask(0),currTask(1),currTask(2),currTask(3)};
        emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_CURR_TASK, task);
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
    m_currGamma = atan2(m_BBfixed_BBmobile(1,0), m_BBfixed_BBmobile(0,0));

    //printf("Psy : %.3f Gamma : %.3f\n", total_psy * deg180overPi, m_currGamma * deg180overPi);
    std::vector<double> psyGamma = {total_psy*deg180overPi, m_currGamma*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_CURR_PSY_GAMMA, psyGamma);

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

    //printf("dx : %.3f dy : %.3f dz : %.3f dpsi : %.3f\n", m_dXYZPsi(0), m_dXYZPsi(1), m_dXYZPsi(2), m_dXYZPsi(3)* deg180overPi);
    std::vector<double> dxyzpsi = {m_dXYZPsi(0), m_dXYZPsi(1), m_dXYZPsi(2), m_dXYZPsi(3)*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_DXYZPSI, dxyzpsi);
}

void ControllerThread::computeCoordFrameMobile()
{
    //% Use models to reconstruct one 4x4 matrix at this time point
//    % Tip
//    T_BBfixed_CTtraj(1:3,1:3) = convert_eaa_3x3(reshape(BBfixed_CTtraj_model(4:7),1,4));
//    T_BBfixed_CTtraj(1,4) = BBfixed_CTtraj_model(1);
//    T_BBfixed_CTtraj(2,4) = BBfixed_CTtraj_model(2);
//    T_BBfixed_CTtraj(3,4) = BBfixed_CTtraj_model(3);
//    T_BBfixed_CTtraj(4,1:4) = [0 0 0 1];

    EigenVector7d CT_des = m_respModel.getT_BBfixed_CT_des();
    EigenVector7d CT_future_des = m_respModel.getT_BBfixed_CTtraj_future_des();
    EigenVector7d BB_des = m_respModel.getT_BBfixed_BB_des();

    Eigen::AngleAxis<double> rotCTdes(CT_des(6), Eigen::Vector3d(CT_des(3),CT_des(4),CT_des(5)));
    Eigen::Translation3d transCTdes(CT_des(0),CT_des(1),CT_des(2));
    EigenAffineTransform3d T_BBfixed_CT_des = transCTdes * rotCTdes;
    //printf("transCTdes: x %.3f y %.3f z %.3f\n",CT_des(0),CT_des(1),CT_des(2));

    Eigen::AngleAxis<double> rotCTtraj(CT_future_des(6), Eigen::Vector3d(CT_future_des(3),CT_future_des(4),CT_future_des(5)));
    Eigen::Translation3d transCTtraj(CT_future_des(0),CT_future_des(1),CT_future_des(2));
    EigenAffineTransform3d T_BBfixed_CTtraj_future_des = transCTtraj * rotCTtraj;

    Eigen::AngleAxis<double> rotBBdes(BB_des(6), Eigen::Vector3d(BB_des(3),BB_des(4),BB_des(5)));
    Eigen::Translation3d transBBdes(BB_des(0),BB_des(1),BB_des(2));
    EigenAffineTransform3d T_BBfixed_BB_des = transBBdes * rotBBdes;

    //% Rotation about the z-axis of the catheter tip.
    //    % Here we calculate the angle between the current x-axis and the
    //    % pure-breathing x-axis.
    //    T_CTtraj_CT=inv(T_BBfixed_CTtraj)*T_BBfixed_CT;

    EigenAffineTransform3d T_CTtraj_CT = T_BBfixed_CT_des.inverse() * m_BB_CT_curTipPos;

    //    % This is the total amount of psy that has occurred at the tip compared
    //    % with pure breathing
    //    total_psy=atan2(T_CTtraj_CT(2,1),T_CTtraj_CT(1,1));

    double total_psy = atan2(T_CTtraj_CT(1,0), T_CTtraj_CT(0,0));

    //% Existing roll in the catheter handle, from Base_Roll which was a separate
    //    % function but is now incorporated into here
    //    T_BBtraj_BBmob = inv(T_BBfixed_BBtraj) * T_BBfixed_BBmob;
    //    % Assuming the BB point has rotated about its Z axis, the amount of roll in
    //    % the handle is calculated as the angle of rotation about the base z-axis.
    //    % Here we calculate the angle between the new x-axis and the trajectory
    //    % x-axis.
    //    gamma=atan2(T_BBtraj_BBmob(2,1),T_BBtraj_BBmob(1,1));

    EigenAffineTransform3d T_BBtraj_BBmob = T_BBfixed_BB_des.inverse() * m_BBfixed_BBmobile;

    m_currGamma = atan2(T_BBtraj_BBmob(1,0), T_BBtraj_BBmob(0,0));

    //printf("Psy : %.3f Gamma : %.3f\n", total_psy * deg180overPi, m_currGamma * deg180overPi);
    std::vector<double> psyGamma = {total_psy*deg180overPi, m_currGamma*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_CURR_PSY_GAMMA, psyGamma);

    //% Get BT w.r.t. mobile BB, rather than fixed BB
    //        T_BBmob_BT = inv(T_BBfixed_BBmob)*T_BBfixed_CT*inv(T_BT_CT);

    EigenAffineTransform3d T_BBmob_BT = m_BBfixed_BBmobile.inverse() * m_BBfixed_CTorig * m_BT_CT.inverse();

    //% Initialize even if EKF is off
    //    T_BBfixed_CT_future = zeros(1,16);

    EigenAffineTransform3d T_BBfixed_CT_future = EigenAffineTransform3d::Identity();


    switch(m_modeFlags.instTrackState)
    {
    case INST_TRACK_OFF:

        switch(m_modeFlags.tethered)
        {
        case MODE_TETHETERED:
            // TODO : % **** this is not implemented correctly right now
            //            % If the cath is tethered to a point... fix this later
            //            % Right now I'm just copying the relative case over here in case I
            //            % hit the wrong button by accident
            m_dXYZPsi(0) = m_input_AbsXYZ(0) + T_BBfixed_CT_des(0,3) - m_BB_CT_curTipPos(0,3);
            m_dXYZPsi(1) = m_input_AbsXYZ(1) + T_BBfixed_CT_des(1,3) - m_BB_CT_curTipPos(1,3);
            m_dXYZPsi(2) = m_input_AbsXYZ(2) + T_BBfixed_CT_des(2,3) - m_BB_CT_curTipPos(2,3);
            m_dXYZPsi(3) = m_input_delPsi - total_psy;
            break;
        case MODE_RELATIVE:
            //% If the cath is being told to do relative adjustments then that's
            //            % like adding the relative adjustments to the Traj point. Then I
            //            % just take the difference between where I am now and the relative
            //            % change from the Traj point.
            //            dx = inputbox_delt_x + T_BBfixed_CTtraj(1,4) - T_BBfixed_CT(1,4);
            //            dy = inputbox_delt_y + T_BBfixed_CTtraj(2,4) - T_BBfixed_CT(2,4);
            //            dz = inputbox_delt_z + T_BBfixed_CTtraj(3,4) - T_BBfixed_CT(3,4);
            //            dpsi = inputbox_delt_psi - total_psy;

            m_dXYZPsi(0) = m_input_AbsXYZ(0) + T_BBfixed_CT_des(0,3) - m_BB_CT_curTipPos(0,3);
            m_dXYZPsi(1) = m_input_AbsXYZ(1) + T_BBfixed_CT_des(1,3) - m_BB_CT_curTipPos(1,3);
            m_dXYZPsi(2) = m_input_AbsXYZ(2) + T_BBfixed_CT_des(2,3) - m_BB_CT_curTipPos(2,3);
            m_dXYZPsi(3) = m_input_delPsi - total_psy;

            break;
        default:
            qCritical() << "Unknown Mode! MOBILE - INST_TRACK_OFF.";
            break;
        }

        break;
    case INST_TRACK_ON: // Mobile, IT then

        switch(m_modeFlags.instTrackMode)
        {
        case INST_TRACK_POSITION:
        // Mobile, IT, Position
            switch(m_modeFlags.EKFstate)
            {
            case EKF_OFF: // Mobile, IT, Position
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - m_BB_CT_curTipPos(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - m_BB_CT_curTipPos(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - m_BB_CT_curTipPos(2,3);
                m_dXYZPsi(3) = 0.0;
                break;
            case EKF_ON: // Mobile, IT, Position, EKF
                T_BBfixed_CT_future = T_BBfixed_CTtraj_future_des * T_CTtraj_CT;
//                dx = T_BBfixed_Instr_EKF_x - T_BBfixed_CT_future(1,4);
//                dy = T_BBfixed_Instr_EKF_y - T_BBfixed_CT_future(2,4);
//                dz = T_BBfixed_Instr_EKF_z - T_BBfixed_CT_future(3,4);
//                dpsi = 0;
                // TODO : add EKF info here
                // this is doing inst track with predicted CT
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - T_BBfixed_CT_future(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - T_BBfixed_CT_future(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - T_BBfixed_CT_future(2,3);
                m_dXYZPsi(3) = 0;
                break;
            default:
                m_dXYZPsi(0) = m_BB_targetPos(0,3) - m_BB_CT_curTipPos(0,3);
                m_dXYZPsi(1) = m_BB_targetPos(1,3) - m_BB_CT_curTipPos(1,3);
                m_dXYZPsi(2) = m_BB_targetPos(2,3) - m_BB_CT_curTipPos(2,3);
                m_dXYZPsi(3) = 0.0;
                qCritical() << "Unknown EKFstate! Mobile, IT, Position.";
                break;
            }
            break;
        // Mobile, IT, Position ends
        case INST_TRACK_IMAGER:
        // Mobile, IT, Imager
        {
            Eigen::Vector3d objectXYZ;

            switch(m_modeFlags.EKFstate)
            {
            case EKF_OFF: // Mobile, IT, Imager
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                m_dXYZPsi(3) = computeSweep(m_BB_CT_curTipPos, objectXYZ);
                break;
            case EKF_ON: // Mobile, IT, Imager, EKF
                // % Take future CT traj point and transform by the same
                // % matrix that transforms from current CT traj to current
                // % CT. This is the best estimate that we can get of where
                // % the CT will be in the future.
                T_BBfixed_CT_future = T_BBfixed_CTtraj_future_des * T_CTtraj_CT;
//                object_loc = [T_BBfixed_Instr_EKF_x;
//                              T_BBfixed_Instr_EKF_y;
//                              T_BBfixed_Instr_EKF_z];
                // TODO: add Kalman here
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                m_dXYZPsi(3) = computeSweep(T_BBfixed_CT_future, objectXYZ);
                break;
            default:
                objectXYZ = m_BB_targetPos.matrix().col(3).segment(0,3);
                m_dXYZPsi(3) = computeSweep(m_BB_CT_curTipPos, objectXYZ);
                qCritical() << "Unknown EKFstate! Defaulting to EKF Off.";
                break;
            }

            m_dXYZPsi(0) = T_BBfixed_CT_des(0,3) - m_BB_CT_curTipPos(0,3);
            m_dXYZPsi(1) = T_BBfixed_CT_des(1,3) - m_BB_CT_curTipPos(1,3);
            m_dXYZPsi(2) = T_BBfixed_CT_des(2,3) - m_BB_CT_curTipPos(2,3);
        }
            break;
        // Mobile, IT, Imager ends
        default:
            qCritical() << "Unknown! Mobile, IT.";
            break;
        }

        break;
    default:
        break;
    }

    //printf("m_dXYZPsi - x %.3f y %.3f z %.3f psi %.3f\n", m_dXYZPsi(0), m_dXYZPsi(1), m_dXYZPsi(2), m_dXYZPsi(3)*deg180overPi);
    std::vector<double> dxyzpsi = {m_dXYZPsi(0), m_dXYZPsi(1), m_dXYZPsi(2), m_dXYZPsi(3)*deg180overPi};
    emit logData(QTime::currentTime(), m_numCycles, CONTROLLER_DXYZPSI, dxyzpsi);
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
