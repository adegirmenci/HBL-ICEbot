#include "controllerthread.h"

ControllerThread::ControllerThread(QObject *parent) :
    QObject(parent)
{
    m_isEpochSet = false;
    m_isReady = false;
    m_keepControlling = false;
    m_abort = false;
    m_latestReading.resize(4);

    m_numCycles = 0;

    m_cathKin = Kinematics_4DOF(0.05*1000.0, 0.00135*1000.0, 0.90*0.0254*1000.0);

    loadConstants();

    m_targetPos = m_targetPos.Identity();

    m_mutex = new QMutex(QMutex::Recursive);

    m_isReady = true;

    emit statusChanged(CONTROLLER_INITIALIZED);
}

ControllerThread::~ControllerThread()
{
    std::cout << "FwdKin\n" << m_cathKin.forwardKinematics(1,0.1,0.1,0).matrix() << std::endl;

    std::cout << "Jacobian\n" << m_cathKin.JacobianNumeric(1,0.1,0.1,0).matrix() << std::endl;

    std::cout << "InvKin3D\n" << m_cathKin.inverseKinematics3D(5,5,12,0.1) << std::endl;

    Eigen::Transform<double, 3, Eigen::Affine> T_in(Eigen::Matrix<double, 4, 4>::Identity());
    std::cout << "T_in\n" << T_in.matrix() << std::endl;
    Eigen::Vector4d configIn(0.0010, 0.0001, 0.0001, 0);
    std::cout << "control_icra2016\n" << m_cathKin.control_icra2016(T_in, configIn, 0.0) << std::endl;

    qDebug() << "Ending ControllerThread - ID: " << QThread::currentThreadId() << ".";

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
    qDebug() << QTime::currentTime() << "Worker Thread ID: " << QThread::currentThreadId();
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
    m_BBmobile_CT = m_Box_BBmobile.inverse()*m_curTipPos*m_STm_BT*m_BT_CT;

    //Calculate T_BB_CT_curTipPos:
    // process CT point
    m_BB_CT_curTipPos = m_BB_Box*m_curTipPos*m_STm_BT*m_BT_CT; // convert to CT in terms of BBfixed

    //Calculate T_BB_targetPos
    m_targetPos = m_targetPos * m_ISm_INSTR; // the tip of the instrument in EM coord
    m_BB_targetPos = m_BB_Box * m_targetPos; // the tip of the instr in BBfixed coord

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

//    qint64 elNsec = elTimer.nsecsElapsed();
//    qDebug() << "Nsec elapsed:" << elNsec;

    controlCycle();
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
        m_targetPos = tempT;
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
        m_deltaXYZPsiToTarget(0) = x - m_BBfixed_CTorig(0,3);
        m_deltaXYZPsiToTarget(1) = y - m_BBfixed_CTorig(1,3);
        m_deltaXYZPsiToTarget(2) = z - m_BBfixed_CTorig(2,3);
    }
    else
    {
        m_deltaXYZPsiToTarget(0) = x;
        m_deltaXYZPsiToTarget(1) = y;
        m_deltaXYZPsiToTarget(2) = z;
    }
    m_deltaXYZPsiToTarget(3) = delPsi;

    // check limits

    // emit event to DataLogger
    std::cout << "New deltaXYZPsi : " << m_deltaXYZPsiToTarget << std::endl;
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

void ControllerThread::controlCycle()
{
    QMutexLocker locker(m_mutex);

    if(m_isReady && m_keepControlling)
    {
        // ########################################
        //
        // TODO: calculate distance and angle error
        //
        // ########################################

        // Calculate the angle between the new x-axis and the original x-axis
        Eigen::Transform<double,3,Eigen::Affine> T_CTorig_CT = m_BBfixed_CTorig.inverse()*m_BB_CT_curTipPos;
        // This is the total amount of psy that has occurred at the tip since we started
        double total_psy = atan2(T_CTorig_CT(1,0), T_CTorig_CT(0,0));

//        double normalization = pow(T_CTorig_CT(0,0),2) + pow(T_CTorig_CT(1,0),2);
//        double total_psy2 = atan2(T_CTorig_CT(1,0)/normalization, T_CTorig_CT(0,0)/normalization);

//        std::cout << "Psy 1 : " << total_psy << "Psy 2 : " << total_psy2 << std::endl;

        // Existing roll in the catheter handle
        // Assuming the BB point has rotated about its Z axis, the amount of roll in
        // the handle is calculated as the angle of rotation about the base z-axis.
        // Here we calculate the angle between the new x-axis and the original
        // x-axis.
        double currGamma = atan2(m_BBfixed_BBmobile(1,0), m_BBfixed_BBmobile(0,0));

        std::cout << "Psy : " << total_psy * deg180overPi << " Gamma : " << currGamma * deg180overPi << std::endl;

        // Get BT w.r.t. mobile BB, rather than fixed BB
        Eigen::Transform<double,3,Eigen::Affine> T_BBmob_BT = m_BBfixed_BBmobile.inverse() * m_BB_CT_curTipPos * m_BT_CT.inverse();

        // How much does the cath still need to move?
        // calculate delta x,y,z,psi
        Eigen::Vector4d dXYZPsi;
        dXYZPsi(0) = m_deltaXYZPsiToTarget(0) - (m_BB_CT_curTipPos(0,3) - m_BBfixed_CTorig(0,3));
        dXYZPsi(1) = m_deltaXYZPsiToTarget(1) - (m_BB_CT_curTipPos(1,3) - m_BBfixed_CTorig(1,3));
        dXYZPsi(2) = m_deltaXYZPsiToTarget(2) - (m_BB_CT_curTipPos(2,3) - m_BBfixed_CTorig(2,3));
        dXYZPsi(3) = m_deltaXYZPsiToTarget(3) - total_psy;

        std::cout << "dx : " << dXYZPsi(0) << " dy : " << dXYZPsi(1) << " dz : " << dXYZPsi(2) << " dpsi : " << dXYZPsi(3) * deg180overPi << std::endl;

        // Calculate distance from target
        double distError = dXYZPsi.segment(0,3).norm();
        double angleError = dXYZPsi(3);

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

        std::cout << "Gains - kR: " << kRoll << " kP: " << kPitch << " kY: " << kYaw << " kT: " << kTrans << std::endl;


//        Eigen::Transform<double,3,Eigen::Affine> errorT;
//        errorT = m_targetPos - m_curTipPos;

        // calculate current gamma based on m_curTipPos and m_BB_SBm
        //double currGamma = 0.0;

        // feed into control_icra2016
        Eigen::Matrix<double, 4, 2> jointsCurrAndTarget;
        jointsCurrAndTarget = m_cathKin.control_icra2016(m_BB_CT_curTipPos, dXYZPsi, currGamma);

        Eigen::Vector4d relQCs;  // knob_tgt - knob_curr
        relQCs(0) = (jointsCurrAndTarget(0,1) - jointsCurrAndTarget(0,0)) * kTrans * 0.001 * EPOS_TRANS_RAD2QC;
        relQCs(1) = (jointsCurrAndTarget(1,1) - jointsCurrAndTarget(1,0)) * kPitch * EPOS_PITCH_RAD2QC;
        relQCs(2) = (jointsCurrAndTarget(2,1) - jointsCurrAndTarget(2,0)) * kYaw * EPOS_YAW_RAD2QC;
        relQCs(3) = (jointsCurrAndTarget(3,1) - jointsCurrAndTarget(3,0)) * kRoll * EPOS_ROLL_RAD2QC;

        // motor commands
        //std::cout << "QCs - T: " << relQCs(0) << " P: " << relQCs(1) << " Y: " << relQCs(2) << " R: " << relQCs(3) << std::endl;

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
        // TODO : velocity is QCs/s, so we should convert this velocity to QCs/control cycle (= Ascension time = ~6ms)
        if(abs(targetPos[0]) > EPOS_VELOCITY[0])
            targetPos[0] = boost::math::copysign(EPOS_VELOCITY[0], targetPos[0]);
        if(abs(targetPos[1]) > EPOS_VELOCITY[1])
            targetPos[1] = boost::math::copysign(EPOS_VELOCITY[1], targetPos[1]);
        if(abs(targetPos[2]) > EPOS_VELOCITY[2])
            targetPos[2] = boost::math::copysign(EPOS_VELOCITY[2], targetPos[2]);
        if(abs(targetPos[3]) > EPOS_VELOCITY[3])
            targetPos[3] = boost::math::copysign(EPOS_VELOCITY[3], targetPos[3]);

        std::cout << "QCs - T: " << targetPos[0] << " P: " << targetPos[1] << " Y: " << targetPos[2] << " R: " << targetPos[3] << std::endl;

        // TODO : check if the EPOS Servo Loop is active, if not, don't send commands
        emit setEPOSservoTargetPos(targetPos, false); //relative

        // emit logData(QTime::currentTime(), newData);

        qDebug() << QTime::currentTime() << "Cycle:" << m_numCycles;

        m_numCycles++;
    }
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

std::shared_ptr< std::vector<double> > ControllerThread::cycle_recalculate(const std::vector<double> &inputs)
{
    //    std::ifstream is("inputs.txt");
    //    std::istream_iterator<double> start(is), end;
    //    std::vector<double> inputs(start, end);
    //    qDebug() << "Read " << inputs.size() << " numbers";

    //    std::shared_ptr< std::vector<double> > outputs = cycle_recalculate(inputs);

    //    std::ofstream output_file("outputs.txt");
    //    std::ostream_iterator<double> output_iterator(output_file, "\n");
    //    std::copy(outputs->begin(), outputs->end(), output_iterator);

//    QElapsedTimer elTimer;
//    elTimer.start();

    // TODO: add error checking to ensure that the vector size is correct

    std::vector<double>::const_iterator iter = inputs.begin();

    // get all of the constants
    int m = static_cast<int>(*iter); ++iter; // m = number of sinusoid components
    double delta_t = *iter; ++iter; // Time step for each collected data point
    int N_initpts = static_cast<int>(*iter); ++iter;  // number of initialization points
    int edge_effect = static_cast<int>(*iter); ++iter;
    double omega_0 = *iter; ++iter;

    // calculate things from inputs
    int num_states = m * 2 + 2;
    int N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    Eigen::Map<Eigen::VectorXd> z_init_x(iter._Ptr, N_filtered); std::advance(iter, N_filtered); // Initialize with the previously lowpass filtered data
    Eigen::Map<Eigen::VectorXd> z_init_y(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_z(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_xaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_yaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_zaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_angle(iter._Ptr, N_filtered); std::advance(iter, N_filtered);

    // allocate zero matrices
    Eigen::MatrixXd A_init_x(N_filtered, num_states - 1); A_init_x.setZero(); A_init_x.col(0).setOnes();
    Eigen::MatrixXd A_init_y(N_filtered, num_states - 1); A_init_y.setZero(); A_init_y.col(0).setOnes();
    Eigen::MatrixXd A_init_z(N_filtered, num_states - 1); A_init_z.setZero(); A_init_z.col(0).setOnes();
    Eigen::MatrixXd A_init_xaxis(N_filtered, num_states - 1); A_init_xaxis.setZero(); A_init_xaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_yaxis(N_filtered, num_states - 1); A_init_yaxis.setZero(); A_init_yaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_zaxis(N_filtered, num_states - 1); A_init_zaxis.setZero(); A_init_zaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_angle(N_filtered, num_states - 1); A_init_angle.setZero(); A_init_angle.col(0).setOnes();

    for (int i = 0; i < N_filtered; i++)
    {
        for (int j = 1; j <= m; j++)
        {
            A_init_x(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_y(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_z(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_xaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_yaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_zaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_angle(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_x(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_y(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_z(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_xaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_yaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_zaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_angle(i, j + m)	= cos(j*omega_0*i*delta_t);
        }
    }

    // Step 2. Use least squares estimate to solve for x.
    //VectorXd x_init_x = pseudoInverse(A_init_x) * z_init_x;
    Eigen::VectorXd x_init_x = A_init_x.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_x);
    Eigen::VectorXd x_init_y = A_init_y.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_y);
    Eigen::VectorXd x_init_z = A_init_z.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_z);
    Eigen::VectorXd x_init_xaxis = A_init_xaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_xaxis);
    Eigen::VectorXd x_init_yaxis = A_init_yaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_yaxis);
    Eigen::VectorXd x_init_zaxis = A_init_zaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_zaxis);
    Eigen::VectorXd x_init_angle = A_init_angle.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_angle);

    //VectorXd x_rect_x = x_init_x;
    //VectorXd x_rect_y = x_init_y;
    //VectorXd x_rect_z = x_init_z;
    //VectorXd x_rect_xaxis = x_init_xaxis;
    //VectorXd x_rect_yaxis = x_init_yaxis;
    //VectorXd x_rect_zaxis = x_init_zaxis;
    //VectorXd x_rect_angle = x_init_angle;

    // Step 3. Convert from rectangular into polar and assemble the state
    // vector, x, at k = 430 (which is the last trustworthy state we can know)
    // State vector, x
    // x = [c; r(1:4); omega; theta(1:4)];

    Eigen::VectorXd x_polar_x(num_states); x_polar_x.setZero();
    Eigen::VectorXd x_polar_y(num_states); x_polar_y.setZero();
    Eigen::VectorXd x_polar_z(num_states); x_polar_z.setZero();
    Eigen::VectorXd x_polar_xaxis(num_states); x_polar_xaxis.setZero();
    Eigen::VectorXd x_polar_yaxis(num_states); x_polar_yaxis.setZero();
    Eigen::VectorXd x_polar_zaxis(num_states); x_polar_zaxis.setZero();
    Eigen::VectorXd x_polar_angle(num_states); x_polar_angle.setZero();
    x_polar_x(0) = x_init_x(0); // c(dc offset)
    x_polar_y(0) = x_init_y(0); // c(dc offset)
    x_polar_z(0) = x_init_z(0); // c(dc offset)
    x_polar_xaxis(0) = x_init_xaxis(0); // c(dc offset)
    x_polar_yaxis(0) = x_init_yaxis(0); // c(dc offset)
    x_polar_zaxis(0) = x_init_zaxis(0); // c(dc offset)
    x_polar_angle(0) = x_init_angle(0); // c(dc offset)

    for (size_t i = 1; i <= m; i++)
    {
        x_polar_x(i) = std::sqrt(std::pow(x_init_x(i), 2) + std::pow(x_init_x(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_y(i) = std::sqrt(std::pow(x_init_y(i), 2) + std::pow(x_init_y(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_z(i) = std::sqrt(std::pow(x_init_z(i), 2) + std::pow(x_init_z(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_xaxis(i) = std::sqrt(std::pow(x_init_xaxis(i), 2) + std::pow(x_init_xaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_yaxis(i) = std::sqrt(std::pow(x_init_yaxis(i), 2) + std::pow(x_init_yaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_zaxis(i) = std::sqrt(std::pow(x_init_zaxis(i), 2) + std::pow(x_init_zaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_angle(i) = std::sqrt(std::pow(x_init_angle(i), 2) + std::pow(x_init_angle(i + m), 2)); // r_i, convert rectangular coords back to polar
    }

    x_polar_x(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_y(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_z(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_xaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_yaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_zaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_angle(m + 1) = omega_0; // omega_0, (rad / sec)

    for (int i = 0; i < m; i++)
    {
        x_polar_x(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_x(i + m + 1), x_init_x(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_y(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_y(i + m + 1), x_init_y(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_z(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_z(i + m + 1), x_init_z(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_xaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_xaxis(i + m + 1), x_init_xaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_yaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_yaxis(i + m + 1), x_init_yaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_zaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_zaxis(i + m + 1), x_init_zaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_angle(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_angle(i + m + 1), x_init_angle(i + 1)); // theta = i*omega*T + phi, (rad)
    }


    // return output
    std::shared_ptr< std::vector<double> > outputs(std::make_shared< std::vector<double> >());

    //outputs->insert(outputs->end(), z_init_x.data(), z_init_x.data() + z_init_x.size());
    //outputs->insert(outputs->end(), z_init_y.data(), z_init_y.data() + z_init_y.size());
    //outputs->insert(outputs->end(), z_init_z.data(), z_init_z.data() + z_init_z.size());
    //outputs->insert(outputs->end(), z_init_xaxis.data(), z_init_xaxis.data() + z_init_xaxis.size());
    //outputs->insert(outputs->end(), z_init_yaxis.data(), z_init_yaxis.data() + z_init_yaxis.size());
    //outputs->insert(outputs->end(), z_init_zaxis.data(), z_init_zaxis.data() + z_init_zaxis.size());
    //outputs->insert(outputs->end(), z_init_angle.data(), z_init_angle.data() + z_init_angle.size());

    outputs->insert(outputs->end(), x_init_x.data(), x_init_x.data() + x_init_x.size());
    outputs->insert(outputs->end(), x_init_y.data(), x_init_y.data() + x_init_y.size());
    outputs->insert(outputs->end(), x_init_z.data(), x_init_z.data() + x_init_z.size());
    outputs->insert(outputs->end(), x_init_xaxis.data(), x_init_xaxis.data() + x_init_xaxis.size());
    outputs->insert(outputs->end(), x_init_yaxis.data(), x_init_yaxis.data() + x_init_yaxis.size());
    outputs->insert(outputs->end(), x_init_zaxis.data(), x_init_zaxis.data() + x_init_zaxis.size());
    outputs->insert(outputs->end(), x_init_angle.data(), x_init_angle.data() + x_init_angle.size());
    outputs->insert(outputs->end(), x_polar_x.data(), x_polar_x.data() + x_polar_x.size());
    outputs->insert(outputs->end(), x_polar_y.data(), x_polar_y.data() + x_polar_y.size());
    outputs->insert(outputs->end(), x_polar_z.data(), x_polar_z.data() + x_polar_z.size());
    outputs->insert(outputs->end(), x_polar_xaxis.data(), x_polar_xaxis.data() + x_polar_xaxis.size());
    outputs->insert(outputs->end(), x_polar_yaxis.data(), x_polar_yaxis.data() + x_polar_yaxis.size());
    outputs->insert(outputs->end(), x_polar_zaxis.data(), x_polar_zaxis.data() + x_polar_zaxis.size());
    outputs->insert(outputs->end(), x_polar_angle.data(), x_polar_angle.data() + x_polar_angle.size());

//    qint64 elNsec = elTimer.nsecsElapsed();

//    qDebug() << "Nsec elapsed:" << elNsec;

    return outputs;
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
