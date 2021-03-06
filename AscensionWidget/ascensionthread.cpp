#include "ascensionthread.h"

AscensionThread::AscensionThread(QObject *parent) :
    QObject(parent)
{
    m_isEpochSet = false;
    m_isReady = false;
    m_keepRecording = false;
    m_abort = false;

    m_mutex = new QMutex(QMutex::Recursive);

    m_numSensorsAttached = 0;
    m_records = 0;
    m_latestReading.resize(4);

    m_deltaT.resize(EM_DELTA_T_SIZE,0);
    m_avgSamplingFreq = 0.0;

    qRegisterMetaType<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD>("DOUBLE_POSITION_MATRIX_TIME_Q_RECORD");
    qRegisterMetaType< std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> >("std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD>");
}

AscensionThread::~AscensionThread()
{
    disconnectEM();
    m_mutex->lock();
    m_abort = true;
    qDebug() << "Ending AscensionThread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
    m_mutex->unlock();

    delete m_mutex;

    emit finished();
}

// ------------------------------
//     SLOTS IMPLEMENTATION
// ------------------------------
bool AscensionThread::initializeEM() // open connection to EM
{
    bool status = true;

    QMutexLocker locker(m_mutex);

    // Initialize the ATC3DG driver and DLL
    // It is always necessary to first initialize the ATC3DG "system". By
    // "system" we mean the set of ATC3DG cards installed in the PC. All cards
    // will be initialized by a single call to InitializeBIRDSystem(). This
    // call will first invoke a hardware reset of each board. If at any time
    // during operation of the system an unrecoverable error occurs then the
    // first course of action should be to attempt to Recall InitializeBIRDSystem()
    // if this doesn't restore normal operating conditions there is probably a
    // permanent failure - contact tech support.
    // A call to InitializeBIRDSystem() does not return any information.
    //
    emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_INITIALIZE_BEGIN);
    emit statusChanged(EM_INITIALIZE_BEGIN);

    m_errorCode = InitializeBIRDSystem();
    if(m_errorCode!=BIRD_ERROR_SUCCESS)
    {
        errorHandler_(m_errorCode);
        emit statusChanged(EM_INITIALIZE_FAILED);
        return status = false;
    }

    // GET SYSTEM CONFIGURATION
    //
    // In order to get information about the system we have to make a call to
    // GetBIRDSystemConfiguration(). This call will fill a fixed size structure
    // containing amongst other things the number of boards detected and the
    // number of sensors and transmitters the system can support (Note: This
    // does not mean that all sensors and transmitters that can be supported
    // are physically attached)
    //
    m_errorCode = GetBIRDSystemConfiguration(&m_ATC3DG.m_config);
    if(m_errorCode!=BIRD_ERROR_SUCCESS)
    {
        errorHandler_(m_errorCode);
        emit statusChanged(EM_INITIALIZE_FAILED);
        return status = false;
    }

    // GET SENSOR CONFIGURATION
    //
    // Having determined how many sensors can be supported we can dynamically
    // allocate storage for the information about each sensor.
    // This information is acquired through a call to GetSensorConfiguration()
    // This call will fill a fixed size structure containing amongst other things
    // a status which indicates whether a physical sensor is attached to this
    // sensor port or not.
    //
    m_pSensor = new CSensor[m_ATC3DG.m_config.numberSensors];
    for(m_i = 0; m_i < m_ATC3DG.m_config.numberSensors; m_i++)
    {
        m_errorCode = GetSensorConfiguration(m_i, &(m_pSensor + m_i)->m_config);
        if(m_errorCode!=BIRD_ERROR_SUCCESS)
        {
            errorHandler_(m_errorCode);
            emit statusChanged(EM_INITIALIZE_FAILED);
            return status = false;
        }
        // if this sensor is attached, set number of sensors to current index
        // !THIS ASSUMES THAT SENSORS ARE ATTACHED IN ORDER
        // !BEGINNING AT 1 ON THE BOX (AND 0 IN C++)
        if( (m_pSensor + m_i)->m_config.attached )
            m_numSensorsAttached = m_i + 1;
    }

    emit logEventWithMessage(SRC_EM, LOG_INFO, QTime::currentTime(), EM_SENSORS_DETECTED,
                             QString::number(m_numSensorsAttached));

    // GET TRANSMITTER CONFIGURATION
    //
    // The call to GetTransmitterConfiguration() performs a similar task to the
    // GetSensorConfiguration() call. It also returns a status in the filled
    // structure which indicates whether a transmitter is attached to this
    // port or not. In a single transmitter system it is only necessary to
    // find where that transmitter is in order to turn it on and use it.
    //
    m_pXmtr = new CXmtr[m_ATC3DG.m_config.numberTransmitters];
    for(m_i = 0; m_i < m_ATC3DG.m_config.numberTransmitters; m_i++)
    {
        m_errorCode = GetTransmitterConfiguration(m_i, &(m_pXmtr + m_i)->m_config);
        if(m_errorCode!=BIRD_ERROR_SUCCESS)
        {
            errorHandler_(m_errorCode);
            emit statusChanged(EM_INITIALIZE_FAILED);
            return status = false;
        }
    }

    // Search for the first attached transmitter and turn it on
    //
    for(m_id = 0; m_id < m_ATC3DG.m_config.numberTransmitters; m_id++)
    {
        if((m_pXmtr + m_id)->m_config.attached)
        {
            // Transmitter selection is a system function.
            // Using the SELECT_TRANSMITTER parameter we send the id of the
            // transmitter that we want to run with the SetSystemParameter() call
            m_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &m_id, sizeof(m_id));
            if(m_errorCode!=BIRD_ERROR_SUCCESS)
            {
                errorHandler_(m_errorCode);
                emit statusChanged(EM_INITIALIZE_FAILED);
                return status = false;
            }
            break;
        }
    }

    // report the transmitter being used

    emit logEventWithMessage(SRC_EM, LOG_INFO, QTime::currentTime(), EM_TRANSMITTER_SET,
                             QString::number(m_id));

    // report the detected sampling rate
    m_samplingFreq = m_ATC3DG.m_config.measurementRate;
    emit logEventWithMessage(SRC_EM, LOG_INFO, QTime::currentTime(), EM_FREQ_DETECTED,
                             QString::number(m_samplingFreq));

    // Set sensor output to position + rotation matrix + time stamp
    for(m_sensorID = 0; m_sensorID < m_numSensorsAttached; m_sensorID++)
    {
        DATA_FORMAT_TYPE buf = DOUBLE_POSITION_MATRIX_TIME_Q;
        DATA_FORMAT_TYPE *pBuf = &buf;
        m_errorCode = SetSensorParameter(m_sensorID, DATA_FORMAT, pBuf, sizeof(buf));
        if(m_errorCode!=BIRD_ERROR_SUCCESS) {
            errorHandler_(m_errorCode);
            emit statusChanged(EM_INITIALIZE_FAILED);
            return status = false;
        }
    }

    // resize container based on the number of sensors
    m_latestReading.resize(m_numSensorsAttached);

    // set flag to ready
    m_isReady = status;

    //locker.unlock();
    // set sample rate
    setSampleRate(EM_DEFAULT_SAMPLE_RATE);

    // set to METRIC
    BOOL isMetric = TRUE;
    m_errorCode = SetSystemParameter(METRIC, &isMetric, sizeof(isMetric));
    if(m_errorCode!=BIRD_ERROR_SUCCESS) {
        errorHandler_(m_errorCode);
        emit statusChanged(EM_INITIALIZE_FAILED);
        return status = false;
    }

    emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_INITIALIZED);
    emit statusChanged(EM_INITIALIZED);

    return status;
}

void AscensionThread::startAcquisition() // start timer
{
    QMutexLocker locker(m_mutex);

    // reset stats
    m_deltaT.clear();
    m_deltaT.resize(EM_DELTA_T_SIZE,0);
    m_avgSamplingFreq = 0;

    // start timer
    m_timer = new QTimer(this);
    connect(m_timer,SIGNAL(timeout()),this,SLOT(getSample()));
    m_timer->start(1000./m_samplingFreq);

    m_keepRecording = true;
    m_i = 0;

    emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_ACQUISITION_STARTED);
    emit statusChanged(EM_ACQUISITION_STARTED);
}

void AscensionThread::getSample() // called by timer
{
    // Collect data from all birds
    // Note: The default data format is DOUBLE_POSITION_ANGLES. We can use this
    // format without first setting it.

    // 4 sensors
    DOUBLE_POSITION_MATRIX_TIME_Q_RECORD record[8*4], *pRecord = record;
    //DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD *pRecord = record
    //DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD record, *pRecord = &record;

    QMutexLocker locker(m_mutex);

    // request all sensor readings at once
    m_errorCode = GetSynchronousRecord(ALL_SENSORS, pRecord, sizeof(record[0]) * m_numSensorsAttached);
    QTime tstamp = QTime::currentTime(); // get time
    if(m_errorCode != BIRD_ERROR_SUCCESS)
    {
        errorHandler_(m_errorCode);
        emit statusChanged(EM_ACQUIRE_FAILED);
        //locker.unlock();
        stopAcquisition();
        return;
    }

    // async version, not prefered, therefore commented out
    /*
    //    // scan the sensors and request a record - this is async, not preferred
    //    for(m_sensorID = 0; m_sensorID < m_numSensorsAttached; m_sensorID++)
    //    {
    //        m_errorCode = GetAsynchronousRecord(m_sensorID, &record[m_sensorID], sizeof(record[m_sensorID]));
    //        //m_errorCode = GetSynchronousRecord(m_sensorID, &record[m_sensorID], sizeof(record[m_sensorID]));
    //        if(m_errorCode != BIRD_ERROR_SUCCESS)
    //        {
    //            errorHandler_(m_errorCode);
    //            emit statusChanged(EM_ACQUIRE_FAILED);
    //            //locker.unlock();
    //            stopAcquisition();
    //            return;
    //        }
    //        // Weirdly, the vector part is reported with a flipped sign. Scalar part is fine.
    //        // You can check this in MATLAB
    //        // Use the Cubes.exe provided by Ascension to get the azimuth, elevation, and roll.
    //        // These are the Z,Y,X rotation angles.
    //        // You can use the Robotics Toolbox by P. Corke and run convert from quat to rpy
    //        // rad2deg(quat2rpy(q0,q1,q2,q3)) -> this gives you the X,Y,Z rotation angles (notice the flipped order!)
    //        // Flipping the sign of q1,q2,q3 will fix the mismatch
    //        record[m_sensorID].q[1] *= -1.0;
    //        record[m_sensorID].q[2] *= -1.0;
    //        record[m_sensorID].q[3] *= -1.0;
    //    }
    */

    // get time difference between readings
    // m_deltaT hold the latest index as the last element in the vector
    // this index is incremented to essentially create a circular buffer
    m_deltaT[m_deltaT[EM_DELTA_T_SIZE-1]] = record[0].time - m_latestReading[0].time;
    m_deltaT[EM_DELTA_T_SIZE-1] = ((int)(m_deltaT[EM_DELTA_T_SIZE-1] + 1))%(EM_DELTA_T_SIZE-1);
    // average the deltaT's
    m_avgSamplingFreq = std::accumulate(m_deltaT.begin(),m_deltaT.end()-1,0.0)/(EM_DELTA_T_SIZE-1);

    m_latestReading.clear();
    m_latestReading.resize(4);

    for(m_sensorID = 0; m_sensorID < m_numSensorsAttached; m_sensorID++)
    {
        // get the status of the last data record
        // only report the data if everything is okay
        unsigned int status = GetSensorStatus( m_sensorID );

        if( status == VALID_STATUS)
        {
            // The rotations are reported in inverted form.
            // You can check this in MATLAB
            // Use the Cubes.exe provided by Ascension to get the azimuth, elevation, and roll.
            // These are the Z,Y,X rotation angles.
            // You can use the Robotics Toolbox by P. Corke and run convert from quat to rpy
            // rad2deg(quat2rpy(q0,q1,q2,q3)) -> this gives you the X,Y,Z rotation angles (notice the flipped order!)
            // Flipping the sign of q1,q2,q3 will fix the mismatch

            // For a quaternion, this corresponds to the vector part being negated.
//            record[m_sensorID].q[1] *= -1.0;
//            record[m_sensorID].q[2] *= -1.0;
//            record[m_sensorID].q[3] *= -1.0;

            // transpose rotation matrix to invert it (R transpose = R inverse)
            double temp = record[m_sensorID].s[1][0];
            record[m_sensorID].s[1][0] = record[m_sensorID].s[0][1];
            record[m_sensorID].s[0][1] = temp;
            temp = record[m_sensorID].s[2][0];
            record[m_sensorID].s[2][0] = record[m_sensorID].s[0][2];
            record[m_sensorID].s[0][2] = temp;
            temp = record[m_sensorID].s[1][2];
            record[m_sensorID].s[1][2] = record[m_sensorID].s[2][1];
            record[m_sensorID].s[2][1] = temp;
            // transpose done

            m_latestReading[m_sensorID] = record[m_sensorID];
            emit logData(tstamp, m_sensorID, record[m_sensorID]);
            //emit logData(tstamp, m_sensorID, record);
            if( m_i == 0 ){
                //emit sendDataToGUI(m_sensorID, formatOutput(tstamp, m_sensorID, record));
                emit sendDataToGUI(m_sensorID, formatOutput(tstamp, m_sensorID, record[m_sensorID]));
            }
        }
        else
            qDebug() << "Invalid data received, sensorID:" << m_sensorID;
    }
//    if( m_i == 0 ){
//        std::printf("EM rate: %.3f ms\n", m_avgSamplingFreq*1000.0);
//    }
    m_i = (m_i + 1)%(int)(m_samplingFreq/10);

    // emit m_latestReading for use with the controller
    emit sendLatestReading(m_latestReading);
}

void AscensionThread::stopAcquisition() // stop timer
{
    QMutexLocker locker(m_mutex);

    if ( m_keepRecording )
    {
        m_keepRecording = false;
        m_timer->stop();

        disconnect(m_timer,SIGNAL(timeout()),0,0);

        delete m_timer;

        emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_ACQUISITION_STOPPED);
        emit statusChanged(EM_ACQUISITION_STOPPED);
    }
}

bool AscensionThread::disconnectEM() // disconnect from EM
{
    bool status = true;

    stopAcquisition();

    QMutexLocker locker(m_mutex);
    // Turn off the transmitter before exiting
    // We turn off the transmitter by "selecting" a transmitter with an id of "-1"
    if(m_id != -1)
    {
        m_id = -1;
        m_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &m_id, sizeof(m_id));
        if(m_errorCode != BIRD_ERROR_SUCCESS)
        {
            errorHandler_(m_errorCode);
            emit statusChanged(EM_DISCONNECT_FAILED);
            return status = false;
        }

        //  Free memory allocations before exiting
        delete[] m_pSensor;
        delete[] m_pXmtr;

        emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_DISCONNECTED);
        emit statusChanged(EM_DISCONNECTED);
    }
    return status;
}

void AscensionThread::setEpoch(const QDateTime &datetime) // set Epoch
{
    QMutexLocker locker(m_mutex);
    if(!m_keepRecording)
    {
        m_epoch = datetime;
        m_isEpochSet = true;

        emit logEventWithMessage(SRC_EM, LOG_INFO, QTime::currentTime(), EM_EPOCH_SET,
                                 m_epoch.toString("yyyy/MM/dd - hh:mm:ss.zzz"));
    }
    else
        emit logEvent(SRC_EM, LOG_INFO, QTime::currentTime(), EM_EPOCH_SET_FAILED);
}

void AscensionThread::setSampleRate(int freq) // set freq
{
    QMutexLocker locker(m_mutex);

    // not recording
    if(!m_keepRecording)
    {
        // freq too low, clamp
        if( EM_MIN_SAMPLE_RATE > freq )
        {
            freq = EM_MIN_SAMPLE_RATE;
            emit logError(SRC_EM, LOG_ERROR, QTime::currentTime(),
                          EM_BELOW_MIN_SAMPLE_RATE, QString::number(freq));
        }
        // freq too high, clamp
        else if( EM_MAX_SAMPLE_RATE < freq )
        {
            freq = EM_ABOVE_MAX_SAMPLE_RATE;
            emit logError(SRC_EM, LOG_ERROR, QTime::currentTime(),
                          EM_ABOVE_MAX_SAMPLE_RATE, QString::number(freq));
        }

        // set freq
        double rate = freq*1.0f;
        m_errorCode = SetSystemParameter(MEASUREMENT_RATE, &rate, sizeof(rate));
        if(m_errorCode!=BIRD_ERROR_SUCCESS)
        {
            errorHandler_(m_errorCode);
            emit statusChanged(EM_FREQ_SET_FAILED);
        }
        else
        {
            m_samplingFreq = freq;

            // log event
            emit logEventWithMessage(SRC_EM, LOG_INFO, QTime::currentTime(), EM_FREQ_SET,
                                     QString::number(m_samplingFreq));

            // emit status change
            emit statusChanged(EM_FREQ_SET);
        }
    }
    else
    {
        emit logError(SRC_EM, LOG_ERROR, QTime::currentTime(),
                      EM_CANT_MUTATE_WHILE_RUNNING, QString(""));
        emit statusChanged(EM_FREQ_SET_FAILED);
    }
}


// ----------------
//    ACCESSORS
// ----------------
int AscensionThread::getSampleRate()
{
    QMutexLocker locker(m_mutex);
    return m_samplingFreq;
}

int AscensionThread::getNumSensors()
{
    QMutexLocker locker(m_mutex);
    return m_numSensorsAttached;
}

void AscensionThread::getLatestReading(const int sensorID, DOUBLE_POSITION_MATRIX_TIME_Q_RECORD &dataContainer)
{
    QMutexLocker locker(m_mutex);
    if(sensorID < 0)
        qDebug() << "sensorID can not be negative!";
    else if(sensorID < m_numSensorsAttached)
        dataContainer = m_latestReading[sensorID];
    else
        qDebug() << "sensorID exceeds number of sensors!";
}

void AscensionThread::getLatestReadingsAll(std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> &dataContainer)
{
    QMutexLocker locker(m_mutex);
    dataContainer = m_latestReading;
}

// ----------------
// HELPER FUNCTIONS
// ----------------
void AscensionThread::errorHandler_(int error)
{
    char			buffer[1024];
    char			*pBuffer = &buffer[0];
    int				numberBytes;

    while(error!=BIRD_ERROR_SUCCESS)
    {
        QString msg(QString("Error Code %1: ").arg(QString::number(error)));
        error = GetErrorText(error, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);
        numberBytes = strlen(buffer);
        //buffer[numberBytes] = '\n';		// append a newline to buffer
        //printf("%s", buffer);
        msg.append(buffer);
        qDebug() << msg;
        emit logError(SRC_EM, LOG_ERROR, QTime::currentTime(), EM_FAIL, msg);
    }
}

QString AscensionThread::formatOutput(QTime &timeStamp, const int sensorID, DOUBLE_POSITION_MATRIX_TIME_Q_RECORD &data)
{
    QDateTime ts;
    ts.setMSecsSinceEpoch(data.time*1000);

    QString output;
    output.append(QString("[Sensor %1 - %2] - ").arg(sensorID + 1).arg(EM_SENSOR_NAMES[sensorID]));
    output.append(timeStamp.toString("HH.mm.ss.zzz\n"));
//    output.append(QString("%1\t%2\t%3\n")
//                        .arg(QString::number(data.x,'f',m_prec))
//                        .arg(QString::number(data.y,'f',m_prec))
//                        .arg(QString::number(data.z,'f',m_prec)));
    // TODO: comment out the quaternion
//    output.append("Rot (Quat)\n");
//    output.append(QString("%1\t%2\t%3\n%4\n")
//                        .arg(QString::number(data.q[0],'f',m_prec))
//                        .arg(QString::number(data.q[1],'f',m_prec))
//                        .arg(QString::number(data.q[2],'f',m_prec))
//                        .arg(QString::number(data.q[3],'f',m_prec)));

//    QQuaternion qu = QQuaternion(data.q[0], data.q[1], data.q[2], data.q[3]);
//    QMatrix3x3 mat = qu.toRotationMatrix();

    output.append(QString("%1   %2   %3   %4\n%5   %6   %7   %8\n%9   %10   %11   %12\n")
                        .arg(QString::number(data.s[0][0],'f',m_prec))
                        .arg(QString::number(data.s[0][1],'f',m_prec))
                        .arg(QString::number(data.s[0][2],'f',m_prec))
                        .arg(QString::number(data.x      ,'f',m_prec))
                        .arg(QString::number(data.s[1][0],'f',m_prec))
                        .arg(QString::number(data.s[1][1],'f',m_prec))
                        .arg(QString::number(data.s[1][2],'f',m_prec))
                        .arg(QString::number(data.y      ,'f',m_prec))
                        .arg(QString::number(data.s[2][0],'f',m_prec))
                        .arg(QString::number(data.s[2][1],'f',m_prec))
                        .arg(QString::number(data.s[2][2],'f',m_prec))
                        .arg(QString::number(data.z      ,'f',m_prec)));

    output.append("Quality: ");
    output.append(QString("%1\n")
                        .arg(QString::number(data.quality,'f',1)));
    output.append(QString("Time: %1").arg(ts.toString("hh:mm:ss.zzz\n")));

    return output;
}

inline const QString getCurrTimeStr()
{
    return QTime::currentTime().toString("HH.mm.ss.zzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("yyyy/MM/dd - hh:mm:ss.zzz");
}
