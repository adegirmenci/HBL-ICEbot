#include "labjackthread.h"

LabJackThread::LabJackThread(QObject *parent) : QObject(parent)
{
    qRegisterMetaType< QVector<ushort> >("QVector<ushort>");
    qRegisterMetaType< QVector<QString> >("QVector<QString>");
    qRegisterMetaType< std::vector<double> >("std::vector<double>");


    m_isEpochSet = false;
    m_isReady = false;
    m_keepAcquiring = false;
    m_abort = false;

    m_mutex = new QMutex(QMutex::Recursive);

    m_lngErrorcode = 0;
    m_lngHandle = 0;
    m_i = 0;
    m_k = 0;
    m_lngIOType = 0;
    m_lngChannel = 0;
    m_dblValue = 0;
    m_dblCommBacklog = 0;
    m_dblUDBacklog = 0;
    m_numChannels = 1;
    m_channelNames.push_back(tr("ECG"));
    m_scanRate = 1000; //scan rate = sample rate / #channels
    m_delayms = 1000; // 1 seconds per 1000 samples
    m_numScans = 2*m_scanRate*m_delayms/1000;  //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    //m_adblData = (double *)calloc( m_numScans, sizeof(double)); //Max buffer size (#channels*numScansRequested)
    //m_padblData = (long)&m_adblData[0];

}

LabJackThread::~LabJackThread()
{
    disconnectLabJack();

    m_mutex->lock();
    m_abort = true;
    qDebug() << "Ending LabJackThread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
    m_mutex->unlock();

    delete m_mutex;

    emit finished();
}

void LabJackThread::connectLabJack()
{
    QMutexLocker locker(m_mutex); // lock mutex

    bool success = true;

    emit statusChanged(LABJACK_CONNECT_BEGIN);

    //Open the first found LabJack U6.
    m_lngErrorcode = OpenLabJack(LJ_dtU6, LJ_ctUSB, "1", 1, &m_lngHandle);
    success = ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Read and display the hardware version of this U6.
    m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_CONFIG, LJ_chHARDWARE_VERSION, &m_dblValue, 0);
    success = success && ErrorHandler(m_lngErrorcode, __LINE__, 0);
    emit logEventWithMessage(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_CONNECTED,
                             QString("U6 Hardware Version = %1\n")
                             .arg(QString::number(m_dblValue,'f',3))          );
    qDebug() << "U6 Hardware Version = " << m_dblValue;

    //Read and display the firmware version of this U6.
    m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_CONFIG, LJ_chFIRMWARE_VERSION, &m_dblValue, 0);
    success = success && ErrorHandler(m_lngErrorcode, __LINE__, 0);
    emit logEventWithMessage(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_CONNECTED,
                             QString("U6 Firmware Version = %1\n")
                             .arg(QString::number(m_dblValue,'f',3))          );
    qDebug() << "U6 Firmware Version = " << m_dblValue;

    if(success)
        emit statusChanged(LABJACK_CONNECTED);
    else
        emit statusChanged(LABJACK_CONNECT_FAILED);
        //ResetLabJack(m_lngHandle);

}

void LabJackThread::initializeLabJack(const uint samplesPerSec, const QVector<ushort> channelIdx, const QVector<QString> channelNames)
{
    QMutexLocker locker(m_mutex);

    emit statusChanged(LABJACK_INITIALIZE_BEGIN);

    m_scanRate = samplesPerSec;
    qDebug() << "LabJack samples per sec: " << m_scanRate;

    m_numChannels = channelIdx.size();
    if(m_numChannels < 1)
        qDebug() << "LabJack should have at least one channel enabled!";
    m_channelNames = channelNames;

    m_numScans = 2*m_scanRate*m_delayms/1000;  //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    m_adblData = (double *)calloc( m_numChannels*m_numScans, sizeof(double)); //Max buffer size (#channels*numScansRequested)
    m_padblData = (long)&m_adblData[0];

    bool success = true;

    //Configure the stream:

    //Configure resolution of the analog inputs (pass a non-zero value for quick sampling).
    //See section 2.6 / 3.1 for more information.
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 0, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    // Configure the analog input range on channel 0 for bipolar + -10 volts.
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_AIN_RANGE, 0, LJ_rgBIP10V, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    // Configure the analog input resolution on channel 0 to index 3, 17bit resolution, 0.08 ms/sample
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 3, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

//    //Set the scan rate.
//    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, m_scanRate, 0, 0);
//    ErrorHandler(m_lngErrorcode, __LINE__, 0);

//    //Give the driver a 5 second buffer (scanRate * 1 channels * 5 seconds).
//    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, m_scanRate * 1 * 5, 0, 0);
//    ErrorHandler(m_lngErrorcode, __LINE__, 0);

//    //Configure reads to retrieve whatever data is available without waiting (wait mode LJ_swNONE).
//    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
//    ErrorHandler(m_lngErrorcode, __LINE__, 0);

//    //Define the scan list as AIN0.
//    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
//    ErrorHandler(m_lngErrorcode, __LINE__, 0);
//    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0); // first method for single ended reading - AIN0
//    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Request AIN0.
    m_lngErrorcode = AddRequest (m_lngHandle, LJ_ioGET_AIN, 2, 0, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Execute the list of requests.
    m_lngErrorcode = GoOne(m_lngHandle);
    success = ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Get all the results just to check for errors.
    m_lngErrorcode = GetFirstResult(m_lngHandle, &m_lngIOType, &m_lngChannel, &m_dblValue, 0, 0);
    success = success && ErrorHandler(m_lngErrorcode, __LINE__, 0);

    m_lngGetNextIteration = 0;	//Used by the error handling function.
    while (m_lngErrorcode < LJE_MIN_GROUP_ERROR)
    {
        m_lngErrorcode = GetNextResult(m_lngHandle, &m_lngIOType, &m_lngChannel, &m_dblValue, 0, 0);
        if (m_lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
        {
            success = success && ErrorHandler(m_lngErrorcode, __LINE__, m_lngGetNextIteration);
        }

        m_lngGetNextIteration++;
    }

    m_isReady = success;

    if(success)
        emit statusChanged(LABJACK_INITIALIZED);
    else
        emit statusChanged(LABJACK_INITIALIZE_FAILED);
}

void LabJackThread::startAcquisition()
{
    QMutexLocker locker(m_mutex);

    if(m_isReady && !m_keepAcquiring) // ready to read data
    {
        //Start the stream.
//        m_lngErrorcode = eGet(m_lngHandle, LJ_ioSTART_STREAM, 0, &m_dblValue, 0);
//        ErrorHandler(m_lngErrorcode, __LINE__, 0);

        m_keepAcquiring = true;

        m_timer = new QTimer(this);
        connect(m_timer, SIGNAL(timeout()), this, SLOT(ReadStream()));

        //m_timer->start(m_delayms);
        m_timer->start(1);

        if(m_timer->isActive())
        {
            qDebug() << "Timer started.";
            emit statusChanged(LABJACK_LOOP_STARTED);
            emit logEvent(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_LOOP_STARTED);
        }
        else
        {
            qDebug() << "Timer is not active.";
            emit statusChanged(LABJACK_LOOP_STOPPED);
            emit logError(SRC_LABJACK, LOG_ERROR, QTime::currentTime(), LJE_TIMER_INVALID_MODE, QString("Timer is not active."));
        }
    }
    else
    {
        qDebug() << "LabJack is not ready.";
        emit statusChanged(LABJACK_INITIALIZE_FAILED);
        emit logError(SRC_LABJACK, LOG_ERROR, QTime::currentTime(), LJE_DEVICE_NOT_OPEN, QString("LabJack is not ready."));
    }
}

void LabJackThread::stopAcquisition()
{
    QMutexLocker locker(m_mutex);

    if(m_keepAcquiring)
    {
        m_keepAcquiring = false;

        m_timer->stop();

        disconnect(m_timer, SIGNAL(timeout()), 0, 0);

        delete m_timer;

        emit logEvent(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_LOOP_STOPPED);
        emit statusChanged(LABJACK_LOOP_STOPPED);

//        m_lngErrorcode = eGet(m_lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
//        ErrorHandler(m_lngErrorcode, __LINE__, 0);

        qDebug() << "Timer stopped.";
    }
    else
    {
        qDebug() << "Timer already stopped.";
    }
}

void LabJackThread::disconnectLabJack()
{
    QMutexLocker locker(m_mutex);

    stopAcquisition();

    if(m_lngHandle != 0)
    {// Reset LabJack
        //m_lngErrorcode = ResetLabJack(m_lngHandle);
        //ErrorHandler(m_lngErrorcode, __LINE__, 0);
    }
    emit statusChanged(LABJACK_DISCONNECTED);
}

void LabJackThread::setEpoch(const QDateTime &epoch)
{
    QMutexLocker locker(m_mutex);

    if(!m_keepAcquiring)
    {
        m_epoch = epoch;
        m_isEpochSet = true;

        emit logEventWithMessage(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_EPOCH_SET,
                                 m_epoch.toString("yyyy/MM/dd - hh:mm:ss.zzz"));
    }
    else
        emit logEvent(SRC_LABJACK, LOG_INFO, QTime::currentTime(), LABJACK_EPOCH_SET_FAILED);
}

void LabJackThread::ReadStream()
{
    QMutexLocker locker(m_mutex);

    if(m_isReady)
    {
//        for (m_k = 0; m_k < m_numScans; m_k++)
//        {
//            m_adblData[m_k] = 9999.0;
//        }

        //Read the data.  We will request twice the number we expect, to
        //make sure we get everything that is available.
        //Note that the array we pass must be sized to hold enough SAMPLES, and
        //the Value we pass specifies the number of SCANS to read.
//        m_numScansRequested = m_numScans;
//        m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS,
//                              &m_numScansRequested, m_padblData);
        std::vector<double> newData;
        newData.resize(m_numChannels, 99.0);
        m_lngErrorcode = eGet (m_lngHandle, LJ_ioGET_AIN, 0, &newData[0], 0); // single read
        ErrorHandler(m_lngErrorcode, __LINE__, 0);

//        emit logData(QTime::currentTime(), m_adblData[0]);
        if( (m_i % 1000) < 40)
            newData[0] = 3;
        else
            newData[0] = 0;
        m_i++;

        emit logData(QDateTime::currentMSecsSinceEpoch(), newData);
    }
}

bool LabJackThread::ErrorHandler(LJ_ERROR lngErrorcode,
                                 long lngLineNumber,
                                 long lngIteration)
{
    char err[255];

    if (lngErrorcode != LJE_NOERROR)
    {
        ErrorToString(lngErrorcode, err);

        QString msg = QString("Error string = ").append(err);
        msg.append(QString(" Source line number = %1. ").arg(lngLineNumber));
        msg.append(QString("Iteration = %1").arg(lngIteration));

        if (lngErrorcode > LJE_MIN_GROUP_ERROR)
        {
            qDebug() << "FATAL ERROR!" << msg;
            logError(SRC_LABJACK, LOG_FATAL,
                          QTime::currentTime(),
                          lngErrorcode,
                          msg);
        }
        else
        {
            qDebug() << msg;
            logError(SRC_LABJACK, LOG_ERROR, QTime::currentTime(), lngErrorcode,
                          msg);
        }
        return false; // there was an error
    }
    else
        return true; // no errors - success
}
