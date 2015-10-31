#include "labjack.h"
#include "ui_labjack.h"

// ------------------------------------------- \\
// -------------- LabJackWorker -------------- \\
// ------------------------------------------- \\

LabJackWorker::LabJackWorker()
{
    // default samples per second is 1000
    LabJackWorker(1000);
}

LabJackWorker::LabJackWorker(int samplesPsec)
{
    QMutexLocker locker(&m_mutex); // lock mutex

    m_lngErrorcode = 0;
    m_lngHandle = 0;
    m_i = 0;
    m_k = 0;
    m_lngIOType = 0;
    m_lngChannel = 0;
    m_dblValue = 0;
    m_dblCommBacklog = 0;
    m_dblUDBacklog = 0;
    m_scanRate = samplesPsec; //scan rate = sample rate / #channels
    m_delayms = 1000; // 1 seconds per 1000 samples
    m_numScans = 2*m_scanRate*m_delayms/1000;  //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    m_adblData = (double *)calloc( m_numScans, sizeof(double)); //Max buffer size (#channels*numScansRequested)
    m_padblData = (long)&m_adblData[0];

    m_LabJackReady = false;

    //m_keepRecording = false; // flag for the while loop

    m_eventTimer = new QTimer(this->thread());
    connect(m_eventTimer, SIGNAL(timeout()), this, SLOT(ReadStream()));

    // mutex unlocks when locker goes out of scope
}

LabJackWorker::~LabJackWorker()
{
    qDebug() << "[Destroy LabJack Worker] Disconnect From Labjack" <<
    DisconnectFromLabJack();

    qDebug() << "[Destroy LabJack Worker] DeleteLater m_eventTimer";
    if(!m_eventTimer.isNull())
        delete m_eventTimer;//->deleteLater();

    qDebug() << "[Destroy LabJack Worker] Free m_adblData";
    free( m_adblData );
}

bool LabJackWorker::ConnectToLabJack()
{
    bool success = false;

    QMutexLocker locker(&m_mutex); // lock mutex

    //Open the first found LabJack U6.
    m_lngErrorcode = OpenLabJack(LJ_dtU6, LJ_ctUSB, "1", 1, &m_lngHandle);
    success = ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Read and display the hardware version of this U6.
    m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_CONFIG, LJ_chHARDWARE_VERSION, &m_dblValue, 0);
    printf("U6 Hardware Version = %.3f\n\n", m_dblValue);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Read and display the firmware version of this U6.
    m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_CONFIG, LJ_chFIRMWARE_VERSION, &m_dblValue, 0);
    printf("U6 Firmware Version = %.3f\n\n", m_dblValue);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    return success;

    // mutex unlocks when locker goes out of scope
}

bool LabJackWorker::SetFileName(const QString &fileName)
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(!m_outputFile.isOpen()) // file is not open
    {
        m_outputFile.setFileName(fileName);
        return true;
    }
    else // file is already open
    {
        return false;
    }
}

bool LabJackWorker::ConfigureStream()
{
    bool success = false;

    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    //Open file for write and append
    if(!m_outputFile.open(QIODevice::WriteOnly | QIODevice::Append))
    {
        qDebug() << "File could not be opened: " << m_outputFile.fileName();
        return false;
    }
    else
    {
        m_textStream.setDevice(&m_outputFile);
        m_textStream << "File opened at: " << getCurrDateTimeStr() << '\n';
    }

    //Configure the stream:

    //Configure resolution of the analog inputs (pass a non-zero value for quick sampling).
    //See section 2.6 / 3.1 for more information.
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 0, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    // Configure the analog input range on channel 0 for bipolar + -5 volts.
    //m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_AIN_RANGE, 0, LJ_rgBIP5V, 0, 0);
    //ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Set the scan rate.
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, m_scanRate, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Give the driver a 5 second buffer (scanRate * 1 channels * 5 seconds).
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, m_scanRate * 1 * 5, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Configure reads to retrieve whatever data is available without waiting (wait mode LJ_swNONE).
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);

    //Define the scan list as AIN0.
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(m_lngErrorcode, __LINE__, 0);
    m_lngErrorcode = AddRequest(m_lngHandle, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0); // first method for single ended reading - AIN0
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

    return m_LabJackReady = success;
}

bool LabJackWorker::StartStream()
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(m_LabJackReady) // ready to read data
    {
        m_timeCurr = m_epoch.elapsed(); // msec since epoch

        //Start the stream.
        m_lngErrorcode = eGet(m_lngHandle, LJ_ioSTART_STREAM, 0, &m_dblValue, 0);
        ErrorHandler(m_lngErrorcode, __LINE__, 0);

        // Write start time
        if(m_outputFile.isOpen())
        {
            m_textStream << "Epoch at: "
                         << m_epoch.toString("dd/MM/yyyy - hh:mm:ss.zzz") << '\n';
            m_textStream << "Recording started at: " << getCurrTimeStr() << '\n';

            // write actual scan rate to file
            m_textStream << "Actual Scan Rate = " << m_dblValue << '\n';
            m_textStream << "Time Since Epoch (ms) \t Voltage (V)" << '\n';
        }
        else
        {
            qDebug() << getCurrTimeStr() << "[StartStream] File is closed.";
            return false;
        }

        qDebug() << "Starting timer.";
        m_eventTimer->start(m_delayms);


        if(m_eventTimer->isActive())
        {
            qDebug() << "Timer started.";
            return true;
        }
        else
        {
            qDebug() << getCurrTimeStr() << "[StartStream] m_eventTimer is not active.";
            return false;
        }
    }
    else
    {
        qDebug() << getCurrTimeStr() << "[StartStream] LabJack is not ready.";
        return false;
    }
}

bool LabJackWorker::StopStream()
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(m_eventTimer->isActive())
    {
        m_eventTimer->stop();
        while(m_eventTimer->isActive())
        {
            ; // twiddle thumbs
        }

        m_lngErrorcode = eGet(m_lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
        ErrorHandler(m_lngErrorcode, __LINE__, 0);

        m_textStream << "Recording stopped at: " << getCurrTimeStr() << '\n\n';
        m_textStream.flush();
        qDebug() << "Timer stopped.";
        return true;
    }
    else
    {
        qDebug() << "Timer already stopped.";
        return false;
    }
}

bool LabJackWorker::DisconnectFromLabJack()
{
    bool success = StopStream();

    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(m_outputFile.isOpen())
    {
        m_textStream.flush();
        m_outputFile.flush();
        m_outputFile.close();
    }
    else
        qDebug() << "textStream is already closed!";

    emit finished();

    //this->moveToThread(this->thread());

    return success;
}

void LabJackWorker::setEpoch(const QTime &epoch)
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    m_epoch = epoch;
}

bool LabJackWorker::workerIsReady()
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    bool result = m_LabJackReady;

    return result;
}

bool LabJackWorker::workerIsRecording()
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    bool result = m_eventTimer->isActive();

    return result;
}


bool LabJackWorker::ReadStream() {
    QTime tmr;
    tmr.start();

    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(m_LabJackReady)
    {
        for (m_k = 0; m_k < m_numScans; m_k++)
        {
            m_adblData[m_k] = 9999.0;
        }

        //Read the data.  We will request twice the number we expect, to
        //make sure we get everything that is available.
        //Note that the array we pass must be sized to hold enough SAMPLES, and
        //the Value we pass specifies the number of SCANS to read.
        m_numScansRequested = m_numScans;
        m_lngErrorcode = eGet(m_lngHandle, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS,
                              &m_numScansRequested, m_padblData);

        //The displays the number of scans that were actually read.
        //printf("\nIteration # %d\n", i);
        //printf("Number scans read = %.0f\n", numScansRequested);
        //outputFile << "Number scans read = " << numScansRequested << std::endl;

        //This displays just the first and last scans.
        //printf("First scan = %.3f, %.3f\n", adblData[0], adblData[(int)numScansRequested - 1]);
        ErrorHandler(m_lngErrorcode, __LINE__, 0);

        //Retrieve the current Comm backlog.  The UD driver retrieves stream data from
        //the U6 in the background, but if the computer is too slow for some reason
        //the driver might not be able to read the data as fast as the U6 is
        //acquiring it, and thus there will be data left over in the U6 buffer.
        //lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
        //printf("Comm Backlog = %.0f\n", dblCommBacklog);

        //Retrieve the current UD driver backlog.  If this is growing, then the application
        //software is not pulling data from the UD driver fast enough.
        //lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_UD, &dblUDBacklog, 0);
        //printf("UD Backlog = %.0f\n", dblUDBacklog);

        double tick = 1000./((double)m_scanRate); // msec

        // Write to file
        if (m_outputFile.isOpen()) // file is ready
        {
            for (int idx = 0; idx < m_numScansRequested; idx++)
            {
                m_textStream << QString::number(m_timeCurr + idx*tick, 'f', 3)
                             << '\t' << QString::number(m_adblData[idx], 'f', 6) << '\n';
            }

            // update current time for next iteration
            m_timeCurr += m_numScansRequested*tick;

            m_textStream.flush();

            m_i++;
            qDebug() << "Data acq took (ms): " << tmr.elapsed();
            return true;
        }
        else // file is not ready
        {
            qDebug() << "Data acq took (ms): " << tmr.elapsed();
            return false;
        }
    }
    else // LabJack not ready
    {
        qDebug() << "Data acq took (ms): " << tmr.elapsed();
        return false;
    }
}

bool LabJackWorker::ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration)
{
    char err[255];

    if (lngErrorcode != LJE_NOERROR)
    {
        ErrorToString(lngErrorcode, err);
        qDebug() << "Error number = " << lngErrorcode;
        qDebug() << "Error string = " << err;
        qDebug() << "Source line number = " << lngLineNumber;
        qDebug() << "Iteration = " << lngIteration;
        if (lngErrorcode > LJE_MIN_GROUP_ERROR)
        {
            qDebug() << "FATAL ERROR!";
        }
        return false; // there was an error
    }
    else
        return true; // no errors - success
}

// ------------------------------------- \\
// -------------- LabJack -------------- \\
// ------------------------------------- \\

LabJack::LabJack(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LabJack)
{
    ui->setupUi(this);
}

LabJack::~LabJack()
{
    if(ui->disconnectButton->isEnabled())
        ui->disconnectButton->click();

//    if(!workerThread.isNull())
//    {
//        if(workerThread->isRunning())
//        {
//            workerThread->quit();
//            workerThread->terminate();
//            workerThread->wait();
//            if(!worker.isNull())
//            {
//                delete worker;//->deleteLater();
//            }
//        }
//        delete workerThread;//->deleteLater();
//    }
//    if(!worker.isNull())
//    {
//        delete worker;//->deleteLater();
//    }

    delete ui;
}

bool LabJack::isReady()
{
    if(workerThread->isRunning())
        return worker->workerIsReady();
    else
        return false;
}

bool LabJack::isRecording()
{
    if(workerThread->isRunning())
        return worker->workerIsRecording();
    else
        return false;
}

bool LabJack::setWorkerEpoch(const QTime &epoch)
{
    if(workerThread->isRunning())
    {
        worker->setEpoch(epoch);
        return true;
    }
    else
        return false;
}

bool LabJack::setWorkerFileName(const QString &fileName)
{
    if(workerThread->isRunning())
    {
        if(!isRecording())
        {
            worker->SetFileName(fileName);
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool LabJack::ConfigureWorker(int samplesPsec)
{
    workerThread = new QThread;
    worker = new LabJackWorker(samplesPsec);

    worker->moveToThread(workerThread);
    connect(worker, SIGNAL(finished()), workerThread, SLOT(quit()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
    workerThread->start();
    return workerThread->isRunning();
}


void LabJack::on_connectButton_clicked()
{
    // get the desired number of samples from the GUI
    int samplesPsec = ui->samplesPsecSpinBox->value();

    bool result = ConfigureWorker(samplesPsec);

    // TODO: EPOCH SHOULD COME FROM MAIN GUI
    QTime tmp;
    tmp.start();
    result = result && setWorkerEpoch(tmp);
    // TODO: FILENAME SHOULD COME FROM MAIN GUI
    result = result && setWorkerFileName("test.txt");

    result = result && worker->ConnectToLabJack();

    if( result )
    {
        // update GUI elements
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Connected");
        ui->initializeLJbutton->setEnabled(true);
    }
}

void LabJack::on_disconnectButton_clicked()
{
    worker->DisconnectFromLabJack();

    // Stop collection, stop thread, delete worker
    //worker->StopStream();
//    workerThread->quit();
//    workerThread->terminate();
//    workerThread->wait();
//    delete worker;//->deleteLater();
//    delete workerThread;//->deleteLater();

    // update GUI elements
    ui->connectButton->setEnabled(true);
    ui->disconnectButton->setEnabled(false);
    ui->samplesPsecSpinBox->setEnabled(true);
    ui->statusLineEdit->setText("Disconnected");
    ui->initializeLJbutton->setEnabled(false);
    ui->startRecordButton->setEnabled(false);
    ui->stopRecordButton->setEnabled(false);
}

void LabJack::on_initializeLJbutton_clicked()
{
    bool result = worker->ConfigureStream();

    if( result )
    {
        // update GUI elements
        ui->statusLineEdit->setText("Initialized.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
    }
}

void LabJack::on_startRecordButton_clicked()
{
    bool result = worker->StartStream();

    if( result )
    {
        // update GUI elements
        ui->statusLineEdit->setText("Recording!");
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(true);
    }
}

void LabJack::on_stopRecordButton_clicked()
{
    bool result = worker->StopStream();

    if( result )
    {
        // update GUI elements
        ui->statusLineEdit->setText("Stopped! Ready.");
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
    }
}

inline const QString getCurrTimeStr()
{
    return QTime::currentTime().toString("HH.mm.ss.zzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("dd/MM/yyyy - hh:mm:ss.zzz");
}
