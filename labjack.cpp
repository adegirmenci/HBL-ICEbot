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
    //m_delayms = 1000 - 10; // 1 seconds per 1000 samples - subtract 10 ms for other processing
    m_numScans = 2*m_scanRate;  //Max number of scans per read.  2x the expected # of scans (2*scanRate*delayms/1000).
    m_adblData = (double *)calloc( m_numScans, sizeof(double)); //Max buffer size (#channels*numScansRequested)
    m_padblData = (long)&m_adblData[0];

    m_LabJackReady = false;

    m_keepRecording = false; // flag for the while loop
}

LabJackWorker::~LabJackWorker()
{
    m_outputFile->close();
}

bool LabJackWorker::ConnectToLabJack()
{
    bool success = false;

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
}

bool LabJackWorker::configStream()
{
    bool success = false;
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

void LabJackWorker::doWork(const QString &parameter) {
    QString result;
    /* ... here is the expensive or blocking operation ... */
    emit resultReady(result);
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
    workerThread->quit();
    workerThread->wait();
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

void LabJack::ConfigureStream(int samplesPsec)
{
    workerThread = new QThread;
    worker = new LabJackWorker(samplesPsec);

    worker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(this, &LabJack::operate, worker, &LabJackWorker::doWork);
    connect(worker, &LabJackWorker::resultReady, this, &LabJack::handleResults);
    workerThread->start();
}

void LabJack::handleResults(const QString &str)
{

}

void LabJack::on_connectButton_clicked()
{
    // get the desired number of samples from the GUI
    int samplesPsec = ui->samplesPsecSpinBox->value();

    ConfigureStream(samplesPsec);
}
