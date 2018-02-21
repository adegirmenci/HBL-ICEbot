#include "labjackwidget.h"
#include "ui_labjackwidget.h"

LabJackWidget::LabJackWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LabJackWidget)
{
    ui->setupUi(this);

    m_worker = new LabJackThread;
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    // connect widget signals to worker slots
    connect(this, SIGNAL(connectLabJack()), m_worker, SLOT(connectLabJack()));
    connect(this, SIGNAL(initializeLabJack(uint, QVector<ushort>, QVector<QString>)),
            m_worker, SLOT(initializeLabJack(uint, QVector<ushort>, QVector<QString>)));
    connect(this, SIGNAL(disconnectLabJack()), m_worker, SLOT(disconnectLabJack()));
    connect(this, SIGNAL(startAcquisition()), m_worker, SLOT(startAcquisition()));
    connect(this, SIGNAL(stopAcquisition()), m_worker, SLOT(stopAcquisition()));

    // connect worker signals to widget slots
    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));

    // QCustomPlot
    // include this section to fully disable antialiasing for higher performance:
    ui->plotWidget->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    ui->plotWidget->xAxis->setTickLabelFont(font);
    ui->plotWidget->yAxis->setTickLabelFont(font);
    ui->plotWidget->legend->setFont(font);

    ui->plotWidget->addGraph(); // blue line
    ui->plotWidget->graph(0)->setPen(QPen(Qt::blue));
    //ui->plotWidget->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
    ui->plotWidget->graph(0)->setAntialiasedFill(false);

    ui->plotWidget->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->plotWidget->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->plotWidget->xAxis->setAutoTickStep(false);
    ui->plotWidget->xAxis->setTickStep(2);
    ui->plotWidget->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->plotWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plotWidget->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plotWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plotWidget->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(m_worker, SIGNAL(logData(qint64,std::vector<double>)), this, SLOT(addDataToPlot(qint64,std::vector<double>)));

    // Heart Rate Widget
    connect(m_worker, SIGNAL(logData(qint64,std::vector<double>)), ui->HRwidget, SLOT(receiveECG(qint64,std::vector<double>)));

    // qDebug() << "LabJack Widget Thread ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
    m_hrWidget = ui->HRwidget;
}

LabJackWidget::~LabJackWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "LabJack thread quit.";

    delete ui;
}

void LabJackWidget::addDataToPlot(qint64 timeStamp, std::vector<double> data)
{
    //TODO : automatically add more lines depending on the size of 'data'
    double key = timeStamp/1000.0;
    static double lastPointKey = 0;
    if( (key - lastPointKey) > 0.010) // at most add point every 10 ms
    {
      // add data to lines:
      ui->plotWidget->graph(0)->addData(key, data[0]);
      // remove data of lines that's outside visible range:
      ui->plotWidget->graph(0)->removeDataBefore(key-8);
      // rescale value (vertical) axis to fit the current data:
      ui->plotWidget->graph(0)->rescaleValueAxis();
      lastPointKey = key;

      // make key axis range scroll with the data (at a constant range size of 8):
      ui->plotWidget->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
      ui->plotWidget->replot();
    }
}

void LabJackWidget::workerStatusChanged(int status)
{
    switch(status)
    {
    case LABJACK_CONNECT_BEGIN:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Connecting.");
        ui->initializeLJbutton->setEnabled(true);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_CONNECT_FAILED:
        ui->connectButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->samplesPsecSpinBox->setEnabled(true);
        ui->statusLineEdit->setText("Connection failed.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_CONNECTED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Connected.");
        ui->initializeLJbutton->setEnabled(true);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_INITIALIZE_BEGIN:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Initializing.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_INITIALIZE_FAILED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Initialization failed.");
        ui->initializeLJbutton->setEnabled(true);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_INITIALIZED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Initialized.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_LOOP_STARTED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Acquiring.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(true);
        break;
    case LABJACK_LOOP_STOPPED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Acquisition stopped.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_EPOCH_SET:
        ui->statusLineEdit->setText("Epoch set.");
        break;
    case LABJACK_EPOCH_SET_FAILED:
        ui->statusLineEdit->setText("Epoch set failed.");
        break;
    case LABJACK_DISCONNECTED:
        ui->connectButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->samplesPsecSpinBox->setEnabled(true);
        ui->statusLineEdit->setText("Disconnected.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(false);
        ui->stopRecordButton->setEnabled(false);
        break;
    case LABJACK_DISCONNECT_FAILED:
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->samplesPsecSpinBox->setEnabled(false);
        ui->statusLineEdit->setText("Disconnection failed.");
        ui->initializeLJbutton->setEnabled(false);
        ui->startRecordButton->setEnabled(true);
        ui->stopRecordButton->setEnabled(false);
        break;
    default:
        ui->statusLineEdit->setText("Unknown state.");
        break;
    }
}

void LabJackWidget::on_connectButton_clicked()
{
    emit connectLabJack();
}

void LabJackWidget::on_disconnectButton_clicked()
{
    emit disconnectLabJack();
}

void LabJackWidget::on_initializeLJbutton_clicked()
{
    int samplesPsec = ui->samplesPsecSpinBox->value();

    QVector<ushort> channelIdx;
    QVector<QString> channelNames;

    // TODO: figure out which channels are desired by the user
    channelIdx.push_back(0);
    channelNames.push_back("ECG");

    emit initializeLabJack(samplesPsec,channelIdx,channelNames);
}

void LabJackWidget::on_startRecordButton_clicked()
{
    emit startAcquisition();
}

void LabJackWidget::on_stopRecordButton_clicked()
{
    emit stopAcquisition();
}
