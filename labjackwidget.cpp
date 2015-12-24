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
    connect(this, SIGNAL(initializeLabJack(uint)), m_worker, SLOT(initializeLabJack(uint)));
    connect(this, SIGNAL(disconnectLabJack()), m_worker, SLOT(disconnectLabJack()));
    connect(this, SIGNAL(startAcquisition()), m_worker, SLOT(startAcquisition()));
    connect(this, SIGNAL(stopAcquisition()), m_worker, SLOT(stopAcquisition()));

    // connect worker signals to widget slots
    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));
}

LabJackWidget::~LabJackWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "LabJack thread quit.";

    delete ui;
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

    emit initializeLabJack(samplesPsec);
}

void LabJackWidget::on_startRecordButton_clicked()
{
    emit startAcquisition();
}

void LabJackWidget::on_stopRecordButton_clicked()
{
    emit stopAcquisition();
}
