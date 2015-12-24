#include "ascensionwidget.h"
#include "ui_ascensionwidget.h"

AscensionWidget::AscensionWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AscensionWidget)
{
    ui->setupUi(this);

    m_worker = new AscensionThread;
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_thread.start();

    connect(ui->initButton, SIGNAL(clicked(bool)), m_worker, SLOT(initializeEM()));
    connect(ui->acquireButton, SIGNAL(clicked(bool)), m_worker, SLOT(startAcquisition()));
    connect(ui->stopButton, SIGNAL(clicked(bool)), m_worker, SLOT(stopAcquisition()));
    connect(ui->disconnectButton, SIGNAL(clicked(bool)), m_worker, SLOT(disconnectEM()));

    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));
    connect(m_worker, SIGNAL(sendDataToGUI(int,QString)),
                             this, SLOT(receiveDataFromWorker(int,QString)));
}

AscensionWidget::~AscensionWidget()
{
    m_thread.quit();
    m_thread.wait();

    qDebug() << "Ascension thread quit.";

    delete ui;
}

void AscensionWidget::workerStatusChanged(int status)
{
    switch(status)
    {
    case EM_INITIALIZE_BEGIN:
        ui->initButton->setEnabled(false);
        ui->statusLineEdit->setText("Initializing...");
        ui->outputTextEdit->appendPlainText("Initializing...This will take a minute.");
        break;
    case EM_INITIALIZE_FAILED:
        ui->initButton->setEnabled(true);
        ui->statusLineEdit->setText("Initialization failed!");
        ui->outputTextEdit->appendPlainText("Initialization failed!");
        break;
    case EM_INITIALIZED:
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->acquireButton->setEnabled(true);
        ui->outputTextEdit->appendPlainText(QString("%1 sensor(s) plugged in.")
                                            .arg(m_worker->getNumSensors()));
        ui->outputTextEdit->appendPlainText(QString("Measurement Rate: %1Hz")
                                            .arg(m_worker->getSampleRate()));
        break;
    case EM_DISCONNECT_FAILED:
        ui->statusLineEdit->setText("Disconnection failed.");
        ui->outputTextEdit->appendPlainText("Disconnection failed! FATAL ERROR!");
        break;
    case EM_DISCONNECTED:
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->acquireButton->setEnabled(false);
        ui->statusLineEdit->setText("Disconnected.");
        ui->outputTextEdit->appendPlainText("Disconnected.");
        break;
    case EM_FREQ_SET_FAILED:
        ui->outputTextEdit->appendPlainText("Sample rate could not be changed!");
        break;
    case EM_FREQ_SET:
        ui->outputTextEdit->appendPlainText(QString("Sample rate changed to %1Hz.")
                                            .arg(m_worker->getSampleRate()));
        break;
    case EM_ACQUISITION_STARTED:
        ui->statusLineEdit->setText("Acquiring data.");
        ui->acquireButton->setEnabled(false);
        ui->stopButton->setEnabled(true);
        break;
    case EM_ACQUISITION_STOPPED:
        ui->statusLineEdit->setText("Acquisition stopped.");
        ui->acquireButton->setEnabled(true);
        ui->stopButton->setEnabled(false);
        break;
    case EM_ACQUIRE_FAILED:
        ui->statusLineEdit->setText("Acquisition failed.");
        break;
    default:
        ui->statusLineEdit->setText("Unknown state!");
        break;
    }
}

void AscensionWidget::receiveDataFromWorker(int sensorID, const QString &data)
{
    if(sensorID == 0)
        ui->outputTextEdit->clear();

    ui->outputTextEdit->appendPlainText(data);
}

void AscensionWidget::on_initButton_clicked()
{
    ui->initButton->setEnabled(false);
}
