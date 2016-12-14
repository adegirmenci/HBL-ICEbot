#include "frmgrabwidget.h"
#include "ui_frmgrabwidget.h"

FrmGrabWidget::FrmGrabWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FrmGrabWidget)
{
    ui->setupUi(this);

    m_worker = new FrmGrabThread;
    m_worker->moveToThread(&m_thread);

    mKeepSavingFrames = false;

    //connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(&m_thread, SIGNAL(finished()), m_worker, SLOT(deleteLater()));
    m_thread.start();

    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));

    connect(this, SIGNAL(frmGrabConnect()), m_worker, SLOT(frmGrabConnect()));
    connect(this, SIGNAL(frmGrabInitialize(int,int,double)), m_worker, SLOT(frmGrabInitialize(int,int,double)));
    connect(this, SIGNAL(startStream()), m_worker, SLOT(startStream()));
    connect(this, SIGNAL(stopStream()), m_worker, SLOT(stopStream()));
    connect(this, SIGNAL(toggleLiveFeed()), m_worker, SLOT(toggleLiveFeed()));
    connect(this, SIGNAL(addSaveRequest(unsigned short)), m_worker, SLOT(addSaveRequest(unsigned short)));
    connect(this, SIGNAL(frmGrabDisconnect()), m_worker, SLOT(frmGrabDisconnect()));
    connect(this, SIGNAL(startSaving()), m_worker, SLOT(startSaving()));
    connect(this, SIGNAL(stopSaving()), m_worker, SLOT(stopSaving()));
}

FrmGrabWidget::~FrmGrabWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "FrmGrab thread quit.";

    delete ui;
}

void FrmGrabWidget::workerStatusChanged(int status)
{
    switch(status)
    {
    case FRMGRAB_CONNECT_BEGIN:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Connecting.");
        break;
    case FRMGRAB_CONNECT_FAILED:
        ui->connectButton->setEnabled(true);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(false);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Connection failed.");
        break;
    case FRMGRAB_CONNECTED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Connected.");
        break;
    case FRMGRAB_INITIALIZE_BEGIN:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Initializing.");
        break;
    case FRMGRAB_INITIALIZE_FAILED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Initialization failed.");
        break;
    case FRMGRAB_INITIALIZED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(true);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Initialized.");
        break;
    case FRMGRAB_LOOP_STARTED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(true);
        ui->saveFrameButton->setEnabled(true);
        ui->stopStreamButton->setEnabled(true);
        ui->statusLineEdit->setText("Streaming.");
        break;
    case FRMGRAB_LOOP_STOPPED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(true);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Stream stopped.");
        break;
    case FRMGRAB_LIVE_FEED_STARTED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(true);
        ui->liveFeedButton->setText("Close Feed");
        ui->saveFrameButton->setEnabled(true);
        ui->stopStreamButton->setEnabled(true);
        ui->statusLineEdit->setText("Streaming.");
        break;
    case FRMGRAB_LIVE_FEED_STOPPED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(true);
        ui->liveFeedButton->setText("Live Feed");
        ui->saveFrameButton->setEnabled(true);
        ui->stopStreamButton->setEnabled(true);
        ui->statusLineEdit->setText("Streaming.");
        break;
    case FRMGRAB_EPOCH_SET:
        qDebug() << "Epoch set.";
        break;
    case FRMGRAB_EPOCH_SET_FAILED:
        qDebug() << "Epoch set failed.";
        break;
    case FRMGRAB_DISCONNECTED:
        ui->connectButton->setEnabled(true);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(false);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Disconnected.");
        break;
    case FRMGRAB_DISCONNECT_FAILED:
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->startStreamButton->setEnabled(false);
        ui->liveFeedButton->setEnabled(false);
        ui->saveFrameButton->setEnabled(false);
        ui->stopStreamButton->setEnabled(false);
        ui->statusLineEdit->setText("Disconnect failed.");
        break;
    default:
        qDebug() << "Unknown state!";
        break;
    }
}

void FrmGrabWidget::on_connectButton_clicked()
{
    emit frmGrabConnect();
}

void FrmGrabWidget::on_initButton_clicked()
{
    int width = ui->widthSpinBox->value();
    int height = ui->heightSpinBox->value();
    double fps = ui->fpsSpinBox->value();

    emit frmGrabInitialize(width, height, fps);
}

void FrmGrabWidget::on_disconnectButton_clicked()
{
    emit frmGrabDisconnect();
}

void FrmGrabWidget::on_startStreamButton_clicked()
{
    emit startStream();
}

void FrmGrabWidget::on_liveFeedButton_clicked()
{
    emit toggleLiveFeed();
}

void FrmGrabWidget::on_saveFrameButton_clicked()
{
    emit addSaveRequest(1); // request a single frame
}

void FrmGrabWidget::on_stopStreamButton_clicked()
{
    emit stopStream();
}

void FrmGrabWidget::on_saveFramesButton_clicked()
{
    if(mKeepSavingFrames) // already saving, so stop saving
    {
        mKeepSavingFrames = false;
        ui->saveFramesButton->setText("Start Saving");
        emit stopSaving();
    }
    else
    {
        mKeepSavingFrames = true;
        ui->saveFramesButton->setText("Stop Saving");
        emit startSaving();
    }
}

void FrmGrabWidget::controlStarted()
{
    if(!mKeepSavingFrames)
    {
        mKeepSavingFrames = true;
        ui->saveFramesButton->setText("Stop Saving");
        emit startSaving();
    }
}

void FrmGrabWidget::controlStopped()
{
    if(mKeepSavingFrames)
    {
        mKeepSavingFrames = false;
        ui->saveFramesButton->setText("Start Saving");
        emit stopSaving();
    }
}
