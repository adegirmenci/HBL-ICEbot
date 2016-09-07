#include "icebot_gui.h"
#include "ui_icebot_gui.h"

ICEbot_GUI::ICEbot_GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ICEbot_GUI)
{
    ui->setupUi(this);

    qDebug() << "Setting up GUI connections.";

    connect(ui->emWidget->m_worker, SIGNAL(logData(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)),
            ui->dataLogWidget->m_worker, SLOT(logEMdata(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)));

    connect(ui->labjackWidget->m_worker, SIGNAL(logData(QTime,double)),
            ui->dataLogWidget->m_worker, SLOT(logLabJackData(QTime,double)));

    connect(ui->frmGrabWidget->m_worker, SIGNAL(pleaseSaveImage(std::shared_ptr<Frame>)),
            ui->dataLogWidget->m_worker, SLOT(logFrmGrabImage(std::shared_ptr<Frame>)));

    connect(ui->eposWidget->m_worker, SIGNAL(logData(QTime,int,int,long)),
            ui->dataLogWidget->m_worker, SLOT(logEPOSdata(QTime,int,int,long)));
    connect(ui->eposWidget->m_worker, SIGNAL(logData(QTime,int,std::vector<long>)),
            ui->dataLogWidget->m_worker, SLOT(logEPOSdata(QTime,int,std::vector<long>)));

    connect(ui->emWidget->m_worker, SIGNAL(logEvent(int,int,QTime,int)),
            ui->dataLogWidget->m_worker, SLOT(logEvent(int,int,QTime,int)));
    connect(ui->eposWidget->m_worker, SIGNAL(logEvent(int,int,QTime,int)),
            ui->dataLogWidget->m_worker, SLOT(logEvent(int,int,QTime,int)));
    connect(ui->frmGrabWidget->m_worker, SIGNAL(logEvent(int,int,QTime,int)),
            ui->dataLogWidget->m_worker, SLOT(logEvent(int,int,QTime,int)));
    connect(ui->labjackWidget->m_worker, SIGNAL(logEvent(int,int,QTime,int)),
            ui->dataLogWidget->m_worker, SLOT(logEvent(int,int,QTime,int)));

    connect(ui->emWidget->m_worker, SIGNAL(logError(int,int,QTime,int,QString)),
            ui->dataLogWidget->m_worker, SLOT(logError(int,int,QTime,int,QString)));
    connect(ui->eposWidget->m_worker, SIGNAL(logError(int,int,QTime,int,QString)),
            ui->dataLogWidget->m_worker, SLOT(logError(int,int,QTime,int,QString)));
    connect(ui->frmGrabWidget->m_worker, SIGNAL(logError(int,int,QTime,int,QString)),
            ui->dataLogWidget->m_worker, SLOT(logError(int,int,QTime,int,QString)));
    connect(ui->labjackWidget->m_worker, SIGNAL(logError(int,int,QTime,int,QString)),
            ui->dataLogWidget->m_worker, SLOT(logError(int,int,QTime,int,QString)));

    connect(ui->emWidget->m_worker, SIGNAL(logData(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)),
            ui->sceneVizWidget->m_modifier, SLOT(receiveEMreading(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)));

    // inter-process communication
    connect(ui->frmGrabWidget->m_worker, SIGNAL(imageAcquired(std::shared_ptr<Frame>)),
            ui->frameClientWidget->m_worker, SLOT(receiveFrame(std::shared_ptr<Frame>)));
    connect(ui->emWidget->m_worker, SIGNAL(logData(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)),
            ui->frameClientWidget->m_worker, SLOT(receiveEMreading(QTime,int,DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD)));

    // get current date time
    m_epoch = QDateTime::currentDateTime();

    qDebug() << "Setting epoch.";

    // set date time of widgets
    ui->emWidget->m_worker->setEpoch(m_epoch);
    ui->frmGrabWidget->m_worker->setEpoch(m_epoch);
    ui->eposWidget->m_worker->setEpoch(m_epoch);
    ui->labjackWidget->m_worker->setEpoch(m_epoch);
    ui->frameClientWidget->m_worker->setEpoch(m_epoch);

    qDebug() << "GUI ready.";

}

ICEbot_GUI::~ICEbot_GUI()
{
    delete ui;
}
