#include "icebot_gui.h"
#include "ui_icebot_gui.h"

ICEbot_GUI::ICEbot_GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ICEbot_GUI)
{
    ui->setupUi(this);

    qDebug() << "Setting up GUI connections.";

    //connect(ui->frmGrabWidget->m_worker, SIGNAL(pleaseSaveImage(std::shared_ptr<Frame>)), ui->dataLogWidget->m_worker, SLOT(...));
    connect(ui->emWidget->m_worker, SIGNAL(logData(QTime,int,DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD)),
            ui->dataLogWidget->m_worker, SLOT(logEMdata(QTime,int,DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD)));

    qDebug() << "GUI ready.";
}

ICEbot_GUI::~ICEbot_GUI()
{
    delete ui;
}
