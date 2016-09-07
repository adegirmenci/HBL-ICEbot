#include "frameclientwidget.h"
#include "ui_frameclientwidget.h"

FrameClientWidget::FrameClientWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FrameClientWidget)
{
    ui->setupUi(this);

    //m_keepClientRunning = false;

    m_worker = new FrameClientThread;
    m_worker->moveToThread(&m_thread);

    connect(&m_thread, SIGNAL(finished()), m_worker, SLOT(deleteLater()));
    m_thread.start();

    connect(m_worker, SIGNAL(statusChanged(int)), this, SLOT(workerStatusChanged(int)));

    connect(this, SIGNAL(sendFrame()), m_worker, SLOT(sendFrame()));
}

FrameClientWidget::~FrameClientWidget()
{
    m_thread.quit();
    m_thread.wait();
    qDebug() << "FrameClientWidget thread quit.";

    delete ui;
}

void FrameClientWidget::workerStatusChanged(int status)
{
    switch(status)
    {
    case FRMCLNT_CONNECTED:
        ui->statusTextEdit->appendPlainText("Connected to server.");
        ui->addrPortLineEdit->setText(tr("%1:%2")
                                      .arg(m_worker->getServerAddress().toString())
                                      .arg(m_worker->getServerPort()));
        break;
    case FRMCLNT_CONNECTION_FAILED:
        ui->statusTextEdit->appendPlainText("Server connection failed.");
        break;
    case FRMCLNT_DISCONNECTED:
        ui->statusTextEdit->appendPlainText("Server connection closed.");
        break;
    case FRMCLNT_DISCONNECTION_FAILED:
        ui->statusTextEdit->appendPlainText("Server disconnect failed.");
        break;
    case FRMCLNT_SOCKET_NOT_WRITABLE:
        ui->statusTextEdit->appendPlainText("Incoming connection.");
        break;
    case FRMCLNT_FRAME_SENT:
        ui->statusTextEdit->appendPlainText("Frame sent.");
        break;
    case FRMCLNT_EPOCH_SET:
        ui->statusTextEdit->appendPlainText("Epoch set.");
        break;
    case FRMCLNT_EPOCH_SET_FAILED:
        ui->statusTextEdit->appendPlainText("Epoch set failed.");
        break;
    default:
        ui->statusTextEdit->appendPlainText("Unknown worker state.");
    }
}

void FrameClientWidget::on_sendFrameButton_clicked()
{
    emit sendFrame();
}
