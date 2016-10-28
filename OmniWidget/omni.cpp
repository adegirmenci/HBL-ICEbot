#include "omni.h"
#include "ui_omni.h"

/** Use to init the clock */
//#define TIMER_INIT LARGE_INTEGER frequency;LARGE_INTEGER t1,t2;double elapsedTime;QueryPerformanceFrequency(&frequency);
/** Use to start the performance timer */
//#define TIMER_START QueryPerformanceCounter(&t1);
/** Use to stop the performance timer and output the result to the standard stream. Less verbose than \c TIMER_STOP_VERBOSE */
//#define TIMER_STOP QueryPerformanceCounter(&t2);elapsedTime=(float)(t2.QuadPart-t1.QuadPart)/frequency.QuadPart;printf("%g sec\n",elapsedTime);


Omni::Omni(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Omni)
{
    ui->setupUi(this);

//    omniTimer = new QTimer();
//    omniTimer->setInterval(1);
//    connect(omniTimer, SIGNAL(timeout()), this, SLOT(mainLoop()));
//    //connect(omniTimer, SIGNAL(timeout()), ui->omniWidget, SLOT(mainLoop()));
//    omniTimer->start();

    m_omnithread = new omniThread();
    //m_omnithread->moveToThread(&momniThread);
    m_omnithread->start();

    updateTimer = new QTimer();
    updateTimer->setInterval(150);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateLCD()));
    updateTimer->start();
}

Omni::~Omni()
{
    //omniTimer->stop();
    updateTimer->stop();

    if(m_omnithread->isRunning())
    {
        m_omnithread->close();
        m_omnithread->exit();
    }

    delete ui;
}

void Omni::updateLCD()
{
    hduVector3Dd pos;
    hduVector3Dd rot;
    m_omnithread->getDevicePosRot(pos, rot);
    ui->xLCD->display(pos[0]);
    ui->yLCD->display(pos[1]);
    ui->zLCD->display(pos[2]);
    ui->rollLCD->display(rot[0]);
    ui->pitchLCD->display(rot[1]);
    ui->yawLCD->display(rot[2]);
}

void Omni::on_sphSizeSpinBox_valueChanged(double arg1)
{
    m_omnithread->setSphereRad(arg1);
}
