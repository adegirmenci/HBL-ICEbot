#include "frmgrabthread.h"

FrmGrabThread::FrmGrabThread(QObject *parent) : QObject(parent)
{
    qRegisterMetaType< std::shared_ptr<Frame> >("std::shared_ptr<Frame>");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepStreaming = false;
    m_showLiveFeed = false;
    m_numSaveImageRequests = 0;
    m_frameCount = 0;
    m_abort = false;

    m_continuousSaving = false;

    m_videoFPS = 0;

    m_mutex = new QMutex(QMutex::Recursive);
}

FrmGrabThread::~FrmGrabThread()
{
    frmGrabDisconnect();

    m_mutex->lock();
    m_continuousSaving = false;
    m_abort = true;
    qDebug() << "Ending FrmGrabThread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
    m_mutex->unlock();

    delete m_mutex;
    m_src.release();
    m_dst.release();

    emit finished();
}


void FrmGrabThread::frmGrabConnect()
{
    QMutexLocker locker(m_mutex);

    emit statusChanged(FRMGRAB_CONNECT_BEGIN);

    m_cap.open(0); // open the default camera
    if(m_cap.isOpened()) // check if we succeeded
    {
        emit statusChanged(FRMGRAB_CONNECTED); // success
    }
    else
    {
        emit statusChanged(FRMGRAB_CONNECT_FAILED); // failed
    }
}

void FrmGrabThread::frmGrabInitialize(int width, int height, double fps)
{
    QMutexLocker locker(m_mutex);

    if(m_cap.isOpened())
    {
        m_videoFPS = fps; // m_cap.get(CV_CAP_PROP_FPS);
        m_imgSize.height = height; //m_cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        m_imgSize.width = width; //m_cap.get(CV_CAP_PROP_FRAME_WIDTH);

        m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_imgSize.height);
        m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_imgSize.width);
        m_cap.set(CV_CAP_PROP_FPS, m_videoFPS);

        qDebug() << "Width:" << m_imgSize.width << "Height:" << m_imgSize.height << "FPS:" << m_videoFPS;

        emit statusChanged(FRMGRAB_INITIALIZED);
    }
    else
        emit statusChanged(FRMGRAB_INITIALIZE_FAILED);
}

void FrmGrabThread::startStream()
{
    QMutexLocker locker(m_mutex);

    if(!m_keepStreaming)
    {
        m_timer = new QTimer(this);
        m_timer->start(floor(1000./m_videoFPS));
        connect(m_timer, SIGNAL(timeout()), this, SLOT(grabFrame()));

        m_keepStreaming = true;

        emit logEvent(SRC_FRMGRAB, LOG_INFO, QTime::currentTime(), FRMGRAB_LOOP_STARTED);
        emit statusChanged(FRMGRAB_LOOP_STARTED);

        qDebug() << "Streaming started.";
    }
}

void FrmGrabThread::stopStream()
{
    QMutexLocker locker(m_mutex);

    if ( m_keepStreaming )
    {
        if(m_showLiveFeed)
            toggleLiveFeed();

        m_keepStreaming = false;
        m_timer->stop();

        disconnect(m_timer,SIGNAL(timeout()), 0, 0);

        delete m_timer;

        emit logEvent(SRC_FRMGRAB, LOG_INFO, QTime::currentTime(), FRMGRAB_LOOP_STOPPED);

        qDebug() << "Streaming stopped.";
    }
    else
    {
        qDebug() << "Streaming already stopped.";
    }

    emit statusChanged(FRMGRAB_LOOP_STOPPED);
}

void FrmGrabThread::toggleLiveFeed()
{
    QMutexLocker locker(m_mutex);

    m_showLiveFeed = !m_showLiveFeed;

    if(m_showLiveFeed)
    {
        cv::namedWindow(m_winName.toStdString());
        cv::resizeWindow(m_winName.toStdString(), m_imgSize.width, m_imgSize.height);
        // make connections
        connect(this, SIGNAL(imageAcquired(std::shared_ptr<Frame>)), this, SLOT(displayFrame(std::shared_ptr<Frame>)));

        emit statusChanged(FRMGRAB_LIVE_FEED_STARTED);
    }
    else
    {
        // break connections

        disconnect(this, SIGNAL(imageAcquired(std::shared_ptr<Frame>)), this, SLOT(displayFrame(std::shared_ptr<Frame>)));

        cv::destroyWindow(m_winName.toStdString());

        emit statusChanged(FRMGRAB_LIVE_FEED_STOPPED);
    }
}

void FrmGrabThread::displayFrame(std::shared_ptr<Frame> frm)
{
    QMutexLocker locker(m_mutex);

    cv::imshow(m_winName.toStdString(), frm->image_);
}


void FrmGrabThread::frmGrabDisconnect()
{
    stopStream();

    QMutexLocker locker(m_mutex);

    m_cap.release(); // release video capture device

    emit statusChanged(FRMGRAB_DISCONNECTED);
}

void FrameDeleter(Frame* frm)
{
    frm->image_.release();
}

void FrmGrabThread::grabFrame()
{
    QMutexLocker locker(m_mutex);

    // capture frame
    m_cap >> m_src;
    //m_cap.read(m_src);

    qint64 msec = QDateTime::currentMSecsSinceEpoch();

    // convert to grayscale
    cv::cvtColor(m_src, m_dst, CV_BGR2GRAY);

    // construct new frame
    std::shared_ptr<Frame> frame(new Frame(m_dst, msec, m_frameCount), FrameDeleter);

    if(m_continuousSaving)
    {
        m_numSaveImageRequests++;
    }

    if(m_numSaveImageRequests > 0)
    {
        emit pleaseSaveImage(frame);
        m_frameCount++;
        m_numSaveImageRequests--;
    }

    //if(m_showLiveFeed)

    emit imageAcquired(frame);
}

void FrmGrabThread::startSaving()
{
    QMutexLocker locker(m_mutex);

    m_continuousSaving = true;
}

void FrmGrabThread::stopSaving()
{
    QMutexLocker locker(m_mutex);

    m_continuousSaving = false;
}

void FrmGrabThread::addSaveRequest(unsigned short numFrames)
{
    QMutexLocker locker(m_mutex);

    m_numSaveImageRequests += numFrames;
}

void FrmGrabThread::setEpoch(const QDateTime &datetime)
{
    QMutexLocker locker(m_mutex);
    if(!m_keepStreaming)
    {
        m_epoch = datetime;
        m_isEpochSet = true;

        emit logEventWithMessage(SRC_FRMGRAB, LOG_INFO, QTime::currentTime(), FRMGRAB_EPOCH_SET,
                                 m_epoch.toString("yyyy/MM/dd - hh:mm:ss.zzz"));
    }
    else
        emit logEvent(SRC_FRMGRAB, LOG_INFO, QTime::currentTime(), FRMGRAB_EPOCH_SET_FAILED);
}
