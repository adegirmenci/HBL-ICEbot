#include "frameclientthread.h"

FrameClientThread::FrameClientThread(QObject *parent) : QObject(parent)
{
    qRegisterMetaType< FrameExtd >("FrameExtd");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepStreaming = false;
    m_frameCount = 0;
    m_abort = false;
    m_serverAddress = QHostAddress(QHostAddress::LocalHost);
    m_serverPort = (quint16)4417;

    m_TcpSocket = Q_NULLPTR;

    m_currExtdFrame = FrameExtd();
    m_currFrame = nullptr;
    m_currBird = EMreading();

    m_mutex = new QMutex(QMutex::Recursive);
}

FrameClientThread::~FrameClientThread()
{
    m_mutex->lock();
    m_abort = true;
    m_isReady = false;
    m_keepStreaming = false;
    if(m_TcpSocket)
    {
        m_TcpSocket->flush();
        m_TcpSocket->disconnectFromHost();
        m_TcpSocket->deleteLater();
    }
    m_mutex->unlock();

    qDebug() << "Ending FrameClientThread - ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";

    delete m_mutex;
    emit finished();
}

void FrameClientThread::initializeFrameClient()
{
    QMutexLocker locker(m_mutex);

    m_TcpSocket = new QTcpSocket(this);

    connect(m_TcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(handleTcpError(QAbstractSocket::SocketError)));
    connect(this, SIGNAL(tcpError(QAbstractSocket::SocketError)),
            this, SLOT(handleTcpError(QAbstractSocket::SocketError)));

    connect(m_TcpSocket, SIGNAL(connected()), this, SLOT(connectedToHost()));
    connect(m_TcpSocket, SIGNAL(disconnected()), this, SLOT(disconnectedFromHost()));

    m_isReady = true;
}

void FrameClientThread::sendFrame()
{
    QMutexLocker locker(m_mutex);

    if(!m_isReady)
        initializeFrameClient();

    if(m_keepStreaming)
    {
        // construct frame - rot matrix from Ascension is transposed
        QMatrix4x4 tmp(m_currBird.data.s[0][0], m_currBird.data.s[0][1], m_currBird.data.s[0][2], m_currBird.data.x,
                       m_currBird.data.s[1][0], m_currBird.data.s[1][1], m_currBird.data.s[1][2], m_currBird.data.y,
                       m_currBird.data.s[2][0], m_currBird.data.s[2][1], m_currBird.data.s[2][2], m_currBird.data.z,
                                           0.0,                     0.0,                     0.0,               1.0);
        Qt3DCore::QTransform tform;
        tform.setMatrix(tmp);
//        QQuaternion q(m_currBird.data.q[0],
//                      m_currBird.data.q[1],
//                      m_currBird.data.q[2],
//                      m_currBird.data.q[3]);
        QQuaternion q = tform.rotation();
        QVector3D v = tform.translation();
        m_currExtdFrame.EMq_ = q;
        m_currExtdFrame.EMv_ = v;
        m_currExtdFrame.image_ = saveFrame(m_currFrame); // write to disk
        m_currExtdFrame.index_ = m_currFrame->index_;
        m_currExtdFrame.mask_ = tr("C:\\Users\\Alperen\\Documents\\QT Projects\\RT3DReconst_GUI\\Acuson_Epiphan.bin");
        m_currExtdFrame.timestamp_ = m_currFrame->timestamp_;

        QByteArray block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_5_7);
        out << (quint16)0;
        out << m_currExtdFrame;
        out.device()->seek(0);
        out << (quint16)(block.size() - sizeof(quint16));

        m_TcpSocket->connectToHost(m_serverAddress, m_serverPort, QIODevice::WriteOnly);

        if (m_TcpSocket->waitForConnected(100))
        {
              qDebug("Connected!");

            m_TcpSocket->write(block);
            m_TcpSocket->flush();
            m_TcpSocket->disconnectFromHost();
            if( (m_TcpSocket->state() != QAbstractSocket::UnconnectedState) &&
                (!m_TcpSocket->waitForDisconnected(100)) ) {
                emit statusChanged(FRMCLNT_DISCONNECTION_FAILED); }
        }
        else
            emit statusChanged(FRMCLNT_CONNECTION_FAILED);
    }
    else
        emit statusChanged(FRMCLNT_FIRST_FRAME_NOT_RECEIVED);
}

void FrameClientThread::receiveFrame(std::shared_ptr<Frame> frame)
{
    QMutexLocker locker(m_mutex);

    // qDebug() << "FrameClientThread: receiveFrame";

    m_currFrame = frame;
}

void FrameClientThread::receiveEMreading(QTime timeStamp, int sensorID, DOUBLE_POSITION_MATRIX_TIME_Q_RECORD data)
{
    QMutexLocker locker(m_mutex);

    // qDebug() << "FrameClientThread: receiveEMreading";
    if(m_currFrame)
    {
        qint64 tBirdCurr(m_currBird.data.time*1000);
        qint64 tBirdNext(data.time*1000);
        qint64 diffCurr = qAbs(m_currFrame->timestamp_ - tBirdCurr);
        qint64 diffNext = qAbs(m_currFrame->timestamp_ - tBirdNext);

        if(diffNext < diffCurr)
        {
            m_currBird.data = data;
            m_currBird.sensorID = sensorID;
            m_currBird.timeStamp = timeStamp;
        }
    }

    if(m_currFrame)
        m_keepStreaming = true;
}

void FrameClientThread::receiveLatestEMreading(std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> readings)
{
    QMutexLocker locker(m_mutex);

    Q_ASSERT(4 == readings.size());

    if(m_currFrame)
    {
        qint64 tBirdCurr(m_currBird.data.time*1000);
        qint64 tBirdNext(readings[EM_SENSOR_BT].time*1000);
        qint64 diffCurr = qAbs(m_currFrame->timestamp_ - tBirdCurr);
        qint64 diffNext = qAbs(m_currFrame->timestamp_ - tBirdNext);

        if(diffNext < diffCurr)
        {
            m_currBird.data = readings[EM_SENSOR_BT];
            m_currBird.sensorID = EM_SENSOR_BT;
            m_currBird.timeStamp = QTime::currentTime();
        }
    }

    if(m_currFrame)
        m_keepStreaming = true;
}

void FrameClientThread::receive_T_CT(std::vector<double> T_BB_CT, double time)
{
    QMutexLocker locker(m_mutex);

    if(m_currFrame && (T_BB_CT.size() == 16))
    {
        qint64 tBirdCurr(m_currBird.data.time*1000);
        qint64 tBirdNext(time*1000);
        qint64 diffCurr = qAbs(m_currFrame->timestamp_ - tBirdCurr);
        qint64 diffNext = qAbs(m_currFrame->timestamp_ - tBirdNext);

        if(diffNext < diffCurr)
        {
            // data in Eigen is stored as column-major
            m_currBird.data.s[0][0] = T_BB_CT[0];
            m_currBird.data.s[1][0] = T_BB_CT[1];
            m_currBird.data.s[2][0] = T_BB_CT[2];
            m_currBird.data.s[0][1] = T_BB_CT[4];
            m_currBird.data.s[1][1] = T_BB_CT[5];
            m_currBird.data.s[2][1] = T_BB_CT[6];
            m_currBird.data.s[0][2] = T_BB_CT[8];
            m_currBird.data.s[1][2] = T_BB_CT[9];
            m_currBird.data.s[2][2] = T_BB_CT[10];
            m_currBird.data.x = T_BB_CT[12];
            m_currBird.data.y = T_BB_CT[13];
            m_currBird.data.z = T_BB_CT[14];
            m_currBird.data.time = time;
            m_currBird.sensorID = EM_SENSOR_BT;
            m_currBird.timeStamp = QTime::currentTime();
        }
    }

    if(m_currFrame)
        m_keepStreaming = true;
}

void FrameClientThread::connectedToHost()
{
    emit statusChanged(FRMCLNT_CONNECTED);
}

void FrameClientThread::disconnectedFromHost()
{
    emit statusChanged(FRMCLNT_DISCONNECTED);
}

QString FrameClientThread::saveFrame(std::shared_ptr<Frame> frm)
{
    QMutexLocker locker(m_mutex);

    QDateTime imgTime;
    imgTime.setMSecsSinceEpoch(frm->timestamp_);

    // file name of frame
    QString m_imgFname = imgTime.toString("ddMMyyyy_hhmmsszzz");
    // populate m_imgFname with index
    m_imgFname.append( QString("_%1").arg(frm->index_) );

    QString m_DirImgFname = QDir::currentPath();
    m_DirImgFname.append("/");
    m_DirImgFname.append(m_imgFname);
    // save frame
    //state = frame->image_.save(m_imgFname, "JPG", 100);
    cv::imwrite((m_DirImgFname + tr(".jp2")).toStdString().c_str(), frm->image_ ); // write frame

    // save EM data to file
    QFile txtfile(m_DirImgFname + tr(".txt"));
    if (txtfile.open(QFile::WriteOnly)) {
        QTextStream txtout(&txtfile);
        txtout << tr("%1\t%2\t%3\n")
                  .arg(m_currExtdFrame.EMv_.x())
                  .arg(m_currExtdFrame.EMv_.y())
                  .arg(m_currExtdFrame.EMv_.z());
        txtout << tr("%1\t%2\t%3\t%4\n")
                  .arg(m_currExtdFrame.EMq_.x())
                  .arg(m_currExtdFrame.EMq_.y())
                  .arg(m_currExtdFrame.EMq_.z())
                  .arg(m_currExtdFrame.EMq_.scalar());
        txtout << "Line 1: QVector3D, Line2: QQuaternion";
    }
    txtfile.close();

    return m_DirImgFname;
}

void FrameClientThread::handleTcpError(QAbstractSocket::SocketError error)
{
    QMutexLocker locker(m_mutex);

    QString errStr;
    switch(error)
    {
    case QAbstractSocket::ConnectionRefusedError:
        errStr = "ConnectionRefusedError"; break;
    case QAbstractSocket::RemoteHostClosedError:
        errStr = "RemoteHostClosedError"; break;
    case QAbstractSocket::HostNotFoundError:
        errStr = "HostNotFoundError"; break;
    case QAbstractSocket::SocketAccessError:
        errStr = "SocketAccessError"; break;
    case QAbstractSocket::SocketResourceError:
        errStr = "SocketResourceError"; break;
    case QAbstractSocket::SocketTimeoutError:
        errStr = "SocketTimeoutError"; break;
    case QAbstractSocket::DatagramTooLargeError:
        errStr = "DatagramTooLargeError"; break;
    case QAbstractSocket::NetworkError:
        errStr = "NetworkError"; break;
    case QAbstractSocket::AddressInUseError:
        errStr = "AddressInUseError"; break;
    case QAbstractSocket::SocketAddressNotAvailableError:
        errStr = "SocketAddressNotAvailableError"; break;
    case QAbstractSocket::UnsupportedSocketOperationError:
        errStr = "UnsupportedSocketOperationError"; break;
    case QAbstractSocket::OperationError:
        errStr = "OperationError"; break;
    case QAbstractSocket::TemporaryError:
        errStr = "TemporaryError"; break;
    case QAbstractSocket::UnknownSocketError:
        errStr = "UnknownSocketError"; break;
    default:
        errStr = "UnknownError";
    }

    qDebug() << tr("Error in FrameClientThread: %1.")
                   .arg(errStr);
}

void FrameClientThread::setEpoch(const QDateTime &datetime)
{
    QMutexLocker locker(m_mutex);
    if(!m_keepStreaming)
    {
        m_epoch = datetime;
        m_isEpochSet = true;

//        emit logEventWithMessage(SRC_FRMCLNT, LOG_INFO, QTime::currentTime(), FRMCLNT_EPOCH_SET,
//                                 m_epoch.toString("yyyy/MM/dd - hh:mm:ss.zzz"));
    }
//    else
//        emit logEvent(SRC_FRMCLNT, LOG_INFO, QTime::currentTime(), FRMCLNT_EPOCH_SET_FAILED);
}

QDataStream & operator << (QDataStream &o, const FrameExtd& f)
{
    return o << f.image_
             << f.index_
             << f.timestamp_
             << f.EMq_
             << f.EMv_
             << f.mask_
             << f.phaseHR_
             << f.phaseResp_
             << '\0';
}

QDataStream & operator >> (QDataStream &i, FrameExtd& f)
{
     i >> f.image_
         >> f.index_
         >> f.timestamp_
         >> f.EMq_
         >> f.EMv_
         >> f.mask_
         >> f.phaseHR_
         >> f.phaseResp_;

    return i;
}
