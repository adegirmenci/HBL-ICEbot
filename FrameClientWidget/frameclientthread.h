#ifndef FRAMECLIENTTHREAD_H
#define FRAMECLIENTTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <QSharedPointer>
#include <QDir>

#include <QNetworkInterface>
#include <QTcpSocket>
#include <QDataStream>
#include <QByteArray>
#include <QHostAddress>

#include <QMatrix4x4>
#include <Qt3DCore/QTransform>
#include <QQuaternion>
#include <QVector3D>

#include <vector>
#include <memory>
#include <valarray>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../icebot_definitions.h"
#include "../AscensionWidget/3DGAPI/ATC3DG.h"
#include "../FrmGrabWidget/frmgrabthread.h"

#define N_PHASES 500 // keep track of the last N_PHASES cardiac phases and corresponding time stamps

struct FrameExtd{
    QString image_; /*!< Image data. */
    QQuaternion EMq_; /*!< EM reading - rotation as Quaternion. */
    QVector3D EMv_; /*!< EM reading - translation. */
    float phaseHR_; /*!< Phase in the heart cycle (0.0 - 1.0). */
    float phaseResp_; /*!< Phase in the respiration cycle (0.0 - 1.0). */
    qint64 timestamp_; /*!< Timestamp, msec since some epoch. */
    int index_; /*!< Index value of Frame, indicate order of acquisition. */
    QString mask_; /*!< Image mask. */

    //! Constructor.
    explicit FrameExtd(QString img = QString(),
                       QQuaternion emq = QQuaternion(),
                       QVector3D emv = QVector3D(),
                       qint64 ts = -1,
                       float pHR = 0.f,
                       float pResp = 0.f,
                       int id = -1,
                       QString mask = QString()) :
        timestamp_(ts), phaseHR_(pHR), phaseResp_(pResp), index_(id)
    {
        image_ = img;
        EMq_ = emq;
        EMv_ = emv;
        mask_ = mask;
    }
    //! Destructor
    ~FrameExtd() {
    }
};

struct EMreading{
    QTime timeStamp;
    int sensorID;
    DOUBLE_POSITION_MATRIX_TIME_Q_RECORD data;

    explicit EMreading(QTime t_ = QTime(),
                       int s_ = -1,
                       DOUBLE_POSITION_MATRIX_TIME_Q_RECORD r_ = DOUBLE_POSITION_MATRIX_TIME_Q_RECORD()) :
        timeStamp(t_), sensorID(s_)
    {
        data = r_;
    }
    ~EMreading(){}
};

Q_DECLARE_METATYPE(FrameExtd)

class FrameClientThread : public QObject
{
    Q_OBJECT
public:
    explicit FrameClientThread(QObject *parent = 0);
    ~FrameClientThread();

    friend QDataStream & operator << (QDataStream &o, const FrameExtd& f);
    friend QDataStream & operator >> (QDataStream &i, FrameExtd& f);

signals:
    void statusChanged(int event);
    void finished(); // emit upon termination
    void tcpError(QAbstractSocket::SocketError error);

public slots:
    void setEpoch(const QDateTime &datetime); // set Epoch
    void initializeFrameClient();
    void sendFrame(); // change to Frame type
    void receiveFrame(std::shared_ptr<Frame> frame);
    void receiveEMreading(QTime timeStamp,
                          int sensorID,
                          DOUBLE_POSITION_MATRIX_TIME_Q_RECORD data);
    void receiveLatestEMreading(std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> readings);
    void receive_T_CT(std::vector<double> T_BB_CT, double time);
    void receivePhase(qint64 timeStamp, double phase);
    void handleTcpError(QAbstractSocket::SocketError error);
    void connectedToHost();
    void disconnectedFromHost();
    const QHostAddress getServerAddress() { return m_serverAddress; }
    const quint16 getServerPort() { return m_serverPort; }

private:
    QString saveFrame(std::shared_ptr<Frame>);

    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Epoch for time stamps
    // During initializeFrameClient(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if Frame Client is ready
    // True if initializeFrameClient was successful
    bool m_isReady;

    // Flag to tell that we are still streaming
    bool m_keepStreaming;

    // Flag to abort actions (e.g. initialize, acquire, etc.)
    bool m_abort;

    int m_frameCount; // keep a count of number of transmitted frames

    // server info
    QHostAddress m_serverAddress;
    quint16 m_serverPort;

    QTcpSocket *m_TcpSocket;

    FrameExtd m_currExtdFrame;
    std::shared_ptr<Frame> m_currFrame;
    double m_currFramePhase;
    EMreading m_currBird;

    std::vector<double> m_cardiacPhases;
    std::vector<qint64> m_phaseTimes;
};

#endif // FRAMECLIENTTHREAD_H
