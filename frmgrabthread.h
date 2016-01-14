#ifndef FRMGRABTHREAD_H
#define FRMGRABTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <QSharedPointer>

#include <vector>
#include <memory>

#include "../AscensionWidget/icebot_definitions.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/video/video.hpp"

//#include <epiphan/frmgrab/include/frmgrab.h>

/** Frame structure.
*  This will hold an image, a timestamp, and an index.
*/
struct Frame{
    cv::Mat image_; /*!< Image data. */
    qint64 timestamp_; /*!< Timestamp, msec since some epoch. */
    int index_; /*!< Index value of Frame, indicate order of acquisition. */

    //! Constructor.
    explicit Frame(cv::Mat img = cv::Mat(), qint64 ts = -1, int id = -1) :
        timestamp_(ts), index_(id)
    {
        image_ = img;
    }
    //! Destructor
    ~Frame() {
        image_.release();
    }
};

Q_DECLARE_METATYPE(std::shared_ptr<Frame>)

class FrmGrabThread : public QObject
{
    Q_OBJECT

public:
    explicit FrmGrabThread(QObject *parent = 0);
    ~FrmGrabThread();

signals:
    void statusChanged(int event);
    void imageAcquired(std::shared_ptr<Frame> frm);
    void pleaseSaveImage(std::shared_ptr<Frame> frm);
    void logData(QTime timeStamp,
                 int frameIdx,
                 QString &data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID); // FRMGRAB_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID, // FRMGRAB_EVENT_IDS
                  QString &message);
    void logError(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int errCode, // FRMGRAB_ERROR_CODES
                  QString &message);
    void finished(); // emit upon termination

public slots:
    void frmGrabConnect();
    void frmGrabInitialize(int width, int height, double fps);
    void startStream();
    void stopStream();
    void toggleLiveFeed();
    void displayFrame(std::shared_ptr<Frame> frm);
    void addSaveRequest(unsigned short numFrames);
    void frmGrabDisconnect();

    void setEpoch(const QDateTime &datetime); // set Epoch
    void grabFrame();


private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling grabFrame every xxx msecs
    QTimer *m_timer;

    // Epoch for time stamps
    // During initializeFrmGrab(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if Frame Grabber is ready
    // True if InitializeFrmGrab was successful
    bool m_isReady;

    // Flag to tell that we are still recording
    bool m_keepStreaming;

    // Flag for live feed
    bool m_showLiveFeed;

    // Flag to abort actions (e.g. initialize, acquire, etc.)
    bool m_abort;

    // Image containers
    cv::Mat m_src;
    cv::Mat m_dst;
    cv::Size m_imgSize;

    int m_frameCount; // keep a count of number of saved frames

    int m_numSaveImageRequests; // Counter to keep track of image saving

    cv::VideoCapture m_cap; // video capture device

    const QString m_winName = QString("Live Feed");

    double m_videoFPS;

};

#endif // FRMGRABTHREAD_H
