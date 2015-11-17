#ifndef FRMGRAB_H
#define FRMGRAB_H

#include <QWidget>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QImage>
#include <list>
#include <memory>
#include <QTime>
#include <QDir>
#include <QDebug>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <epiphan/include/v2u_defs.h>
#include <epiphan/frmgrab/include/frmgrab.h>

//#include <v2u_defs.h>
//#include <frmgrab.h>

//#include "epiphan/include/v2u_defs.h"
//#include "epiphan/frmgrab/include/frmgrab.h"

/** Frame structure.
*  This will hold an image, a timestamp, and an index.
*/
struct Frame{
    cv::Mat image_; /*!< Image data. */
    double timestamp_; /*!< Timestamp, msec since some epoch. */
    int index_; /*!< Index value of Frame, indicate order of acquisition. */

    //! Constructor.
    Frame()
    {
        image_ = NULL;
        timestamp_ = -1.;
        index_ = -1;
    }
    Frame(cv::Mat& img, double ts, int id)
    {
        image_ = img;
        timestamp_ = ts;
        index_ = id;
    }
};

namespace Ui {
class FrmGrab;
}

class FrmGrab : public QWidget
{
    Q_OBJECT

public:
    explicit FrmGrab(QWidget *parent = 0);

    bool setEpoch(const QTime &epoch);

    ~FrmGrab();

private slots:
    void on_connectButton_clicked();

    void on_initButton_clicked();

    void on_disconnectButton_clicked();

    void on_acquireButton_clicked();

    void on_saveButton_clicked();

private:
    Ui::FrmGrab *ui;

    // File names for frame grabber use
    QString m_saveDir; // the directory where files will be saved
    QString m_txtFname; // contains file name of each frame and its timestamp
    QString m_imgFname_pre; // filename prepend
    // File for time stamps : open file for output, delete content if already exists
    QFile m_txtFile;
    QTextStream m_textStream;

    bool m_isConnected;
    bool m_isInitialized;
    int m_numFramesWritten;

    QTime m_epoch; // when was the GUI started - can be set externally

    //cv::VideoCapture m_frmGrab_cap;

    FrmGrabber* m_fg;
    V2U_GrabFrame2* m_frame;

    cv::Mat m_src; // Image containers
    cv::Mat m_dst;
    cv::Size m_imgSize;

    std::list< std::shared_ptr<Frame> > m_frmList;

    // DO NOT use m_mutex.lock()
    // INSTEAD use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex m_mutex;

    bool frmGrabConnect();
    bool SetFileName(const QString &fileName);
    bool frmGrabInitialize(const char* location);
    bool grabFrame();
    bool saveFrame(std::shared_ptr<Frame> frm);
    bool frmGrabDisconnect();

};

inline const QString getCurrDateTimeFileStr();
inline const QString getCurrDateTimeStr();

#endif // FRMGRAB_H
