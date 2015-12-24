#include "frmgrab.h"
#include "ui_frmgrab.h"

FrmGrab::FrmGrab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FrmGrab)
{
    ui->setupUi(this);

    m_fg = NULL;
    m_isInitialized = false;
    m_isConnected = false;

    m_numFramesWritten = 0;
}

FrmGrab::~FrmGrab()
{
    frmGrabDisconnect();

    delete ui;
}

bool FrmGrab::frmGrabConnect()
{
    m_isConnected = false;

    m_fg = FrmGrabLocal_Open();

    if(!(m_fg == NULL))
    {
        m_isConnected = FrmGrab_Start(m_fg); // set global variable to true to indicate frmGrabber is initialized
        if(m_isConnected)
            qDebug() << "FrmGrab started.";
        else
            qDebug() << "FrmGrab failed to start.";
    }
    else
    {
        ui->statusLineEdit->setText("Failed to open device!");
        qDebug() << "Failed to open device!";
    }

    return m_isConnected;
}

bool FrmGrab::SetFileName(const QString &fileName)
{
    QMutexLocker locker(&m_mutex); // lock mutex
    // mutex unlocks when locker goes out of scope

    if(!m_txtFile.isOpen()) // file is not open
    {
        m_txtFile.setFileName(fileName);
        return true;
    }
    else // file is already open
    {
        return false;
    }
}

bool FrmGrab::frmGrabInitialize(const char *location)
{
    bool state = false;

    if(m_isConnected && (!m_isInitialized) ) // successfully connected and not alreadxy initialized
    {
        qDebug() << "Initializing file.";

        QString t = getCurrDateTimeFileStr();   // get date time now

        m_saveDir.clear(); // clear filename
        m_saveDir.append("RECORDED_FRMGRAB/").append(t); // set the directory

        // check if the directory exists, if not, create it
        QDir dir;
        if( ! dir.exists(m_saveDir) ){
            if( dir.mkpath(m_saveDir) )
            {
                qDebug() << "Folder created: " << m_saveDir;
                state = true;
            }
            else
                qDebug() << "Folder creation failed: " << m_saveDir;
        }

        // set file name
        m_imgFname_pre = getCurrDateTimeFileStr();
        m_txtFname = m_saveDir;
        m_txtFname.append("/");
        m_txtFname.append(m_imgFname_pre);
        m_txtFname.append("_");
        m_txtFname.append(location);
        m_txtFname.append(".txt");
        state = SetFileName(m_txtFname);

        //Open file for write and append
        if(!m_txtFile.open(QIODevice::WriteOnly | QIODevice::Append))
        {
            qDebug() << "File could not be opened: " << m_txtFile.fileName();
            state = false;
        }
        else
        {
            m_textStream.setDevice(&m_txtFile);
            m_textStream << "File opened at: " << getCurrDateTimeStr() << '\n';

            m_textStream << location << endl; // write to file where initialize is called from
            m_textStream << "File Name \t Time (s)" << endl; // write header

            state = state && true;

            qDebug() << "File opened:" << m_txtFile.fileName();
        }
    }

    m_isInitialized = state;

    qDebug() << "State is " << state;

    return state; // return false if already initialized or if the file couldn't be opened
}

bool FrmGrab::grabFrame()
{
    bool state = false;

    if(m_fg == NULL)
    {
        ui->statusLineEdit->setText("Failed to read device.");
    }
    else
    {
        std::shared_ptr<Frame> newFrame( new Frame );

        newFrame->timestamp_ = m_epoch.elapsed()/1000.; // get time stamp

        m_frame = FrmGrab_Frame(m_fg, V2U_GRABFRAME_FORMAT_BGR24, NULL);
        m_imgSize = cv::Size(m_frame->mode.width, m_frame->mode.height);

        //qDebug() << m_imgSize.width  << " x " << m_imgSize.height;

        if (m_frame)
        {
            m_src = cv::Mat(m_imgSize,CV_8UC3);

            m_src.data = (uchar*)(m_frame->pixbuf);

            cv::cvtColor(m_src, m_dst, CV_BGR2GRAY);

            newFrame->image_ = m_dst; // add frame to container

            FrmGrab_Release(m_fg, m_frame);

            newFrame->index_ = m_numFramesWritten;
            m_numFramesWritten++;

            QMutexLocker locker(&m_mutex);
            m_frmList.push_back( newFrame );

            state = true;
        }
    }

    return state;
}

bool FrmGrab::saveFrame(std::shared_ptr<Frame> frm)
{
    bool state = false;

    if(m_isInitialized) // not already initialized
    {
        QString m_imgFname = m_saveDir; // contains file name of frame
        m_imgFname.append("/").append(m_imgFname_pre);

        // populate m_imgFname with index
        m_imgFname.append( QString("_%1.jpg").arg(frm->index_) );

        // save frame
        //state = frame->image_.save(m_imgFname, "JPG", 100);
        cv::imwrite(m_imgFname.toStdString().c_str(), frm->image_ ); // write frame

        // output to text
        if(m_txtFile.isOpen())
        {
            m_textStream << m_imgFname_pre << QString("_%1.jpg").arg(frm->index_) << "\t"
                         << QString::number(frm->timestamp_, 'f', 6) << '\n';
            m_textStream.flush();
            state = true;
        }
    }
    // save image to file using QtConcurrent calls

    return state;
}

bool FrmGrab::frmGrabDisconnect()
{
    FrmGrab_Close(m_fg);

//    if(m_fg == NULL)
//        return true;
//    else
//        return false;

    return true;
}

bool FrmGrab::setEpoch(const QTime &epoch)
{
    m_epoch = epoch;

    return true;
}

// ------------------------------ //
// GUI INTERACTION IMPLEMENTATION //
// ------------------------------ //

void FrmGrab::on_connectButton_clicked()
{
    if( frmGrabConnect() )
    {
        ui->connectButton->setEnabled(false);
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(true);
        ui->statusLineEdit->setText("Connected.");
    }
    else
        ui->statusLineEdit->setText("Failed to connect.");
}

void FrmGrab::on_initButton_clicked()
{
    if( frmGrabInitialize("GUI_Button") )
    {
        ui->initButton->setEnabled(false);
        ui->statusLineEdit->setText("Initialized.");
        ui->acquireButton->setEnabled(true);
        ui->saveButton->setEnabled(true);
    }
    else
    {
        ui->statusLineEdit->setText("Failed to initialize.");
        ui->acquireButton->setEnabled(false);
        ui->saveButton->setEnabled(false);
    }
}

void FrmGrab::on_disconnectButton_clicked()
{
    if( frmGrabDisconnect() )
    {
        ui->connectButton->setEnabled(true);
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(false);
        ui->acquireButton->setEnabled(false);
        ui->saveButton->setEnabled(false);
        ui->statusLineEdit->setText("Disconnected.");
    }
    else
        ui->statusLineEdit->setText("Failed to disconnect.");

}

void FrmGrab::on_acquireButton_clicked()
{
    for( int i = 0; i < 120; i++)
        grabFrame();

    if( grabFrame() )
        ui->statusLineEdit->setText("Frame grabbed.");
    else
        ui->statusLineEdit->setText("Failed to grab frame.");
}


void FrmGrab::on_saveButton_clicked()
{
    while( m_frmList.size() > 0)
    {
        if( saveFrame(m_frmList.front()) )
        {
            ui->statusLineEdit->setText("Frame saved.");
            QMutexLocker locker(&m_mutex);
            m_frmList.pop_front();
        }
        else
            ui->statusLineEdit->setText("Failed to save frame.");
    }

    ui->statusLineEdit->setText("No more frames to write.");
}


// ------------------------------------- //
// GUI INTERACTION IMPLEMENTATION - DONE //
// ------------------------------------- //


inline const QString getCurrDateTimeFileStr()
{
    return QDateTime::currentDateTime().toString("ddMMyyyy_hhmmsszzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("dd/MM/yyyy - hh:mm:ss.zzz");
}
