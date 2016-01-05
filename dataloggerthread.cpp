#include "dataloggerthread.h"

DataLoggerThread::DataLoggerThread(QObject *parent) : QObject(parent)
{
    qRegisterMetaType< std::vector<int> >("std::vector<int>");
    qRegisterMetaType< std::vector<QString> >("std::vector<QString>");

    m_isEpochSet = false;
    m_isReady = false;
    m_keepLogging = false;

    m_mutex = new QMutex(QMutex::Recursive);

    for (int i = 0; i < DATALOG_NUM_FILES; i++)
    {
        m_files.push_back(std::shared_ptr<QFile>(new QFile()) );
        m_streams.push_back(std::shared_ptr<QTextStream>(new QTextStream()));
    }

    //qRegisterMetaType<DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD>("DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD");

}

DataLoggerThread::~DataLoggerThread()
{


    qDebug() << "Ending DataLoggerThread.";

    emit finished();
}

void DataLoggerThread::setRootDirectory(QString &dir)
{
    QMutexLocker locker(m_mutex);

    m_rootDirectory = dir;
}

void DataLoggerThread::initializeDataLogger(std::vector<int> enableMask, std::vector<QString> fileNames)
{
    QMutexLocker locker(m_mutex);

    emit statusChanged(DATALOG_INITIALIZE_BEGIN);

    if(enableMask.size() == DATALOG_NUM_FILES) // within bounds
    {
        for (size_t i = 0; i < enableMask.size(); i++) // iterate through items
        {
            if(enableMask[i]) // user wants to log this
            {
                bool status = true;

//                // check if the directory exists, if not, create it
//                QDir dir;
//                if( ! dir.exists(fileNames[i]) )
//                {
//                    if( dir.mkpath(fileNames[i]) )
//                    {
//                        status = true;
//                        qDebug() << "Folder created: " << fileNames[i];
//                    }
//                    else
//                    {
//                        status = false;
//                        qDebug() << "Folder creation failed: " << fileNames[i];

//                        emit fileStatusChanged(i, DATALOG_FILE_ERROR);
//                    }
//                }

                if(status) // no issues with folder creation
                {
                    m_files[i]->setFileName(fileNames[i]); // set filename of QFile

                    if(m_files[i]->open(QIODevice::WriteOnly | QIODevice::Append)) // opened successfuly
                    {

                        m_streams[i]->setDevice(&(*m_files.at(i)));
                        (*m_streams[i]) << "File opened at: " << getCurrDateTimeStr() << '\n';

                        emit fileStatusChanged(i, DATALOG_FILE_OPENED);
                    }
                    else // can't open
                    {
                        qDebug() << "File could not be opened: " << m_files[i]->fileName();

                        status = false;
                        emit fileStatusChanged(i, DATALOG_FILE_ERROR);
                    }
                }
            }
            else // user doesn't want
            {
                emit fileStatusChanged(i, DATALOG_FILE_CLOSED);
            }
        }
        emit statusChanged(DATALOG_INITIALIZED);
    }
    else
        emit statusChanged(DATALOG_INITIALIZE_FAILED); //error!
}


void DataLoggerThread::logEMdata(QTime timeStamp,
                                 int sensorID,
                                 DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD data)
{
    QDateTime ts;
    ts.setMSecsSinceEpoch(data.time*1000);

    // Data Format
    // | Sensor ID | Time Stamp | x | y | z | r11 | r12 | ... | r33 |

    QString output;
    output.append(QString("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\t%10\t%11\t%12\t%13\t%14\n")
                     .arg(sensorID)
                     .arg(QString::number(data.x,'f',m_prec))
                     .arg(QString::number(data.y,'f',m_prec))
                     .arg(QString::number(data.z,'f',m_prec))
                     .arg(QString::number(data.s[0][0],'f',m_prec))
                     .arg(QString::number(data.s[0][1],'f',m_prec))
                     .arg(QString::number(data.s[0][2],'f',m_prec))
                     .arg(QString::number(data.s[1][0],'f',m_prec))
                     .arg(QString::number(data.s[1][1],'f',m_prec))
                     .arg(QString::number(data.s[1][2],'f',m_prec))
                     .arg(QString::number(data.s[2][0],'f',m_prec))
                     .arg(QString::number(data.s[2][1],'f',m_prec))
                     .arg(QString::number(data.s[2][2],'f',m_prec))
                     .arg(QString::number(data.time*1000,'f',m_prec)));
                     //.arg(ts.toString("hh:mm:ss:zzz")));

    QMutexLocker locker(m_mutex);

    if(m_files[DATALOG_EM_ID]->isOpen())
    {
        (*m_streams[DATALOG_EM_ID]) << output;
    }
    else
        qDebug() << "File is closed.";
}

void DataLoggerThread::logEPOSEvent(int logType, QTime timeStamp, int eventID)
{
    // Data Format
    // | Time Stamp | Log Type | Source | eventID |

    QString output = timeStamp.toString("HH:mm:ss.zzz");

    switch(logType)
    {
    case LOG_INFO:
        output.append(" INFO ");
        break;
    case LOG_WARNING:
        output.append(" WARNING ");
        break;
    case LOG_ERROR:
        output.append(" ERROR ");
        break;
    case LOG_FATAL:
        output.append(" FATAL ");
        break;
    default:
        output.append(" UNKNOWN ");
        break;
    }

    output.append("EPOS ");

    output.append(QString("%1\n").arg(eventID));


    QMutexLocker locker(m_mutex);

    if(m_files[DATALOG_Log_ID]->isOpen())
    {
        (*m_streams[DATALOG_Log_ID]) << output;
    }
    else
        qDebug() << "File is closed.";
}

void DataLoggerThread::startLogging()
{
    QMutexLocker locker(m_mutex);

    // make connections

    emit statusChanged(DATALOG_LOGGING_STARTED);

}

void DataLoggerThread::stopLogging()
{
    QMutexLocker locker(m_mutex);

    // sever connections

    emit statusChanged(DATALOG_LOGGING_STOPPED);
}

void DataLoggerThread::logNote(QTime timeStamp, QString note)
{
    if(m_files[DATALOG_Note_ID]->isOpen())
    {
        (*m_streams[DATALOG_Note_ID]) << "[" << timeStamp.toString("HH:mm:ss.zzz") << "] " << note << endl;

        emit fileStatusChanged(DATALOG_Note_ID, DATALOG_FILE_DATA_LOGGED);
    }
}

void DataLoggerThread::closeLogFile(const unsigned short fileID)
{
    QMutexLocker locker(m_mutex);

    if(m_files[fileID]->isOpen())
    {
        m_streams[fileID]->flush();
        m_files[fileID]->close();

        emit fileStatusChanged(fileID, DATALOG_FILE_CLOSED);
    }
}

void DataLoggerThread::closeLogFiles()
{
    QMutexLocker locker(m_mutex);

    for (unsigned short i = 0; i < DATALOG_NUM_FILES; i++) // iterate through items
    {
        closeLogFile(i);
    }
    emit statusChanged(DATALOG_CLOSED);
}

// Helper functions

inline const QString getCurrDateTimeFileStr()
{
    return QDateTime::currentDateTime().toString("ddMMyyyy_hhmmsszzz");
}

inline const QString getCurrDateTimeStr()
{
    return QDateTime::currentDateTime().toString("dd/MM/yyyy - hh:mm:ss.zzz");
}
