#ifndef ASCENSIONEM_H
#define ASCENSIONEM_H

#include <QWidget>
#include <QTime>
#include <QElapsedTimer>

#include "3DGAPI/ATC3DG.h"

class CSystem
{
public:
    SYSTEM_CONFIGURATION	m_config;
};

class CSensor
{
public:
    SENSOR_CONFIGURATION	m_config;
};

class CXmtr
{
public:
    TRANSMITTER_CONFIGURATION	m_config;
};


namespace Ui {
class AscensionEM;
}

class AscensionEM : public QWidget
{
    Q_OBJECT

public:
    explicit AscensionEM(QWidget *parent = 0);
    ~AscensionEM();

private slots:
    void on_initButton_clicked();

    void on_disconnectButton_clicked();

    void on_acquireButton_clicked();

private:
    Ui::AscensionEM *ui;

    CSystem			m_ATC3DG;
    CSensor			*m_pSensor;
    CXmtr			*m_pXmtr;
    int				m_errorCode;
    int				m_i;
    int				m_sensorID;
    short			m_id;
    int				m_records;// = 100;
    char			m_output[256];
    int				m_numberBytes;

    void errorHandler(int error);

    bool initializeEM();
    bool disconnectEM();
};

#endif // ASCENSIONEM_H
