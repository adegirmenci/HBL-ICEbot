#ifndef EPOS2_H
#define EPOS2_H

#include <QWidget>
#include <QSharedPointer>
#include <QList>
#include <QElapsedTimer>

#include "MaxonLibs/Definitions.h"
#include <stdio.h>
#include <Windows.h>

#define EPOS_VELOCITY 5000
#define EPOS_ACCEL 8000
#define EPOS_DECEL 8000

// node ID's for motors
#define TRANS_MOTOR_ID 1
#define PITCH_MOTOR_ID 2
#define YAW_MOTOR_ID 3
#define ROLL_MOTOR_ID 4

// indices for the motor in list
#define TRANS 0
#define PITCH 1
#define YAW 2
#define ROLL 3

struct eposMotor
{
    __int8 m_bMode;
    WORD m_nodeID; // motor ID
    DWORD m_ulProfileAcceleration; // acceleration value
    DWORD m_ulProfileDeceleration; // deceleration value
    DWORD m_ulProfileVelocity; // velocity value
    BOOL m_enabled;

    long m_lActualValue; // volatile?
    long m_lStartPosition; // volatile?
    long m_lTargetPosition; // volatile?

    long m_maxQC; // upper limit
    long m_minQC; // lower limit
};

namespace Ui {
class epos2;
}

class epos2 : public QWidget
{
    Q_OBJECT
    
public:
    explicit epos2(QWidget *parent = 0);
    ~epos2();

    long m_lActualValue;
    long m_lStartPosition;
    long m_lTargetPosition;

    void moveMotor(long targetPos, QSharedPointer<eposMotor> mot, BOOL moveAbsOrRel);
    void getMotorQC(QSharedPointer<eposMotor> mot);
    void haltMotor(QSharedPointer<eposMotor> mot);
    
    BOOL m_motorsEnabled;

private slots:
    void on_connectionButtonBox_accepted();

    void on_connectionButtonBox_rejected();

    void on_enableNodeButton_clicked();

    void on_disableNodeButton_clicked();

    void on_moveAbsButton_clicked();

    void on_nodeIDcomboBox_currentIndexChanged(int index);

    void on_moveRelButton_clicked();

    void on_haltButton_clicked();

    void on_homingButton_clicked();

private:
    Ui::epos2 *ui;

    BOOL OpenDevice();
    BOOL InitMotor(QSharedPointer<eposMotor> mot);
    BOOL DisableMotor(QSharedPointer<eposMotor> mot);
    BOOL ShowErrorInformation(DWORD p_ulErrorCode);

    BOOL m_oImmediately;
    BOOL m_oInitialisation;
    BOOL m_oUpdateActive;
    DWORD m_ulErrorCode;
    HANDLE m_KeyHandle;

    QList<QSharedPointer<eposMotor>> m_motors;
    eposMotor m_transMotor;
    eposMotor m_pitchMotor;
    eposMotor m_yawMotor;
    eposMotor m_rollMotor;
};

#endif // EPOS2_H
