#ifndef EPOS2_H
#define EPOS2_H

#include <QWidget>

#include "MaxonLibs/Definitions.h"
#include <stdio.h>
#include <Windows.h>

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

    void moveMotor(long targetPos, WORD motorID, BOOL moveAbs);
    void haltMotor(WORD motorID);
    
    BOOL m_motorsEnabled;

private slots:
    void on_connectionButtonBox_accepted();

    void on_connectionButtonBox_rejected();

private:
    Ui::epos2 *ui;

    BOOL OpenDevice();
    BOOL ShowErrorInformation(DWORD p_ulErrorCode);

    __int8 m_bMode;
    BOOL m_oImmediately;
    BOOL m_oInitialisation;
    BOOL m_oUpdateActive;
    DWORD m_ulErrorCode;
    DWORD m_ulProfileAcceleration;
    DWORD m_ulProfileDeceleration;
    DWORD m_ulProfileVelocity;
    HANDLE m_KeyHandle;
    WORD m_usNodeId;

};

#endif // EPOS2_H
