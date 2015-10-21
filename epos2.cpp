#include "epos2.h"
#include "ui_epos2.h"
#include <QDebug>

epos2::epos2(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::epos2)
{
    ui->setupUi(this);

    m_KeyHandle = 0;

    //user has to open connection to motors using the GUI
    m_motorsEnabled = FALSE;

    m_transMotor.m_nodeID = TRANS_MOTOR_ID;
    m_rollMotor.m_nodeID = ROLL_MOTOR_ID;
    m_pitchMotor.m_nodeID = PITCH_MOTOR_ID;
    m_yawMotor.m_nodeID = YAW_MOTOR_ID;

    qDebug() << "Initizializing pointers to eposMotor";

    m_motors.reserve(4);
    m_motors.push_back(QSharedPointer<eposMotor>(&m_transMotor));
    m_motors.push_back(QSharedPointer<eposMotor>(&m_rollMotor));
    m_motors.push_back(QSharedPointer<eposMotor>(&m_pitchMotor));
    m_motors.push_back(QSharedPointer<eposMotor>(&m_yawMotor));
}

epos2::~epos2()
{
    for(int i = 0; i < 4; i++)
        DisableMotor(m_motors[i]);

    m_motors.clear();

    //VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
    VCS_CloseAllDevices(&m_ulErrorCode);
    qDebug() << "EPOS closed successfully.";
    delete ui;
}

BOOL epos2::InitMotor(QSharedPointer<eposMotor> mot)
{
    mot.data()->m_bMode = 0;
    mot.data()->m_lActualValue = 0;
    mot.data()->m_lStartPosition = 0;
    mot.data()->m_lTargetPosition = 0;

    //mot.data()->m_ulProfileVelocity = EPOS_VELOCITY;
    //mot.data()->m_ulProfileAcceleration = EPOS_ACCEL;
    //mot.data()->m_ulProfileDeceleration = EPOS_DECEL;

    WORD motorID = mot.data()->m_nodeID;

    ui->outputText->append(QString("Connecting to Node %1...").arg(motorID));

    mot.data()->m_enabled = FALSE;

    if(m_KeyHandle)
    {
        //Clear Error History
        if(VCS_ClearFault(m_KeyHandle, motorID, &m_ulErrorCode))
        {
            //Enable
            if( VCS_SetEnableState(m_KeyHandle, motorID, &m_ulErrorCode) )
            {
                //Read Operation Mode
                if(VCS_GetOperationMode(m_KeyHandle, motorID,
                                        &(mot.data()->m_bMode),
                                        &m_ulErrorCode))
                {
                    //Read Position Profile Objects
                    if(VCS_GetPositionProfile(m_KeyHandle, motorID,
                                              &(mot.data()->m_ulProfileVelocity),
                                              &(mot.data()->m_ulProfileAcceleration),
                                              &(mot.data()->m_ulProfileDeceleration),
                                              &m_ulErrorCode))
                    {
                        //Write Profile Position Mode
                        if(VCS_SetOperationMode(m_KeyHandle, motorID,
                                                OMD_PROFILE_POSITION_MODE,
                                                &m_ulErrorCode))
                        {
                            //Write Profile Position Objects
                            if(VCS_SetPositionProfile(m_KeyHandle, motorID,
                                                      EPOS_VELOCITY,
                                                      EPOS_ACCEL,
                                                      EPOS_DECEL,
                                                      &m_ulErrorCode))
                            {
                                //Read Actual Position
                                if(VCS_GetPositionIs(m_KeyHandle, motorID,
                                                     &(mot.data()->m_lStartPosition),
                                                     &m_ulErrorCode))
                                {
                                    ui->outputText->append("DONE!");
                                    mot.data()->m_enabled = TRUE;
                                }
                            }
                        }
                    }
                }
            }
        }
        if(!mot.data()->m_enabled)
        {
            ui->outputText->append("Can't connect to motor!");
            ShowErrorInformation(m_ulErrorCode);
        }
    }
    else
    {
        ui->outputText->append("Can't open device!");
        mot.data()->m_enabled = FALSE;
    }
    return mot.data()->m_enabled;
}

BOOL epos2::DisableMotor(QSharedPointer<eposMotor> mot)
{
    WORD nodeId = mot.data()->m_nodeID;
    mot.data()->m_enabled = FALSE;
    return VCS_SetDisableState(m_KeyHandle, nodeId, &m_ulErrorCode);
}

BOOL epos2::OpenDevice() //(WORD motorID)
{
    ui->outputText->append("Opening connection to EPOS...");

    if(m_KeyHandle)
    {
        //Close Previous Device
        VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
        m_KeyHandle = 0;
    }
    else
        m_KeyHandle = 0;

    //Settings
    m_oImmediately = TRUE;
    m_oUpdateActive = FALSE;

    HANDLE hNewKeyHandle;

    hNewKeyHandle = VCS_OpenDeviceDlg(&m_ulErrorCode);
    if(hNewKeyHandle)
        m_KeyHandle = hNewKeyHandle;
    else
        return FALSE;

    // Set properties of each motor
    BOOL initSuccess = TRUE;
    initSuccess = initSuccess && InitMotor(m_motors[TRANS]);
    initSuccess = initSuccess && InitMotor(m_motors[ROLL]);
    initSuccess = initSuccess && InitMotor(m_motors[PITCH]);
    initSuccess = initSuccess && InitMotor(m_motors[YAW]);

    return initSuccess;
}

void epos2::moveMotor(long targetPos, QSharedPointer<eposMotor> mot, BOOL moveAbs)
{
    mot.data()->m_lTargetPosition = targetPos;
    WORD usNodeId = mot.data()->m_nodeID;

    if(mot.data()->m_enabled)
    {
        if(!VCS_MoveToPosition(m_KeyHandle, usNodeId, targetPos, moveAbs, m_oImmediately, &m_ulErrorCode))
        {
            ShowErrorInformation(m_ulErrorCode);
        }
    }

//    if(VCS_GetPositionIs(m_KeyHandle, m_usNodeId, &m_lStartPosition, &m_ulErrorCode))
//    {
//        if(!VCS_MoveToPosition(m_KeyHandle, m_usNodeId, m_lTargetPosition, moveAbs, m_oImmediately, &m_ulErrorCode))
//        {
//            ShowErrorInformation(m_ulErrorCode);
//        }
//    }
}

void epos2::haltMotor(QSharedPointer<eposMotor> mot)
{
    WORD usNodeId = mot.data()->m_nodeID;

    if(!VCS_HaltPositionMovement(m_KeyHandle, usNodeId, &m_ulErrorCode))
    {
        ShowErrorInformation(m_ulErrorCode);
    }
}

BOOL epos2::ShowErrorInformation(DWORD p_ulErrorCode)
{
    char* pStrErrorInfo;
    const char* strDescription;

    if((pStrErrorInfo = (char*)malloc(100)) == NULL)
    {
        qDebug() << "Not enough memory to allocate buffer for error information string.";

        return FALSE;
    }

    if(VCS_GetErrorInfo(p_ulErrorCode, pStrErrorInfo, 100))
    {
        strDescription = pStrErrorInfo;
        qDebug() << "Maxon: " << strDescription;

        free(pStrErrorInfo);

        return TRUE;
    }
    else
    {
        free(pStrErrorInfo);
        qDebug() << "Error information can't be read!";

        return FALSE;
    }
}

void epos2::on_connectionButtonBox_accepted()
{
    if(OpenDevice())
    {
        m_motorsEnabled = TRUE;
        ui->enableNodeButton->setEnabled(false);
        ui->disableNodeButton->setEnabled(true);
    }
    else
        m_motorsEnabled = FALSE;
}

void epos2::on_connectionButtonBox_rejected()
{
    m_motorsEnabled = FALSE;
    VCS_CloseAllDevices(&m_ulErrorCode);
    ui->outputText->append("Closed.\n");
    ui->enableNodeButton->setEnabled(false);
    ui->disableNodeButton->setEnabled(false);
}

void epos2::on_enableNodeButton_clicked()
{
    int selection = ui->nodeIDcomboBox->currentIndex();
    if(InitMotor(m_motors[selection]))
    {
        ui->enableNodeButton->setEnabled(false);
        ui->disableNodeButton->setEnabled(true);
    }
}

void epos2::on_disableNodeButton_clicked()
{
    int selection = ui->nodeIDcomboBox->currentIndex();
    if(DisableMotor(m_motors[selection]))
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
    }
}

void epos2::on_moveAbsButton_clicked()
{
    int selection = ui->nodeIDcomboBox->currentIndex();
    long counts = (long)ui->targetQClineEdit->value();
    qDebug() << "Moving motor " << m_motors[selection].data()->m_nodeID << " to " << counts;
    moveMotor(counts, m_motors[selection], true);
}

void epos2::on_moveRelButton_clicked()
{
    int selection = ui->nodeIDcomboBox->currentIndex();
    long counts =  (long)ui->targetQClineEdit->value();
    moveMotor(counts, m_motors[selection], false);
}

void epos2::on_nodeIDcomboBox_currentIndexChanged(int index)
{
    if(m_motors[index].data()->m_enabled)
    {
        ui->enableNodeButton->setEnabled(false);
        ui->disableNodeButton->setEnabled(true);
    }
    else
    {
        ui->enableNodeButton->setEnabled(true);
        ui->disableNodeButton->setEnabled(false);
    }
}


void epos2::on_haltButton_clicked()
{
    int selection = ui->nodeIDcomboBox->currentIndex();
    haltMotor(m_motors[selection]);
}

void epos2::on_homingButton_clicked()
{
    long homepos[4]={0, 0, 0, 0};

    // set a lower speed
    for(int i = 0; i < 4; i++)
    {
        WORD nodeID = m_motors[i].data()->m_nodeID;
        if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                   1400,2000,2000,&m_ulErrorCode))
        {
            ShowErrorInformation(m_ulErrorCode);
        }
    }

    for(int i = 0; i < 4; i++)
    {
        WORD nodeID = m_motors[i].data()->m_nodeID;
        if(!VCS_MoveToPosition(m_KeyHandle,nodeID,homepos[i],true,true,&m_ulErrorCode))
        {
            ShowErrorInformation(m_ulErrorCode);
        }
    }

    //TODO: reset the motor limits since roll is back to zero
    //------

    // reset the profile
    for(int i = 0; i < 4; i++)
    {
        WORD nodeID = m_motors[i].data()->m_nodeID;
        if(!VCS_SetPositionProfile(m_KeyHandle,nodeID,
                                   EPOS_VELOCITY,EPOS_ACCEL,EPOS_DECEL,
                                   &m_ulErrorCode))
        {
            ShowErrorInformation(m_ulErrorCode);
        }
    }
}
