#include "epos2.h"
#include "ui_epos2.h"

epos2::epos2(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::epos2)
{
    ui->setupUi(this);
    //user has to open connection to motors using the GUI
    m_motorsEnabled = FALSE;
}

epos2::~epos2()
{
    //VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
    VCS_CloseAllDevices(&m_ulErrorCode);
    printf("EPOS closed successfully.\n");
    delete ui;
}

BOOL epos2::OpenDevice() //(WORD motorID)
{
    ui->outputText->append("Opening connection to EPOS...");

    m_bMode = 0;
    m_KeyHandle = 0;
    m_lActualValue = 0;
    m_lStartPosition = 0;
    m_lTargetPosition = 0;

    //Settings
    m_oImmediately = TRUE;
    m_oUpdateActive = FALSE;
    WORD motorID = 1;
    m_usNodeId = motorID;

    //const char OMD_PROFILE_POSITION_MODE(0x01);

    HANDLE hNewKeyHandle;

    if(m_KeyHandle)
    {
        if(m_KeyHandle) VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
        m_KeyHandle = 0;
    }

    // keep track of node initialization errors
    BOOL motSuccess[2];
    motSuccess[0] = FALSE;
    motSuccess[1] = FALSE;

    hNewKeyHandle = VCS_OpenDeviceDlg(&m_ulErrorCode);
    // Set properties of each motor
    for(int i = 0; i < 2; i++)
    {
        motorID = i+1;
        m_usNodeId = motorID;
        ui->outputText->append(QString("Connecting to Node %1...").arg(motorID));
        if(hNewKeyHandle)
        {
            //Close Previous Device
            //if(m_KeyHandle) VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
            m_KeyHandle = hNewKeyHandle;

            //Clear Error History
            if(VCS_ClearFault(m_KeyHandle, m_usNodeId, &m_ulErrorCode))
            {
                //Read Operation Mode
                if(VCS_GetOperationMode(m_KeyHandle, m_usNodeId, &m_bMode, &m_ulErrorCode))
                {
                    //Read Position Profile Objects
                    if(VCS_GetPositionProfile(m_KeyHandle, m_usNodeId, &m_ulProfileVelocity, &m_ulProfileAcceleration, &m_ulProfileDeceleration, &m_ulErrorCode))
                    {
                        //Write Profile Position Mode
                        if(VCS_SetOperationMode(m_KeyHandle, m_usNodeId, OMD_PROFILE_POSITION_MODE, &m_ulErrorCode))
                        {
                            //Write Profile Position Objects
                            if(VCS_SetPositionProfile(m_KeyHandle, m_usNodeId, 2000, 8000, 8000, &m_ulErrorCode))
                            {
                                //Read Actual Position
                                if(VCS_GetPositionIs(m_KeyHandle, m_usNodeId, &m_lStartPosition, &m_ulErrorCode))
                                {
                                    ui->outputText->append("DONE!");
                                    motSuccess[i] = TRUE;
                                }
                            }
                        }
                    }
                }
            }
            if(!motSuccess[i])
                ShowErrorInformation(m_ulErrorCode);
        }
        else
        {
            ui->outputText->append("Can't open device!\n");
            motSuccess[i] = FALSE;
        }
    }

    return (motSuccess[0] && motSuccess[1]);
}

void epos2::moveMotor(long targetPos, WORD motorID, BOOL moveAbs)
{
    m_lTargetPosition = targetPos;
    m_usNodeId = motorID;

    if(m_motorsEnabled)
    {
        if(!VCS_MoveToPosition(m_KeyHandle, m_usNodeId, m_lTargetPosition, moveAbs, m_oImmediately, &m_ulErrorCode))
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

void epos2::haltMotor(WORD motorID)
{
    m_usNodeId = motorID;

    if(!VCS_HaltPositionMovement(m_KeyHandle, m_usNodeId, &m_ulErrorCode))
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
        printf("Not enough memory to allocate buffer for error information string\n");

        return FALSE;
    }

    if(VCS_GetErrorInfo(p_ulErrorCode, pStrErrorInfo, 100))
    {
        strDescription = pStrErrorInfo;
        printf("\nMaxon: ");
        printf(strDescription);
        printf("\n");

        free(pStrErrorInfo);

        return TRUE;
    }
    else
    {
        free(pStrErrorInfo);
        printf("Error information can't be read!\n");

        return FALSE;
    }
}

void epos2::on_connectionButtonBox_accepted()
{
    if(OpenDevice())
        m_motorsEnabled = TRUE;
}

void epos2::on_connectionButtonBox_rejected()
{
    m_motorsEnabled = FALSE;
    VCS_CloseAllDevices(&m_ulErrorCode);
    ui->outputText->append("Closed.\n");
}
