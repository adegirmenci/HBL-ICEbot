#include "ascensionem.h"
#include "ui_ascensionem.h"

AscensionEM::AscensionEM(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AscensionEM)
{
    ui->setupUi(this);

    m_records = 1000;
}

AscensionEM::~AscensionEM()
{
    if( m_id != -1 )
        disconnectEM();

    delete ui;
}

bool AscensionEM::initializeEM()
{
    bool status = true;

    // Initialize the ATC3DG driver and DLL
    // It is always necessary to first initialize the ATC3DG "system". By
    // "system" we mean the set of ATC3DG cards installed in the PC. All cards
    // will be initialized by a single call to InitializeBIRDSystem(). This
    // call will first invoke a hardware reset of each board. If at any time
    // during operation of the system an unrecoverable error occurs then the
    // first course of action should be to attempt to Recall InitializeBIRDSystem()
    // if this doesn't restore normal operating conditions there is probably a
    // permanent failure - contact tech support.
    // A call to InitializeBIRDSystem() does not return any information.
    //
    ui->statusLineEdit->setText("Initializing...");
    ui->outputTextEdit->appendPlainText("Initializing...This will take a minute.");

    QWidget::repaint();

    m_errorCode = InitializeBIRDSystem();
    if(m_errorCode!=BIRD_ERROR_SUCCESS)
    {
        errorHandler(m_errorCode);
        return status = false;
    }

    // GET SYSTEM CONFIGURATION
    //
    // In order to get information about the system we have to make a call to
    // GetBIRDSystemConfiguration(). This call will fill a fixed size structure
    // containing amongst other things the number of boards detected and the
    // number of sensors and transmitters the system can support (Note: This
    // does not mean that all sensors and transmitters that can be supported
    // are physically attached)
    //
    m_errorCode = GetBIRDSystemConfiguration(&m_ATC3DG.m_config);
    if(m_errorCode!=BIRD_ERROR_SUCCESS)
    {
        errorHandler(m_errorCode);
        return status = false;
    }

    // GET SENSOR CONFIGURATION
    //
    // Having determined how many sensors can be supported we can dynamically
    // allocate storage for the information about each sensor.
    // This information is acquired through a call to GetSensorConfiguration()
    // This call will fill a fixed size structure containing amongst other things
    // a status which indicates whether a physical sensor is attached to this
    // sensor port or not.
    //
    m_pSensor = new CSensor[m_ATC3DG.m_config.numberSensors];
    for(m_i = 0; m_i < m_ATC3DG.m_config.numberSensors; m_i++)
    {
        m_errorCode = GetSensorConfiguration(m_i, &(m_pSensor + m_i)->m_config);
        if(m_errorCode!=BIRD_ERROR_SUCCESS)
        {
            errorHandler(m_errorCode);
            return status = false;
        }
        // if this sensor is attached, set number of sensors to current index
        // !THIS ASSUMES THAT SENSORS ARE ATTACHED IN ORDER
        // !BEGINNING AT 1 ON THE BOX (AND 0 IN C++)
        if( (m_pSensor + m_i)->m_config.attached )
            ui->numBirdsComboBox->setCurrentIndex( m_i );
    }

    // GET TRANSMITTER CONFIGURATION
    //
    // The call to GetTransmitterConfiguration() performs a similar task to the
    // GetSensorConfiguration() call. It also returns a status in the filled
    // structure which indicates whether a transmitter is attached to this
    // port or not. In a single transmitter system it is only necessary to
    // find where that transmitter is in order to turn it on and use it.
    //
    m_pXmtr = new CXmtr[m_ATC3DG.m_config.numberTransmitters];
    for(m_i = 0; m_i < m_ATC3DG.m_config.numberTransmitters; m_i++)
    {
        m_errorCode = GetTransmitterConfiguration(m_i, &(m_pXmtr + m_i)->m_config);
        if(m_errorCode!=BIRD_ERROR_SUCCESS)
        {
            errorHandler(m_errorCode);
            return status = false;
        }
    }

    // Search for the first attached transmitter and turn it on
    //
    for(m_id=0; m_id < m_ATC3DG.m_config.numberTransmitters; m_id++)
    {
        if((m_pXmtr + m_id)->m_config.attached)
        {
            // Transmitter selection is a system function.
            // Using the SELECT_TRANSMITTER parameter we send the id of the
            // transmitter that we want to run with the SetSystemParameter() call
            m_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &m_id, sizeof(m_id));
            if(m_errorCode!=BIRD_ERROR_SUCCESS)
            {
                errorHandler(m_errorCode);
                return status = false;
            }
            break;
        }
    }

    ui->outputTextEdit->clear();

    ui->outputTextEdit->appendPlainText(QString("%1 out of %2 sensors detected.").arg(ui->numBirdsComboBox->currentIndex()+1).arg(m_ATC3DG.m_config.numberSensors));
    ui->outputTextEdit->appendPlainText(QString("Measurement Rate: %1Hz").arg(m_ATC3DG.m_config.measurementRate));

    return status;
}

bool AscensionEM::disconnectEM()
{
    bool status = true;

    ui->statusLineEdit->setText("Disconnecting...");
    // Turn off the transmitter before exiting
    // We turn off the transmitter by "selecting" a transmitter with an id of "-1"
    //
    m_id = -1;
    m_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &m_id, sizeof(m_id));
    if(m_errorCode!=BIRD_ERROR_SUCCESS)
    {
        errorHandler(m_errorCode);
        return status = false;
    }

    //  Free memory allocations before exiting
    //
    delete[] m_pSensor;
    delete[] m_pXmtr;

    return status;
}

void AscensionEM::errorHandler(int error)
{
    char			buffer[1024];
    char			*pBuffer = &buffer[0];
    int				numberBytes;

    while(error!=BIRD_ERROR_SUCCESS)
    {
        error = GetErrorText(error, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);
        numberBytes = strlen(buffer);
        buffer[numberBytes] = '\n';		// append a newline to buffer
        //printf("%s", buffer);
        ui->outputTextEdit->appendPlainText("ERROR: ");
        ui->outputTextEdit->appendPlainText(buffer);
    }
}

// ------------------------------ //
// GUI INTERACTION IMPLEMENTATION //
// ------------------------------ //

void AscensionEM::on_initButton_clicked()
{
    if( initializeEM() ) // successfully initialized
    {
        ui->initButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->acquireButton->setEnabled(true);
        ui->statusLineEdit->setText("Initialized.");
    }
    else
        ui->statusLineEdit->setText("Initialization failed!");
}



void AscensionEM::on_disconnectButton_clicked()
{
    if( disconnectEM() )
    {
        ui->initButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->acquireButton->setEnabled(false);
        ui->saveButton->setEnabled(false);
        ui->statusLineEdit->setText("Disconnected.");
    }
    else
        ui->statusLineEdit->setText("Disconnection failed! FATAL ERROR!");
}

void AscensionEM::on_acquireButton_clicked()
{
    // number of sensors
    int numSensors = ui->numBirdsComboBox->currentIndex() + 1;
    // precision (for print operations)
    int prec = 4;

    // Collect data from all birds
    // Loop through all sensors and get a data record if the sensor is attached.
    // Print result to screen
    // Note: The default data format is DOUBLE_POSITION_ANGLES. We can use this
    // format without first setting it.
    //
    DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD record, *pRecord = &record;

    // Set sensor output to position + rotation matrix + time stamp
    for(m_sensorID = 0; m_sensorID < numSensors; m_sensorID++)
    {
        DATA_FORMAT_TYPE buf = DOUBLE_POSITION_MATRIX_TIME_STAMP;
        DATA_FORMAT_TYPE *pBuf = &buf;
        m_errorCode = SetSensorParameter(m_sensorID, DATA_FORMAT, pBuf, sizeof(buf));
        if(m_errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(m_errorCode);}
    }

    // Set up time delay for first loop
    // It only makes sense to request a data record from the sensor once per
    // measurement cycle. Therefore we set up a 10ms loop and request a record
    // only after at least 10ms have elapsed.
    //
    QElapsedTimer elTimer;
    elTimer.start();

    // collect as many records as specified in the command line
    for(m_i = 0; m_i < m_records; m_i++)
    {
        // delay 10ms between collecting data
        // wait till time delay expires
        while( !elTimer.hasExpired(7) );
        // set up time delay for next loop
        elTimer.restart();

        // scan the sensors and request a record
        for(m_sensorID = 0; m_sensorID < numSensors; m_sensorID++)
        {
            // sensor attached so get record
            m_errorCode = GetAsynchronousRecord(m_sensorID, pRecord, sizeof(record));
            if(m_errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(m_errorCode);}

            // get the status of the last data record
            // only report the data if everything is okay
            unsigned int status = GetSensorStatus( m_sensorID );

            if( status == VALID_STATUS)
            {
                // send output to console
                ui->outputTextEdit->clear();
                ui->outputTextEdit->appendPlainText(QString("[Sensor %1]").arg(m_sensorID + 1));
                ui->outputTextEdit->appendPlainText("x\ty\tz");
                ui->outputTextEdit->appendPlainText(QString("%1\t%2\t%3")
                                                    .arg(QString::number(record.x,'f',prec))
                                                    .arg(QString::number(record.y,'f',prec))
                                                    .arg(QString::number(record.z,'f',prec)));
                ui->outputTextEdit->appendPlainText("Rot");
                ui->outputTextEdit->appendPlainText(QString("%1\t%2\t%3\n%4\t%5\t%6\n%7\t%8\t%9")
                                                    .arg(QString::number(record.s[0][0],'f',prec))
                                                    .arg(QString::number(record.s[0][1],'f',prec))
                                                    .arg(QString::number(record.s[0][2],'f',prec))
                                                    .arg(QString::number(record.s[1][0],'f',prec))
                                                    .arg(QString::number(record.s[1][1],'f',prec))
                                                    .arg(QString::number(record.s[1][2],'f',prec))
                                                    .arg(QString::number(record.s[2][0],'f',prec))
                                                    .arg(QString::number(record.s[2][1],'f',prec))
                                                    .arg(QString::number(record.s[2][2],'f',prec)));
                ui->outputTextEdit->appendPlainText(QString("Time: %1").arg(QString::number(record.time,'f',prec)));
            }
            else
            {
                ui->outputTextEdit->clear();
                ui->outputTextEdit->appendPlainText("Invalid status.");
            }

            QWidget::repaint();
        }
    }
}
