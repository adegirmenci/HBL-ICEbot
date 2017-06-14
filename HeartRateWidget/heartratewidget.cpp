#include "heartratewidget.h"
#include "ui_heartratewidget.h"

HeartRateWidget::HeartRateWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HeartRateWidget)
{
    ui->setupUi(this);

    m_time.resize(VEC_SIZE, 0.0);
    m_voltage.resize(VEC_SIZE, 0.0);

    m_HR = 0.0, m_stdHR = 0.0, m_phaseHR = 0.0;

    minPeakDist = 60./120./2.;
    minPeakHei = 0.2;

    m_counter = 0;

    // Initialize MATLAB DLL
    ECGgating_initialize();

    // qDebug() << "HR Widget Thread ID: " << reinterpret_cast<int>(QThread::currentThreadId()) << ".";
}

HeartRateWidget::~HeartRateWidget()
{
    // Terminate MATLAB DLL
    ECGgating_terminate();

    delete ui;
}

void HeartRateWidget::receiveECG(qint64 timeStamp, std::vector<double> data)
{
    // remove first element
    m_time.erase(m_time.begin());
    m_voltage.erase(m_voltage.begin());

    // add new reading
    m_time.push_back((double)timeStamp/1000.);
    m_voltage.push_back(data[0]);

    // peak detection
    ECGgating(&m_voltage[0], &m_time[0], minPeakDist, minPeakHei,
              ECGpeakVals_data, ECGpeakVals_size,
              ECGpeakTimes_data, ECGpeakTimes_size, &m_HR, &m_stdHR, &m_phaseHR);

    m_counter = (m_counter + 1) % 60;
    if(m_counter == 0)
    {
        ui->HRlineEdit->setText(QString::number(m_HR,'f',1));
        ui->phaseLineEdit->setText(QString::number(m_phaseHR,'f',3));
    }

    emit reportPhase(m_phaseHR);
}
