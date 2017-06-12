#include "respmodelwidget.h"
#include "ui_respmodelwidget.h"

respModelWidget::respModelWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::respModelWidget)
{
    ui->setupUi(this);
    this->setWindowTitle("Respiration Modeling");

    // QCustomPlot
    // include this section to fully disable antialiasing for higher performance:
    ui->plotWidget->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    ui->plotWidget->xAxis->setTickLabelFont(font);
    ui->plotWidget->yAxis->setTickLabelFont(font);
    ui->plotWidget->legend->setFont(font);

    ui->plotWidget->addGraph(); // blue line
    ui->plotWidget->graph(0)->setPen(QPen(Qt::blue));
    //ui->plotWidget->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
    ui->plotWidget->graph(0)->setAntialiasedFill(false);

    ui->plotWidget->addGraph(); // red line
    ui->plotWidget->graph(1)->setPen(QPen(Qt::red));
    ui->plotWidget->graph(1)->setAntialiasedFill(false);

    ui->plotWidget->addGraph(); // black line
    ui->plotWidget->graph(2)->setPen(QPen(Qt::black));
    ui->plotWidget->graph(2)->setAntialiasedFill(false);

    ui->plotWidget->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->plotWidget->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->plotWidget->xAxis->setAutoTickStep(false);
    ui->plotWidget->xAxis->setTickStep(2);
    ui->plotWidget->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->plotWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plotWidget->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plotWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plotWidget->yAxis2, SLOT(setRange(QCPRange)));

    m_lastPlotKey = 0.0;
}

respModelWidget::~respModelWidget()
{
    delete ui;
}

void respModelWidget::on_closeButton_clicked()
{
    emit closeRespModelWindow();
}

void respModelWidget::on_initializeButton_clicked()
{
    ui->initializeButton->setEnabled(false);
    ui->reInitButton->setEnabled(false);
    ui->stopButton->setEnabled(true);

    emit initializeRespModel();
}

void respModelWidget::on_reInitButton_clicked()
{
    ui->initializeButton->setEnabled(false);
    ui->reInitButton->setEnabled(false);
    ui->stopButton->setEnabled(true);

    emit re_initializeRespModel();
}

void respModelWidget::on_stopButton_clicked()
{
    ui->initializeButton->setEnabled(false);
    ui->reInitButton->setEnabled(true);
    ui->stopButton->setEnabled(false);

    emit stopRespModel();
}

void respModelWidget::receiveDataFromRespModel(int numSamples,
                                               bool isTrained,
                                               bool inVivoMode,
                                               double omega0)//,
                                               //EigenVectorFiltered Bird4_filtered,
                                               //EigenVectorFiltered Bird4_filtered_new,
                                               //EigenVectorFiltered breathSignalFromModel)
{
    // update things
    ui->samplesCollectedSpinBox->setValue(numSamples);
    ui->periodSpinBox->setValue(2.0*pi/omega0);

    if(inVivoMode)
        ui->inVivoModeLabel->setStyleSheet("QLabel { background-color : green;}");
    else
        ui->inVivoModeLabel->setStyleSheet("QLabel { background-color : red;}");

    if(isTrained)
        ui->modelIsTrainedLabel->setStyleSheet("QLabel { background-color : green;}");
    else
        ui->modelIsTrainedLabel->setStyleSheet("QLabel { background-color : red;}");

//    // plotting
//    double key = numSamples/150.0;
//    static double lastPointKey = 0;
//    if( (key - lastPointKey) > 0.01) // at most add point every 10 ms
//    {
//        if( (numSamples > 2*N_SAMPLES) && isTrained )
//        {
//            ui->plotWidget->graph(1)->addData(key, Bird4_filtered_new(N_SAMPLES-1));
//            ui->plotWidget->graph(2)->addData(key, breathSignalFromModel(N_SAMPLES-1));
//        }
//        else
//        {
//            if( (numSamples > N_SAMPLES) && isTrained )
//            {
//                ui->plotWidget->graph(1)->addData(key, Bird4_filtered_new(numSamples-N_SAMPLES-1));
//                ui->plotWidget->graph(2)->addData(key, breathSignalFromModel(numSamples-N_SAMPLES-1));
//            }
//            else
//            {
//                ui->plotWidget->graph(0)->addData(key, Bird4_filtered(numSamples-1));
//            }
//        }

//        // remove data of lines that's outside visible range:
//        ui->plotWidget->graph(1)->removeDataBefore(key-8);
//        ui->plotWidget->graph(2)->removeDataBefore(key-8);
//        // rescale value (vertical) axis to fit the current data:
//        ui->plotWidget->graph(1)->rescaleValueAxis();
//        lastPointKey = key;
//    }
//    // make key axis range scroll with the data (at a constant range size of 8):
//    //ui->plotWidget->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
    //    ui->plotWidget->replot();
}

void respModelWidget::plotBird4(unsigned int plotID, double time, double value)
{
    // printf(">>> plotBird4 Received\n");
    ui->plotWidget->graph(plotID)->addData(time, value);

    if(plotID == 0)
    {
        if( (time - m_lastPlotKey) > 0.030) // plot every 30ms
        {
            // make key axis range scroll with the data (at a constant range size of 8):
            ui->plotWidget->xAxis->setRange(time, 15.0, Qt::AlignRight);
            // remove data of lines that's outside visible range:
            ui->plotWidget->graph(0)->removeDataBefore(time-15.0);
            ui->plotWidget->graph(1)->removeDataBefore(time-15.0);
            ui->plotWidget->graph(2)->removeDataBefore(time-15.0);
            //ui->plotWidget->graph(0)->rescaleValueAxis();
            ui->plotWidget->yAxis->rescale();

            ui->plotWidget->replot();
            m_lastPlotKey = time;
        }
    }
}

void respModelWidget::closeEvent(QCloseEvent *event)
{
    emit closeRespModelWindow();
    event->accept();
}

void respModelWidget::on_futureSamplesSpinBox_valueChanged(int arg1)
{
    emit newFutureSamplesValue(arg1);
}
