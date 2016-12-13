#include "respmodelwidget.h"
#include "ui_respmodelwidget.h"

respModelWidget::respModelWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::respModelWidget)
{
    ui->setupUi(this);
    this->setWindowTitle("Respiration Modeling");
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
    emit initializeRespModel();
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
