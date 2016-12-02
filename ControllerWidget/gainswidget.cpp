#include "gainswidget.h"
#include "ui_gainswidget.h"

gainsWidget::gainsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::gainsWidget)
{
    ui->setupUi(this);

    qRegisterMetaType<GainsPYRT>("GainsPYRT");
}

gainsWidget::~gainsWidget()
{
    delete ui;
}

void gainsWidget::on_closeButton_clicked()
{
    emit closeGainsWindow();
}

void gainsWidget::closeEvent(QCloseEvent *event)
{
    emit closeGainsWindow();
    event->accept();
}

void gainsWidget::on_setGainsButton_clicked()
{
    GainsPYRT gains;

    gains.kPitchMin = ui->gainPitchMinSpinbox->value();
    gains.kPitchMax = ui->gainPitchMaxSpinbox->value();
    gains.kYawMin = ui->gainYawMinSpinbox->value();
    gains.kYawMax = ui->gainYawMaxSpinbox->value();
    gains.kRollMin = ui->gainRollMinSpinbox->value();
    gains.kRollMax = ui->gainRollMaxSpinbox->value();
    gains.kTransMin = ui->gainTransMinSpinbox->value();
    gains.kTransMax = ui->gainTransMaxSpinbox->value();

    emit setGains(gains);
}
