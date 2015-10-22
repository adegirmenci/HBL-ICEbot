#include "labjack.h"
#include "ui_labjack.h"

LabJack::LabJack(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LabJack)
{
    ui->setupUi(this);
}

LabJack::~LabJack()
{
    delete ui;
}
