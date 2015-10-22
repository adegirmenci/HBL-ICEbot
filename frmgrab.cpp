#include "frmgrab.h"
#include "ui_frmgrab.h"

FrmGrab::FrmGrab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FrmGrab)
{
    ui->setupUi(this);
}

FrmGrab::~FrmGrab()
{
    delete ui;
}
