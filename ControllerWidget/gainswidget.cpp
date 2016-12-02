#include "gainswidget.h"
#include "ui_gainswidget.h"

gainsWidget::gainsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::gainsWidget)
{
    ui->setupUi(this);
}

gainsWidget::~gainsWidget()
{
    delete ui;
}
