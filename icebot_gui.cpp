#include "icebot_gui.h"
#include "ui_icebot_gui.h"

ICEbot_GUI::ICEbot_GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ICEbot_GUI)
{
    ui->setupUi(this);

    qDebug() << "Ready";

    Point a,b,c;
    a = Point();
    a.update(1.0,2.0,3.0);
    b = a*-2.0;
    c = a*5.0 + b;
    qDebug() << c.getx() << c.gety() << c.getz();

}

ICEbot_GUI::~ICEbot_GUI()
{
    delete ui;
}
