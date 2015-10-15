#include "icebot_gui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ICEbot_GUI w;
    w.show();

    return a.exec();
}
