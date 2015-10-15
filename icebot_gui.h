#ifndef ICEBOT_GUI_H
#define ICEBOT_GUI_H

#include <QMainWindow>

namespace Ui {
class ICEbot_GUI;
}

class ICEbot_GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit ICEbot_GUI(QWidget *parent = 0);
    ~ICEbot_GUI();

private:
    Ui::ICEbot_GUI *ui;
};

#endif // ICEBOT_GUI_H
