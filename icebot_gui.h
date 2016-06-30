#ifndef ICEBOT_GUI_H
#define ICEBOT_GUI_H

#include <QMainWindow>
#include <QDebug>

#include <QDateTime>

namespace Ui {
class ICEbot_GUI;
}

class ICEbot_GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit ICEbot_GUI(QWidget *parent = 0);
    ~ICEbot_GUI();

private slots:

private:
    Ui::ICEbot_GUI *ui;

    QDateTime m_epoch;
};

#endif // ICEBOT_GUI_H
