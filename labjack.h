#ifndef LABJACK_H
#define LABJACK_H

#include <QWidget>
#include "LabJackUD.h"

namespace Ui {
class LabJack;
}

class LabJack : public QWidget
{
    Q_OBJECT

public:
    explicit LabJack(QWidget *parent = 0);
    ~LabJack();

private:
    Ui::LabJack *ui;
};

#endif // LABJACK_H
