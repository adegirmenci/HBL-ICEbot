#ifndef FRMGRAB_H
#define FRMGRAB_H

#include <QWidget>

namespace Ui {
class FrmGrab;
}

class FrmGrab : public QWidget
{
    Q_OBJECT

public:
    explicit FrmGrab(QWidget *parent = 0);
    ~FrmGrab();

private:
    Ui::FrmGrab *ui;
};

#endif // FRMGRAB_H
