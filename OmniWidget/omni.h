#ifndef OMNI_H
#define OMNI_H

#include <QWidget>
#include <QTimer>
//#include <QThread>

#include <stdio.h>
#include <assert.h>
#include <windows.h>
#include <conio.h>

//#include <HDU/hduVector.h>
#include "omnithread.h"

namespace Ui {
class Omni;
}

class Omni : public QWidget
{
    Q_OBJECT
    
public:
    explicit Omni(QWidget *parent = 0);
    omniThread *m_omnithread;
    ~Omni();
    
public slots:
    void updateLCD();

private slots:
    void on_sphSizeSpinBox_valueChanged(double arg1);

private:
    Ui::Omni *ui;

//    QTimer *omniTimer;
    QTimer *updateTimer;
    //QThread momniThread;
};

#endif // OMNI_H
