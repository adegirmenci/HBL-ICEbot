#ifndef HEARTRATEWIDGET_H
#define HEARTRATEWIDGET_H

#include <QWidget>
#include <QTime>
#include <QString>
#include <QDebug>
#include <QThread>

#include <cmath>
#include <math.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ECGgating_types.h"
#include "rt_nonfinite.h"
#include "ECGgating.h"
#include "ECGgating_terminate.h"
#include "ECGgating_initialize.h"

#define VEC_SIZE 3000 // 3000 sample points

namespace Ui {
class HeartRateWidget;
}

class HeartRateWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HeartRateWidget(QWidget *parent = 0);
    ~HeartRateWidget();

signals:
    void reportPhase(double phase);

public slots:
    void receiveECG(qint64 timeStamp,
                    std::vector<double> data);

private:
    Ui::HeartRateWidget *ui;

    std::vector<double> m_time;
    std::vector<double> m_voltage;

    double m_HR, m_stdHR, m_phaseHR;

    double ECGpeakVals_data[6000];
    int ECGpeakVals_size[1];
    double ECGpeakTimes_data[6000];
    int ECGpeakTimes_size[2];

    unsigned __int8 m_counter;

    double minPeakDist;
    double minPeakHei;
};

#endif // HEARTRATEWIDGET_H
