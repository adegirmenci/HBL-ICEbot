#ifndef RESPMODELWIDGET_H
#define RESPMODELWIDGET_H

#include <QWidget>

#include <QCloseEvent>

#include "filtfilt.h"
#include "../icebot_definitions.h"

namespace Ui {
class respModelWidget;
}

class respModelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit respModelWidget(QWidget *parent = 0);
    ~respModelWidget();

signals:
    void closeRespModelWindow();

    void initializeRespModel();

    void re_initializeRespModel();

    void stopRespModel();

    void newFutureSamplesValue(int n);

private slots:
    void on_closeButton_clicked();

    void on_initializeButton_clicked();

    void on_futureSamplesSpinBox_valueChanged(int arg1);

    void on_reInitButton_clicked();

    void receiveDataFromRespModel(int numSamples,
                                   bool isTrained,
                                   bool inVivoMode,
                                   double omega0);//,
                                   //EigenVectorFiltered Bird4_filtered,
                                   //EigenVectorFiltered Bird4_filtered_new,
                                   //EigenVectorFiltered breathSignalFromModel);

    void plotBird4(unsigned int plotID, double time, double value);

    void on_stopButton_clicked();

private:
    Ui::respModelWidget *ui;

    void closeEvent(QCloseEvent *event);

    double m_lastPlotKey; // to limit update rate of plot
};

#endif // RESPMODELWIDGET_H
