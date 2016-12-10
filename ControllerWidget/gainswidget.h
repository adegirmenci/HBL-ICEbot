#ifndef GAINSWIDGET_H
#define GAINSWIDGET_H

#include <QWidget>

#include <QCloseEvent>
#include <QVector>

struct GainsPYRT
{
    double kPitchMin;
    double kPitch;
    double kPitchMax;
    double kYawMin;
    double kYaw;
    double kYawMax;
    double kRollMin;
    double kRoll;
    double kRollMax;
    double kTransMin;
    double kTrans;
    double kTransMax;
};

struct ConvergenceLimits
{
    double posMin;
    double posMax;
    double angleMin;
    double angleMax;
};

Q_DECLARE_METATYPE(GainsPYRT)
Q_DECLARE_METATYPE(ConvergenceLimits)

namespace Ui {
class gainsWidget;
}

class gainsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit gainsWidget(QWidget *parent = 0);
    ~gainsWidget();

signals:
    void closeGainsWindow();
    void setGains(GainsPYRT gains);
    void setLimits(ConvergenceLimits limits);

public slots:
    void on_setGainsButton_clicked();

    void on_setLimitsButton_clicked();

private slots:
    void on_closeButton_clicked();

private:
    Ui::gainsWidget *ui;

    void closeEvent(QCloseEvent *event);
};

#endif // GAINSWIDGET_H
