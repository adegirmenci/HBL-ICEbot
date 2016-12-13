#ifndef RESPMODELWIDGET_H
#define RESPMODELWIDGET_H

#include <QWidget>

#include <QCloseEvent>

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

private slots:
    void on_closeButton_clicked();

    void on_initializeButton_clicked();

private:
    Ui::respModelWidget *ui;

    void closeEvent(QCloseEvent *event);
};

#endif // RESPMODELWIDGET_H
