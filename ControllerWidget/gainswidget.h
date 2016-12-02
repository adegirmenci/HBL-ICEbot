#ifndef GAINSWIDGET_H
#define GAINSWIDGET_H

#include <QWidget>

namespace Ui {
class gainsWidget;
}

class gainsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit gainsWidget(QWidget *parent = 0);
    ~gainsWidget();

private:
    Ui::gainsWidget *ui;
};

#endif // GAINSWIDGET_H
