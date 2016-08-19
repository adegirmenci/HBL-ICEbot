#ifndef EXTENDEDQT3DWINDOW_H
#define EXTENDEDQT3DWINDOW_H

#include <QObject>
#include <QDebug>

#include <QScreen>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DInput/QWheelEvent>
#include <QSharedPointer>

Q_DECLARE_METATYPE(QSharedPointer<QWheelEvent>)

class ExtendedQt3DWindow : public Qt3DExtras::Qt3DWindow
{
    Q_OBJECT

public:
    ExtendedQt3DWindow(QScreen *screen = nullptr);

signals:
    void wheelScrolled(QSharedPointer<QWheelEvent> event);

protected:
    void wheelEvent(QWheelEvent *event);
};

#endif // EXTENDEDQT3DWINDOW_H
