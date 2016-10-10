#include "extendedqt3dwindow.h"

ExtendedQt3DWindow::ExtendedQt3DWindow(QScreen *screen) : Qt3DExtras::Qt3DWindow(screen)
{
    qRegisterMetaType< QSharedPointer<QWheelEvent> >("QSharedPointer<QWheelEvent>");
}

void ExtendedQt3DWindow::wheelEvent(QWheelEvent *event)
{
    emit wheelScrolled(QSharedPointer<QWheelEvent>( new QWheelEvent(*event) ));

    event->accept();
}
