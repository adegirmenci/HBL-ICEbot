#ifndef SCENEVIZWIDGET_H
#define SCENEVIZWIDGET_H

/*!
 * Written by Alperen Degirmenci
 * Harvard Biorobotics Laboratory
 */
#include <QWidget>

#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtGui/QScreen>

#include <Qt3DInput/QInputAspect>
#include <Qt3DInput/QWheelEvent>

#include <Qt3DExtras/qtorusmesh.h>
#include <Qt3DRender/qmesh.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qsceneloader.h>

#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qorbitcameracontroller.h>

#include "scenemodifier.h"
#include "extendedqt3dwindow.h"

namespace Ui {
class SceneVizWidget;
}

class SceneVizWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SceneVizWidget(QWidget *parent = 0);
    ~SceneVizWidget();

    // Scenemodifier
    SceneModifier *m_modifier;

private slots:
    void on_toggleRenderWindowButton_clicked();
    void acceptScroll(QSharedPointer<QWheelEvent> event);


    void on_baseCheckBox_toggled(bool checked);

    void on_pushButton_clicked();

    void on_resetBaseButton_clicked();

    void on_noTformRadio_clicked();

    void on_tform1Radio_clicked();

    void on_tform2Radio_clicked();

    void on_usAngleSpinBox_valueChanged(int arg1);

private:
    Ui::SceneVizWidget *ui;

    ExtendedQt3DWindow *m_view; //Qt3DExtras::Qt3DWindow *view;
    QWidget *m_container;
    QSize m_screenSize;
    QWidget *m_widget;
    QHBoxLayout *m_hLayout;
    QVBoxLayout *m_vLayout;
    Qt3DInput::QInputAspect *m_input;
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DRender::QCamera *m_cameraEntity;
    Qt3DExtras::QOrbitCameraController *m_camController;

    bool m_isShown;

protected:
//    void wheelEvent(QWheelEvent *event);
};

#endif // SCENEVIZWIDGET_H
