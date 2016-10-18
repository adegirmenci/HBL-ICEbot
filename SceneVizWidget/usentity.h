#ifndef USENTITY_H
#define USENTITY_H

#include <QtCore/QObject>
#include <Qt3DCore/qentity.h>

#include <QFileInfo>
#include <QUrl>

#include <Qt3DCore/qtransform.h>
#include <QVector3D>
#include <QQuaternion>

#include <QtCore/QVector>
#include <QtCore/QDebug>
#include <QColor>

#include <Qt3DRender/qmesh.h>
#include <Qt3DRender/qtextureimage.h>
#include <Qt3DExtras/qdiffusemapmaterial.h>
#include <Qt3DRender/qcullface.h>
#include <Qt3DRender/qdepthtest.h>
#include <QSharedPointer>

#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qrenderstate.h>

#include <QDir>

class usEntity : public Qt3DCore::QEntity
{
    Q_OBJECT

public:
    explicit usEntity(QEntity *parent = nullptr);
    ~usEntity();
    QVector3D getTranslation() { return m_usTransforms->translation(); }
    QQuaternion getRotation() { return m_usTransforms->rotation(); }

    // signals:

    public slots:
        // setDisplayed // implemented by QEntity

        void translate(QVector3D &trans);
        void rotate(QQuaternion &rot);
        void setTransformation(Qt3DCore::QTransform &tform);

    private:
        Qt3DRender::QMesh *m_usMesh;
        QVector<Qt3DCore::QEntity *> m_usEntityList;
        Qt3DRender::QTextureImage *m_usImage;
        Qt3DExtras::QDiffuseMapMaterial *m_usMaterial;

        Qt3DCore::QTransform *m_usTransforms;

        bool isShown;
};

#endif // USENTITY_H
