#ifndef TRIADENTITY_H
#define TRIADENTITY_H

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
#include <QSharedPointer>

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QGoochMaterial>

//typedef QVector<QComponent*> QComponentVector;

class TriadEntity : public Qt3DCore::QEntity
{
    Q_OBJECT

public:
    explicit TriadEntity(QEntity *parent = nullptr);
    ~TriadEntity();
    QVector3D getTranslation() { return m_triadTransforms->translation(); }
    QQuaternion getRotation() { return m_triadTransforms->rotation(); }

// signals:

public slots:
    // setDisplayed // implemented by QEntity

    void translate(const QVector3D &trans);
    void rotate(const QQuaternion &rot);
    void setTransformation(const Qt3DCore::QTransform &tform);

private:
    Qt3DRender::QMesh *m_arrowMesh;
    //QSharedPointer<Qt3DRender::QMesh> arrowMesh;
    QVector<Qt3DCore::QEntity *> m_arrowEntityList;

    Qt3DCore::QTransform *m_triadTransforms;

    bool isShown;
};

#endif // TRIADENTITY_H
