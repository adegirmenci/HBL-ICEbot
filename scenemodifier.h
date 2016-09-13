#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include <QtCore/QObject>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <QMatrix4x4>

#include <QtCore/QVector>
#include <QtCore/QHash>
#include <QtCore/QDebug>
#include <QSharedPointer>
#include <QVector3D>

#include <QTime>

#include "../AscensionWidget/icebot_definitions.h"
#include "../AscensionWidget/3DGAPI/ATC3DG.h"

#include "triadentity.h"

class SceneModifier : public QObject
{
    Q_OBJECT
public:
    explicit SceneModifier(Qt3DCore::QEntity *rootEntity);
    ~SceneModifier();

    QVector3D getTriadPosition(EM_SENSOR_IDS sensorID);

signals:

public slots:
    void enableObject(bool enabled, int objID);
    void receiveEMreading(QTime timeStamp,
                          int sensorID,
                          DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD data);

private:
    QVector<Qt3DCore::QEntity *> m_entityList;
    QHash<QString, int> m_entityHash; // look up entities by name

    Qt3DCore::QEntity *m_rootEntity;

    QMatrix4x4 m_calibMat;
    Qt3DCore::QTransform m_Box_CT; // Emitter to US crystal transforms
};


// TRIADOBJECT


#endif // SCENEMODIFIER_H
