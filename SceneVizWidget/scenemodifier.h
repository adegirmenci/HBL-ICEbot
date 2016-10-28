#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include <QtCore/QObject>
#include <iostream>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <QMatrix4x4>

#include <QtCore/QVector>
#include <QtCore/QHash>
#include <QtCore/QDebug>
#include <QSharedPointer>
#include <QVector3D>

#include <QTime>

#include "../icebot_definitions.h"
#include "../AscensionWidget/3DGAPI/ATC3DG.h"

#include "triadentity.h"
#include "usentity.h"

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
    void resetBase();
    void changeTFormOption(int opt);
    void setUSangle(int ang);

private:
    QVector<Qt3DCore::QEntity *> m_entityList;
    QHash<QString, int> m_entityHash; // look up entities by name

    Qt3DCore::QEntity *m_rootEntity;

    QMatrix4x4 m_calibMat;
    Qt3DCore::QTransform m_Box_US; // Emitter to US crystal transforms
    QMatrix4x4 m_T_Box_EM;
    QMatrix4x4 m_T_EM_CT;
    Qt3DCore::QTransform m_CT_US;
    QMatrix4x4 m_baseEMpose; // for relative stuff
    int m_tformOption;
    float m_usAngle;
};


// TRIADOBJECT


#endif // SCENEMODIFIER_H
