#include "triadentity.h"

TriadEntity::TriadEntity(Qt3DCore::QEntity *parent) : Qt3DCore::QEntity(parent)
{
    // LOAD MESH
    m_arrowMesh = new Qt3DRender::QMesh();
    //arrowMesh = QSharedPointer<Qt3DRender::QMesh>(new Qt3DRender::QMesh());
    m_arrowMesh->setMeshName("Arrow");
    QFileInfo check_file( QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\SceneVizWidget\\Arrow.obj") );
    if( check_file.exists() && check_file.isFile() )
    {
        m_arrowMesh->setSource(QUrl::fromLocalFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\SceneVizWidget\\Arrow.obj")));
        qDebug() << "Loaded arrow mesh.";
    }
    else
    {
        qDebug() << "Arrow mesh file doesn't exist!";
    }

    //
    m_triadTransforms = new Qt3DCore::QTransform();
    m_triadTransforms->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    this->addComponent(m_triadTransforms);

    // ADD MESH TO THREE ARROW ENTITIES
    qDebug() << "Adding meshes to arrowEntityList.";

    m_arrowEntityList.resize(3);
    for(size_t i = 0; i < 3; i++)
    {
        m_arrowEntityList.replace(i,new Qt3DCore::QEntity(this));
        m_arrowEntityList[i]->addComponent(m_arrowMesh);
        //arrowEntityList[i]->addComponent(arrowMesh.data());

        Qt3DCore::QTransform *arrowTransforms;
        Qt3DExtras::QPhongMaterial *arrowMaterial;

        arrowTransforms = new Qt3DCore::QTransform();

        arrowTransforms->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
        switch(i){
        case 0:
            arrowTransforms->setRotation( QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0),0.0f) );
            break;
        case 1:
            arrowTransforms->setRotation( QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1),90.0f) );
            break;
        case 2:
            arrowTransforms->setRotation( QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0),-90.0f) );
            break;
        }

        arrowTransforms->setScale3D(QVector3D(3,2,2));

        arrowMaterial = new Qt3DExtras::QPhongMaterial();
        switch(i){
        case 0:
            arrowMaterial->setDiffuse(QColor(255,0,0));
            break;
        case 1:
            arrowMaterial->setDiffuse(QColor(0,255,0));
            break;
        case 2:
            arrowMaterial->setDiffuse(QColor(0,0,255));
            break;
        }
        arrowMaterial->setSpecular(Qt::white);

        m_arrowEntityList[i]->addComponent(arrowMaterial);
        m_arrowEntityList[i]->addComponent(arrowTransforms);
    }
    qDebug() << "Adding meshes done.";
}

TriadEntity::~TriadEntity()
{
    qDebug() << "Destroying TriadEntity.";

    //arrowEntityList.clear();

    //arrowMesh->deleteLater();
}

void TriadEntity::translate(QVector3D &trans)
{
    m_triadTransforms->setTranslation(trans);
}

void TriadEntity::rotate(QQuaternion &rot)
{
    m_triadTransforms->setRotation(rot);
}

void TriadEntity::setTransformation(Qt3DCore::QTransform &tform)
{
    m_triadTransforms->setMatrix(tform.matrix());
}
