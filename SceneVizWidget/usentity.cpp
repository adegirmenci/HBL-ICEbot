#include "usentity.h"

usEntity::usEntity(Qt3DCore::QEntity *parent) : Qt3DCore::QEntity(parent)
                                              , m_usTransforms(new Qt3DCore::QTransform())
                                              , m_usMaterial(new Qt3DExtras::QDiffuseMapMaterial())
                                              , m_usImage(new Qt3DRender::QTextureImage())
                                              , m_usMesh(new Qt3DRender::QMesh())
{
    // LOAD MESH
    m_usMesh->setMeshName("USplane");
    QFileInfo check_file( QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\SceneVizWidget\\ultrasound2.obj") );
    if( check_file.exists() && check_file.isFile() )
    {
        m_usMesh->setSource(QUrl::fromLocalFile(check_file.absoluteFilePath()));
        qDebug() << "Loaded ultrasound mesh.";
    }
    else
    {
        qDebug() << "Ultrasound mesh file doesn't exist!";
    }
    // LOAD MESH DONE

    // SET GLOBAL TRANSFORM
    m_usTransforms->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    m_usTransforms->setRotation( QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0),0.0f) );

    this->addComponent(m_usTransforms);
    // SET GLOBAL TRANSFORM DONE

    // SET ULTRASOUND IMAGE AS TEXTURE
    check_file.setFile(QStringLiteral("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\SceneVizWidget\\1.jpg"));
    if( check_file.exists() && check_file.isFile() )
    {
        m_usImage->setSource(QUrl::fromLocalFile(check_file.absoluteFilePath()));
        qDebug() << "Loaded ultrasound image.";
    }
    else
    {
        qDebug() << "Ultrasound image file doesn't exist!";
    }

    m_usMaterial->diffuse()->addTextureImage(m_usImage);
    m_usMaterial->setShininess(10.0f);
    m_usMaterial->setSpecular(QColor::fromRgbF(0.75f, 0.75f, 0.75f, 1.0f));
    //m_usMaterial->diffuse()->setSize(720,480);
    // SET ULTRASOUND IMAGE AS TEXTURE DONE

    // m_usMaterial->setSpecular(Qt::gray);
    // m_usMaterial->setAmbient(Qt::gray);
    // m_usMaterial->setShininess(10.0f);
    // m_usMaterial->setTextureScale(0.01f);

    // Begin: No culling
    // Access the render states of the material
    Qt3DRender::QEffect *effect_ = m_usMaterial->effect();
    QVector<Qt3DRender::QTechnique *> tqs = effect_->techniques();
    if(tqs.size() > 0)
    {
        QVector<Qt3DRender::QRenderPass *> rps = tqs[0]->renderPasses();
        if(rps.size() > 0)
        {
            QVector<Qt3DRender::QRenderState *> rss = rps[0]->renderStates();
            if(rss.size() > 0)
            {
                qDebug() << "A render state already exists:" << rss[0]->objectName();
            }
            else
            {
                Qt3DRender::QCullFace *cullFront = new Qt3DRender::QCullFace();
                cullFront->setMode(Qt3DRender::QCullFace::NoCulling);
                rps[0]->addRenderState(cullFront);
                Qt3DRender::QDepthTest *depthTest = new Qt3DRender::QDepthTest();
                depthTest->setDepthFunction(Qt3DRender::QDepthTest::LessOrEqual);
                rps[0]->addRenderState(depthTest);
            }
        }
        else
            qDebug() << "No renderPasses.";
    }
    else
        qDebug() << "No techniques.";
    // End: No culling


    // ADD MESH TO ULTRASOUND ENTITY
    qDebug() << "Adding meshes to usEntityList.";

    m_usEntityList.fill(new Qt3DCore::QEntity(this), 1);
    for(int i = 0; i < m_usEntityList.size(); i++)
    {
        //m_usEntityList.replace(i, new Qt3DCore::QEntity(this));

        Qt3DCore::QTransform *usTransforms = new Qt3DCore::QTransform();
        usTransforms->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
        usTransforms->setRotation( QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0),90.0f) );
        usTransforms->setScale3D(QVector3D(10,10,10));

        m_usEntityList[i]->addComponent(m_usMesh);
        m_usEntityList[i]->addComponent(m_usMaterial);
        m_usEntityList[i]->addComponent(usTransforms);
    }

    qDebug() << "Adding meshes done.";
}

usEntity::~usEntity()
{
    qDebug() << "Destroying usEntity.";
    qDebug() << m_usImage->status();

//    m_usMesh->deleteLater();
//    m_usImage->deleteLater();
//    m_usMaterial->deleteLater();
//    m_usTransforms->deleteLater();
}

void usEntity::translate(QVector3D &trans)
{
    m_usTransforms->setTranslation(trans);
}

void usEntity::rotate(QQuaternion &rot)
{
    m_usTransforms->setRotation(rot);
}

void usEntity::setTransformation(Qt3DCore::QTransform &tform)
{
    m_usTransforms->setMatrix(tform.matrix());
}
