#include "scenevizwidget.h"
#include "ui_scenevizwidget.h"

SceneVizWidget::SceneVizWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SceneVizWidget)
{
    ui->setupUi(this);

    m_view = new ExtendedQt3DWindow(); //view = new Qt3DExtras::Qt3DWindow();
    m_view->defaultFramegraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    m_container = QWidget::createWindowContainer(m_view);
    m_container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    m_widget = ui->dockWidgetContents;

    m_hLayout = new QHBoxLayout(m_widget);
    m_vLayout = new QVBoxLayout();
    m_vLayout->setAlignment(Qt::AlignTop);
    m_hLayout->addWidget(m_container, 1);
    m_hLayout->addLayout(m_vLayout);

    ui->dockWidget->setWindowTitle(QStringLiteral("Scene Visualizer"));

    m_input = new Qt3DInput::QInputAspect();

    m_view->registerAspect(m_input);

    // Root entity
    m_rootEntity = new Qt3DCore::QEntity();
    m_rootEntity->setObjectName(QStringLiteral("rootEntity"));

    // Camera
    m_cameraEntity = m_view->camera();

    m_cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    m_cameraEntity->setPosition(QVector3D(0, 20.0f, 0));
    m_cameraEntity->setUpVector(QVector3D(0, 0, 1)); // Z is up
    m_cameraEntity->setViewCenter(QVector3D(0, 0, 0));

    // For camera controls
    m_camController = new Qt3DExtras::QOrbitCameraController(m_rootEntity);
    m_camController->setLinearSpeed( 50.0f );
    m_camController->setLookSpeed( 180.0f );
    m_camController->setCamera(m_cameraEntity);

    // Set root object of the scene
    m_view->setRootEntity(m_rootEntity);

    // Scenemodifier
    m_modifier = new SceneModifier(m_rootEntity);

    // Show window
    //view->show();

    //connect(view, SIGNAL(wheelScrolled(std::shared_ptr<QWheelEvent>)), this, SLOT(acceptScroll(std::shared_ptr<QWheelEvent>)));
    connect(m_view, SIGNAL(wheelScrolled(QSharedPointer<QWheelEvent>)), this, SLOT(acceptScroll(QSharedPointer<QWheelEvent>)));

    m_isShown = true;
}

SceneVizWidget::~SceneVizWidget()
{
//    if(ui->dockWidget->isHidden())
//        ui->dockWidget->setHidden(false); // if window was closed
//    ui->dockWidget->setFloating(false);
//    view->close();
//    view->destroy();

    m_modifier->deleteLater();

//    view->deleteLater();
//    container->deleteLater();
//    widget->deleteLater();
//    hLayout->deleteLater();
//    vLayout->deleteLater();
//    input->deleteLater();
//    rootEntity->deleteLater();
//    cameraEntity->deleteLater();
//    camController->deleteLater();

    delete ui;
}

//void SceneVizWidget::wheelEvent(QWheelEvent *event)
//{
//    QPoint angle = event->angleDelta();
//    qDebug() << "Mouse wheel" << angle;
//    double ang = static_cast<double>(angle.y())/240. + 1.;
//    cameraEntity->setPosition(cameraEntity->position()*ang);

//    event->accept();
//}

void SceneVizWidget::acceptScroll(QSharedPointer<QWheelEvent> event)
{
    double ang = static_cast<double>(event->angleDelta().y())/240.;
    m_cameraEntity->setPosition(m_cameraEntity->position()*(1-ang) + m_cameraEntity->viewCenter()*ang);
    // TODO: add zoom with reference to the pointer location
}

void SceneVizWidget::on_toggleRenderWindowButton_clicked()
{
    if(m_view->isVisible())
    {
        ui->dockWidget->hide();
        //view->hide();
        ui->toggleRenderWindowButton->setText("Show Window");
        m_isShown = false;
    }
    else
    {
        ui->dockWidget->show();
        if(ui->dockWidget->isFloating())
        {
            ui->dockWidget->resize(ui->dockFrame->size());
            ui->dockWidget->move(1,1);
            ui->dockWidget->setFloating(false);
        }
        //view->show();
        ui->toggleRenderWindowButton->setText("Hide Window");

        m_isShown = true;
    }

}

void SceneVizWidget::on_baseCheckBox_toggled(bool checked)
{
    m_modifier->enableObject(checked, EM_SENSOR_BB);
}

void SceneVizWidget::on_pushButton_clicked()
{
    QVector3D pos = m_modifier->getTriadPosition(EM_SENSOR_BB);
    m_cameraEntity->setViewCenter(pos);
}

void SceneVizWidget::on_resetBaseButton_clicked()
{
    m_modifier->resetBase();
}

void SceneVizWidget::on_noTformRadio_clicked()
{
    m_modifier->changeTFormOption(0);
}

void SceneVizWidget::on_tform1Radio_clicked()
{
    m_modifier->changeTFormOption(1);
}

void SceneVizWidget::on_tform2Radio_clicked()
{
    m_modifier->changeTFormOption(2);
}

void SceneVizWidget::on_usAngleSpinBox_valueChanged(int arg1)
{
    m_modifier->setUSangle(arg1);
}
