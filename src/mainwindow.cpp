#include "mainwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QFileDialog>

MainWindow::MainWindow()
{
    // 2D cages
    cage2dMVC = new SyncCage2D(Mode2D::MVC);
    cage2dGreen = new SyncCage2D(Mode2D::Green);
    cage2dHigherOrderGreen = new SyncCage2D(Mode2D::HigherOrderGreen);

    cage2dMVC->linkCage1(cage2dGreen);
    cage2dMVC->linkCage2(cage2dHigherOrderGreen);
    cage2dGreen->linkCage1(cage2dMVC);
    cage2dGreen->linkCage2(cage2dHigherOrderGreen);
    cage2dHigherOrderGreen->linkCage1(cage2dMVC);
    cage2dHigherOrderGreen->linkCage2(cage2dGreen);

    // 2D viewports
    glWidget2dMVC = new StaticGLWidget2D(cage2dMVC);
    glWidget2dMVC->setMinimumSize(600, 600);
    glWidget2dMVC->hide();

    glWidget2dGreen = new StaticGLWidget2D(cage2dGreen);
    glWidget2dGreen->setMinimumSize(600, 600);
    glWidget2dGreen->hide();

    glWidget2dGreenHigherOrder = new StaticGLWidget2D(cage2dHigherOrderGreen);
    glWidget2dGreenHigherOrder->setMinimumSize(600, 600);
    glWidget2dGreenHigherOrder->hide();

    // 2D labels
    label2dMVC = new QLabel("MVC");
    label2dMVC->setAlignment(Qt::AlignCenter);
    label2dMVC->hide();

    label2dGreen = new QLabel("Green Coordinate");
    label2dGreen->setAlignment(Qt::AlignCenter);
    label2dGreen->hide();

    label2dGreenHigherOrder = new QLabel("Higher-order Green Coordinate");
    label2dGreenHigherOrder->setAlignment(Qt::AlignCenter);
    label2dGreenHigherOrder->hide();

    // 2D viewports with labels
    QVBoxLayout *vBoxLayout2dMVC = new QVBoxLayout();
    vBoxLayout2dMVC->addWidget(label2dMVC);
    vBoxLayout2dMVC->addWidget(glWidget2dMVC);

    QVBoxLayout *vBoxLayout2dGreen = new QVBoxLayout();
    vBoxLayout2dGreen->addWidget(label2dGreen);
    vBoxLayout2dGreen->addWidget(glWidget2dGreen);

    QVBoxLayout *vBoxLayout2dGreenHigherOrder = new QVBoxLayout();
    vBoxLayout2dGreenHigherOrder->addWidget(label2dGreenHigherOrder);
    vBoxLayout2dGreenHigherOrder->addWidget(glWidget2dGreenHigherOrder);

    // 2D side-by-side layout
    QHBoxLayout *hBoxLayout2d = new QHBoxLayout();
    hBoxLayout2d->addLayout(vBoxLayout2dMVC);
    hBoxLayout2d->addLayout(vBoxLayout2dGreen);
    hBoxLayout2d->addLayout(vBoxLayout2dGreenHigherOrder);

    // 3D cages
    cage3dMVC = new SyncCage3D(false);
    cage3dGreen = new SyncCage3D(true);
    cage3dMVC->linkCage(cage3dGreen);
    cage3dGreen->linkCage(cage3dMVC);

    // 3D viewports
    glWidget3dMVC = new StaticGLWidget3D(nullptr, cage3dMVC);
    glWidget3dMVC->setMinimumSize(600, 600);
    glWidget3dMVC->hide();

    glWidget3dGreen = new StaticGLWidget3D(nullptr, cage3dGreen);
    glWidget3dGreen->setMinimumSize(600, 600);
    glWidget3dGreen->hide();

    // 3D labels
    label3dMVC = new QLabel("MVC");
    label3dMVC->setAlignment(Qt::AlignCenter);
    label3dMVC->hide();

    label3dGreen = new QLabel("Green Coordinate");
    label3dGreen->setAlignment(Qt::AlignCenter);
    label3dGreen->hide();

    // 3D viewports with labels
    QVBoxLayout *vBoxLayout3dMVC = new QVBoxLayout();
    vBoxLayout3dMVC->addWidget(label3dMVC);
    vBoxLayout3dMVC->addWidget(glWidget3dMVC);

    QVBoxLayout *vBoxLayout3dGreen = new QVBoxLayout();
    vBoxLayout3dGreen->addWidget(label3dGreen);
    vBoxLayout3dGreen->addWidget(glWidget3dGreen);

    // 3D side-by-side
    QHBoxLayout *hBoxLayout3d = new QHBoxLayout();
    hBoxLayout3d->addLayout(vBoxLayout3dMVC);
    hBoxLayout3d->addLayout(vBoxLayout3dGreen);

    // Menu buttons
    QPushButton *button1 = new QPushButton("Load 2D Image File");
    QObject::connect(button1, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "texture", "Images (*.png *.jpg *.jpeg)");
        if (!filePath.isEmpty()) {
            glWidget2dMVC->setTextureFilePath(filePath);
            glWidget2dGreen->setTextureFilePath(filePath);
            glWidget2dGreenHigherOrder->setTextureFilePath(filePath);
        }
    });

    QPushButton *button2 = new QPushButton("Load 2D Cage File");
    QObject::connect(button2, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/2d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget2dMVC->setCageFilePath(filePath);
            glWidget2dGreen->setCageFilePath(filePath);
            glWidget2dGreenHigherOrder->setCageFilePath(filePath);
        }
    });

    QPushButton *button3 = new QPushButton("Render 2D");
    QObject::connect(button3, &QPushButton::clicked, [&]() {
        glWidget2dMVC->init();
        glWidget2dGreen->init();
        glWidget2dGreenHigherOrder->init();

        if (glWidget3dMVC->isVisible()) {
            label3dMVC->hide();
            label3dGreen->hide();
            glWidget3dMVC->hide();
            glWidget3dGreen->hide();
        }
        label2dMVC->show();
        label2dGreen->show();
        label2dGreenHigherOrder->show();
        glWidget2dMVC->show();
        glWidget2dGreen->show();
        glWidget2dGreenHigherOrder->show();
    });

    QPushButton *button4 = new QPushButton("Load 3D Cage File");
    QObject::connect(button4, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/3d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget3dMVC->setCageFilePath(filePath);
            glWidget3dGreen->setCageFilePath(filePath);
        }
    });

    QPushButton *button5 = new QPushButton("Load 3D Object File");
    QObject::connect(button5, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/3d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget3dMVC->setObjectFilePath(filePath);
            glWidget3dGreen->setObjectFilePath(filePath);
        }
    });

    QPushButton *button6 = new QPushButton("Render 3D");
    QObject::connect(button6, &QPushButton::clicked, [&]() {
        glWidget3dMVC->init();
        glWidget3dGreen->init();

        if (glWidget2dMVC->isVisible()) {
            label2dMVC->hide();
            label2dGreen->hide();
            label2dGreenHigherOrder->hide();
            glWidget2dMVC->hide();
            glWidget2dGreen->hide();
            glWidget2dGreenHigherOrder->hide();
        }
        label3dMVC->show();
        label3dGreen->show();
        glWidget3dMVC->show();
        glWidget3dGreen->show();
    });

    QVBoxLayout *menu = new QVBoxLayout;
    menu->addWidget(button1);
    menu->addWidget(button2);
    menu->addWidget(button3);
    menu->addWidget(button4);
    menu->addWidget(button5);
    menu->addWidget(button6);

    QWidget *menuWidget = new QWidget;
    menuWidget->setLayout(menu);
    menuWidget->setMaximumWidth(160);

    QHBoxLayout *container = new QHBoxLayout;

    container->addWidget(menuWidget);
    container->addLayout(hBoxLayout2d);
    container->addLayout(hBoxLayout3d);

    this->setLayout(container);
}

MainWindow::~MainWindow()
{
    delete glWidget2dMVC;
    delete glWidget2dGreen;
    delete glWidget3dMVC;
    delete glWidget3dGreen;
}
