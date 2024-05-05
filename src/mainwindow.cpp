#include "mainwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QFileDialog>

MainWindow::MainWindow()
{
    glWidget2d = new GLWidget2D();
    glWidget2d->setMinimumSize(600, 600);
    glWidget2d->hide();

    cageMVC = new SyncCage3D(false);
    cageGreen = new SyncCage3D(true);
    cageMVC->linkCage(cageGreen);
    cageGreen->linkCage(cageMVC);

    glWidget3dMVC = new StaticGLWidget3D(nullptr, cageMVC);
    glWidget3dMVC->setMinimumSize(600, 600);
    glWidget3dMVC->hide();

    glWidget3dGreen = new StaticGLWidget3D(nullptr, cageGreen);
    glWidget3dGreen->setMinimumSize(600, 600);
    glWidget3dGreen->hide();

    QHBoxLayout *hBoxLayout3d = new QHBoxLayout();
    hBoxLayout3d->addWidget(glWidget3dMVC);
    hBoxLayout3d->addWidget(glWidget3dGreen);

    QPushButton *button1 = new QPushButton("Load 2D Image File");
    QObject::connect(button1, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "texture", "Images (*.png *.jpg *.jpeg)");
        if (!filePath.isEmpty()) {
            glWidget2d->setTextureFilePath(filePath);
        }
    });

    QPushButton *button2 = new QPushButton("Load 2D Cage File");
    QObject::connect(button2, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/2d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget2d->setCageFilePath(filePath);
        }
    });

    QPushButton *button3 = new QPushButton("Render 2D");
    QObject::connect(button3, &QPushButton::clicked, [&]() {
        glWidget2d->init();

        if (glWidget3dMVC->isVisible()) {
            glWidget3dMVC->hide();
        }
        glWidget2d->show();
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

        if (glWidget2d->isVisible()) {
            glWidget2d->hide();
        }
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
    container->addWidget(glWidget2d);
    container->addLayout(hBoxLayout3d);

    this->setLayout(container);
}


MainWindow::~MainWindow()
{
    delete glWidget2d;
    delete glWidget3dMVC;
}
