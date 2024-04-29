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

    glWidget3d = new GLWidget3D();
    glWidget3d->setMinimumSize(600, 600);
    glWidget3d->hide();

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

        if (glWidget3d->isVisible()) {
            glWidget3d->hide();
        }
        glWidget2d->show();
    });

    QPushButton *button4 = new QPushButton("Load 3D Cage File");
    QObject::connect(button4, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/3d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget3d->setCageFilePath(filePath);
        }
    });

    QPushButton *button5 = new QPushButton("Load 3D Object File");
    QObject::connect(button5, &QPushButton::clicked, [&]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "meshes/3d", "Meshes (*.obj)");
        if (!filePath.isEmpty()) {
            glWidget3d->setObjectFilePath(filePath);
        }
    });

    QPushButton *button6 = new QPushButton("Render 3D");
    QObject::connect(button6, &QPushButton::clicked, [&]() {
        glWidget3d->init();

        if (glWidget2d->isVisible()) {
            glWidget2d->hide();
        }
        glWidget3d->show();
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
    container->addWidget(glWidget3d);

    this->setLayout(container);
}


MainWindow::~MainWindow()
{
    delete glWidget2d;
    delete glWidget3d;
}
