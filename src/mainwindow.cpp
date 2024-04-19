#include "mainwindow.h"
#include <QHBoxLayout>

MainWindow::MainWindow()
{
    glWidget3d = new GLWidget3D();
    glWidget2d = new GLWidget2D();

    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget(glWidget3d);
    // container->addWidget(glWidget2d);
    this->setLayout(container);
}


MainWindow::~MainWindow()
{
    delete glWidget2d;
    delete glWidget3d;
}
