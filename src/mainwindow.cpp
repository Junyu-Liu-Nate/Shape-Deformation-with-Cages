#include "mainwindow.h"
#include <QHBoxLayout>

MainWindow::MainWindow()
{
    glWidget = new GLWidget();
    render2d = new Render2D();

    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget(glWidget);
    // container->addWidget(render2d);
    this->setLayout(container);
}


MainWindow::~MainWindow()
{
    delete glWidget;
}
