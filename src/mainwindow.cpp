#include "mainwindow.h"
#include <QHBoxLayout>

MainWindow::MainWindow()
{
    glWidget = new GLWidget(RenderMode::Render2D);

    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget(glWidget);
    this->setLayout(container);
}


MainWindow::~MainWindow()
{
    delete glWidget;
}
