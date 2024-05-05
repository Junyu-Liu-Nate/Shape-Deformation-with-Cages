#pragma once

#include <QMainWindow>
#include "glwidget2d.h"
#include "staticglwidget3d.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    SyncCage3D *cageMVC;
    SyncCage3D *cageGreen;
    StaticGLWidget3D *glWidget3dMVC;
    StaticGLWidget3D *glWidget3dGreen;
    GLWidget2D *glWidget2d;
};
