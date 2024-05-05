#pragma once

#include <QMainWindow>
#include "staticglwidget2d.h"
#include "staticglwidget3d.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    SyncCage3D *cage3dMVC;
    SyncCage3D *cage3dGreen;
    StaticGLWidget3D *glWidget3dMVC;
    StaticGLWidget3D *glWidget3dGreen;

    SyncCage2D *cage2dMVC;
    SyncCage2D *cage2dGreen;
    StaticGLWidget2D *glWidget2dMVC;
    StaticGLWidget2D *glWidget2dGreen;
};
