#pragma once

#include <QMainWindow>
#include <QLabel>
#include "staticglwidget2d.h"
#include "staticglwidget3d.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    SyncCage2D *cage2dMVC;
    SyncCage2D *cage2dGreen;
    SyncCage2D *cage2dHigherOrderGreen;
    StaticGLWidget2D *glWidget2dMVC;
    StaticGLWidget2D *glWidget2dGreen;
    StaticGLWidget2D *glWidget2dGreenHigherOrder;
    QLabel *label2dMVC;
    QLabel *label2dGreen;
    QLabel *label2dGreenHigherOrder;

    SyncCage3D *cage3dMVC;
    SyncCage3D *cage3dGreen;
    StaticGLWidget3D *glWidget3dMVC;
    StaticGLWidget3D *glWidget3dGreen;
    QLabel *label3dMVC;
    QLabel *label3dGreen;
};
