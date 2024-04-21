#pragma once

#include <QMainWindow>
#include "glwidget3d.h"
#include "glwidget2d.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    GLWidget3D *glWidget3d;
    GLWidget2D *glWidget2d;
};
