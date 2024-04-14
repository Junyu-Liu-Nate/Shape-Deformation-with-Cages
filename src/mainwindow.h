#pragma once

#include <QMainWindow>
#include "glwidget.h"
#include "render2d.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    GLWidget *glWidget;
    Render2D *render2d;

};
