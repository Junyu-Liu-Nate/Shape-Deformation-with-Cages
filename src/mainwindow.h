#pragma once

#include <QMainWindow>
#include "glwidget.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    GLWidget *glWidget;
};
