#include "mainwindow.h"
#include <cstdlib>
#include <ctime>

#include <QApplication>
#include <QSurfaceFormat>
#include <QScreen>

int main(int argc, char *argv[])
{
    srand(static_cast<unsigned>(time(0)));

    // Create a Qt application
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName("ARAP");
    QCoreApplication::setOrganizationName("CS 2240");
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    // Set OpenGL version to 4.1 and context to Core
    QSurfaceFormat fmt;
    fmt.setVersion(4, 1);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    // Create a GUI window
    MainWindow w;
    w.resize(600, 500);
    int desktopArea = QGuiApplication::primaryScreen()->size().width() *
                      QGuiApplication::primaryScreen()->size().height();
    int widgetArea = w.width() * w.height();
    if (((float)widgetArea / (float)desktopArea) < 0.75f)
        w.show();
    else
        w.showMaximized();


    return a.exec();
}
