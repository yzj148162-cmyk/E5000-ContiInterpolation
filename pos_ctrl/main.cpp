#include <QApplication>

#include "mainwindow.h"
#include "ui/UiRefreshController.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<UiSnapshot>("UiSnapshot");

    MainWindow w;
    w.show();
    return app.exec();
}
