#include <QApplication>

#include "common/ContiTypes.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<ContiStatus>("ContiStatus");
    qRegisterMetaType<ContiTestConfig>("ContiTestConfig");
    qRegisterMetaType<SingleAxisJogConfig>("SingleAxisJogConfig");
    qRegisterMetaType<VelocityControlConfig>("VelocityControlConfig");
    qRegisterMetaType<QVector<VelocityPlotSample>>("QVector<VelocityPlotSample>");

    MainWindow window;
    window.show();
    return app.exec();
}
