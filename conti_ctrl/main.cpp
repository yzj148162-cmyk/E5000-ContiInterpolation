#include <QApplication>

#include "common/ContiTypes.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("CDPR"));
    QCoreApplication::setApplicationName(QStringLiteral("E5000ContiInterpolation"));
    qRegisterMetaType<ContiStatus>("ContiStatus");
    qRegisterMetaType<ContiTestConfig>("ContiTestConfig");
    qRegisterMetaType<SingleAxisJogConfig>("SingleAxisJogConfig");
    qRegisterMetaType<VelocityControlConfig>("VelocityControlConfig");
    qRegisterMetaType<QVector<VelocityPlotSample>>("QVector<VelocityPlotSample>");
    qRegisterMetaType<TorqueTestConfig>("TorqueTestConfig");
    qRegisterMetaType<QVector<TorquePlotSample>>("QVector<TorquePlotSample>");
    qRegisterMetaType<TraceDelayCalibrationConfig>("TraceDelayCalibrationConfig");
    qRegisterMetaType<QVector<TraceDelayPlotSample>>("QVector<TraceDelayPlotSample>");

    MainWindow window;
    window.show();
    return app.exec();
}
