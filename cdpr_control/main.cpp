#include <QApplication>

#include "common/ContiTypes.h"
#include "cdpr/CdprConfiguration.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("CDPR"));
    QCoreApplication::setApplicationName(QStringLiteral("CdprForceInteractionControl"));
    qRegisterMetaType<ContiStatus>("ContiStatus");
    qRegisterMetaType<ContiTestConfig>("ContiTestConfig");
    qRegisterMetaType<SingleAxisJogConfig>("SingleAxisJogConfig");
    qRegisterMetaType<VelocityControlConfig>("VelocityControlConfig");
    qRegisterMetaType<QVector<VelocityPlotSample>>("QVector<VelocityPlotSample>");
    qRegisterMetaType<TorqueTestConfig>("TorqueTestConfig");
    qRegisterMetaType<QVector<TorquePlotSample>>("QVector<TorquePlotSample>");
    qRegisterMetaType<TraceDelayCalibrationConfig>("TraceDelayCalibrationConfig");
    qRegisterMetaType<CdprUiStatus>("CdprUiStatus");
    qRegisterMetaType<CdprOfflinePvtRequest>("CdprOfflinePvtRequest");
    qRegisterMetaType<CdprOfflinePvtPlan>("CdprOfflinePvtPlan");
    qRegisterMetaType<CdprOfflinePvtStatus>("CdprOfflinePvtStatus");

    MainWindow window;
    window.show();
    return app.exec();
}
