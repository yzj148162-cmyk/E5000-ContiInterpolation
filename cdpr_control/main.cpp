#include <QApplication>
#include <QTextStream>

#include "common/ContiTypes.h"
#include "cdpr/CdprConfiguration.h"
#include "cdpr/CdprVirtualConsistencyAnalyzer.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("CDPR"));
    QCoreApplication::setApplicationName(QStringLiteral("CdprForceInteractionControl"));

    const QStringList arguments = QCoreApplication::arguments();
    const int analysisArgument = arguments.indexOf(QStringLiteral("--analyze-cdpr-run"));
    if (analysisArgument >= 0) {
        QTextStream output(stdout);
        if (analysisArgument + 1 >= arguments.size()) {
            output << "--analyze-cdpr-run requires a run directory.\n";
            return 2;
        }
        const CdprVirtualConsistencyAnalysisResult analysis =
            CdprVirtualConsistencyAnalyzer::analyze(arguments.at(analysisArgument + 1));
        output << analysis.summary << '\n';
        if (!analysis.success) {
            output << analysis.errorText << '\n';
            return 1;
        }
        output << analysis.outputDirectory << '\n';
        return 0;
    }

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
    qRegisterMetaType<CdprVelocityControlConfig>("CdprVelocityControlConfig");
    qRegisterMetaType<CdprVelocityControlStatus>("CdprVelocityControlStatus");
    qRegisterMetaType<CdprForceControlRequest>("CdprForceControlRequest");
    qRegisterMetaType<CdprForceControlStatus>("CdprForceControlStatus");

    MainWindow window;
    window.show();
    return app.exec();
}
