#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "common/SharedContext.h"
#include "config/SystemConfig.h"
#include "hardware/HardwareThread.h"
#include "planner/PlannerThread.h"
#include "ui/UiRefreshController.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class ErrorPlotWidget;

// 主窗口只负责 UI 线程内的工作：
// 参数输入、按钮操作、粗粒度状态显示，以及线程生命周期管理。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    void setupRuntimeUi();
    void connectSignals();
    void startThreads();
    void stopThreads();
    void appendLog(const QString &text);
    void applyUiRefreshPeriod();
    MotionConfig collectMotionConfig() const;
    void updateSnapshot(const UiSnapshot &snapshot);

private slots:
    void onInitCard();
    void onCloseCard();
    void onEnableAxis();
    void onDisableAxis();
    void onStartMotion();
    void onStopMotion();
    void onReadPosition();
    void onHardwareLog(const QString &message);
    void onPlannerLog(const QString &message);
    void onBoardStateChanged(bool initialized);

private:
    Ui::MainWindow *ui_ = nullptr;
    ErrorPlotWidget *errorPlot_ = nullptr;

    SharedContext sharedContext_;
    SystemConfig systemConfig_;
    HardwareThread *hardwareThread_ = nullptr;
    PlannerThread *plannerThread_ = nullptr;
    UiRefreshController *uiRefreshController_ = nullptr;
    bool boardInitialized_ = false;
    bool curveCaptureActive_ = false;
    bool lastMotionRunning_ = false;
};

#endif // MAINWINDOW_H
