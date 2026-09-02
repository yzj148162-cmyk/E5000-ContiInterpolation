/*
 * 文件总览：
 * - Qt 程序入口，创建 QApplication 并启动主窗口 MainWindow。
 * - 本文件不包含业务逻辑，所有系统初始化、硬件连接、线程调度和 UI 事件都在 MainWindow 内展开。
 */

#include "mainwindow.h"

#include <QApplication>
#include <QCoreApplication>

#ifndef MOTION_CONTROL_APP_VERSION
#define MOTION_CONTROL_APP_VERSION "1.0.0"
#endif

// Qt 应用入口：创建 QApplication，设置应用名并显示主窗口。
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("MotionControl"));
    QCoreApplication::setApplicationVersion(QStringLiteral(MOTION_CONTROL_APP_VERSION));

    MainWindow w;
    w.show();

    return a.exec();
}
