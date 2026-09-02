#ifndef INFOLABEL_H
#define INFOLABEL_H

/*
 * 文件总览：
 * - InfoLabel 是带提示信息的 QLabel 派生类，鼠标进入时向外发送当前说明文本。
 * - 主要用于界面参数说明或状态提示，不参与控制逻辑。
 */

#include <QWidget>
#include <QLabel>

class InfoLabel : public QLabel
{
    Q_OBJECT
public:
    // 创建带说明信息的 QLabel。
    explicit InfoLabel(QWidget *parent = nullptr);
    // 鼠标进入时弹出/发送当前提示说明。
    void mouseEnterEvent(QMouseEvent* ev);
    // 设置当前标签对应的说明文本。
    void setInfo(std::string _info);
    // 立即显示指定说明文本。
    void showTip(std::string _info);
private:
    std::string info;
};

#endif // INFOLABEL_H
