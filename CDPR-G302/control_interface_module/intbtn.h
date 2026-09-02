#ifndef INTBTN_H
#define INTBTN_H

/*
 * 文件总览：
 * - IntBtn 是携带整数编号的 QPushButton，点击时把编号随信号发出。
 * - 适合多轴、多通道按钮共用同一个槽函数时区分来源。
 */

#include <QWidget>
#include <QPushButton>

class IntBtn : public QPushButton
{
    Q_OBJECT
public:
    // 创建可携带整数编号的按钮。
    explicit IntBtn(QWidget *parent = nullptr);
    // 鼠标按下时发出 sendInt，把内部编号传给共享槽函数。
    void mousePressEvent(QMouseEvent* ev);
    // 设置该按钮代表的编号，例如轴号或通道号。
    void setInt(int _x);
private:
    int x = -1;
signals:
    void sendInt(int _x);
};

#endif // INTBTN_H
