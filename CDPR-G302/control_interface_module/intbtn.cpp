#include "intbtn.h"

/*
 * 文件总览：
 * - IntBtn 的实现文件，保存按钮编号并在鼠标按下时通过 sendInt 发送。
 */

IntBtn::IntBtn(QWidget *parent) : QPushButton(parent){
}

void IntBtn::setInt(int _x){
    x = _x;
}

void IntBtn::mousePressEvent(QMouseEvent *ev){
    emit sendInt(x);
}
