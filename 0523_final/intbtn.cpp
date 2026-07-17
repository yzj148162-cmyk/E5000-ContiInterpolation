#include "intbtn.h"

IntBtn::IntBtn(QWidget *parent) : QPushButton(parent){
}

void IntBtn::setInt(int _x){
    x = _x;
}

void IntBtn::mousePressEvent(QMouseEvent *ev){
    emit sendInt(x);
}
