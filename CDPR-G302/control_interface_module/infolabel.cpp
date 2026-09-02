#include "infolabel.h"

/*
 * 文件总览：
 * - InfoLabel 的实现文件，只维护一段提示文本，并在鼠标进入标签时发出 showTip 信号。
 */

InfoLabel::InfoLabel(QWidget *parent) : QLabel(parent){
    this->setMouseTracking(true);
}

void InfoLabel::setInfo(std::string _info){
    info = _info;
}

void InfoLabel::mouseEnterEvent(QMouseEvent *ev){
    emit showTip(info);
}

void InfoLabel::showTip(std::string _info){
//    this->setToolTip();
}
