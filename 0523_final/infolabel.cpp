#include "infolabel.h"

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
