#ifndef INFOLABEL_H
#define INFOLABEL_H

#include <QWidget>
#include <QLabel>

class InfoLabel : public QLabel
{
    Q_OBJECT
public:
    explicit InfoLabel(QWidget *parent = nullptr);
    void mouseEnterEvent(QMouseEvent* ev);
    void setInfo(std::string _info);
    void showTip(std::string _info);
private:
    std::string info;
};

#endif // INFOLABEL_H
