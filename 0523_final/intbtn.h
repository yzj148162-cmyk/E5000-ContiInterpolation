#ifndef INTBTN_H
#define INTBTN_H

#include <QWidget>
#include <QPushButton>

class IntBtn : public QPushButton
{
    Q_OBJECT
public:
    explicit IntBtn(QWidget *parent = nullptr);
    void mousePressEvent(QMouseEvent* ev);
    void setInt(int _x);
private:
    int x = -1;
signals:
    void sendInt(int _x);
};

#endif // INTBTN_H
