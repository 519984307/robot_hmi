#include <QPushButton>
#include <QEvent>
#include <QMouseEvent>
#include <QTouchEvent>

class MyPushButton :public QPushButton
{
    Q_OBJECT
public:
    explicit MyPushButton(QWidget *parent = 0):QPushButton(parent){}
    void mouseDoubleClickEvent(QMouseEvent *event) {
        if(event->button() == Qt::LeftButton ) {
            emit doubleClicked();
        }
    }
signals:
    void doubleClicked();
};

