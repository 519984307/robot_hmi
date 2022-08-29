#include "messagetips.h"

#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QTimer>
#include<QApplication>

MessageTips::MessageTips(QString showStr,QWidget *parent) : QWidget(parent),
   opacityValue(1.0),
   textSize(18),
   textColor(QColor(255,255,255)),
   backgroundColor(QColor(192,192,192)),
   frameColor(QColor(211,211,211)),
   frameSize(2),
   showTime(3500),
   closeTime(100),
   closeSpeed(0.1),
   hBoxlayout(new QHBoxLayout(this)),
   mText(new QLabel(this))
{
    setWindowFlags(Qt::Window|Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint|Qt::Tool|Qt::X11BypassWindowManagerHint);
    this->setAttribute(Qt::WA_TranslucentBackground); // ****这里很重要****
    this->setAttribute(Qt::WA_TransparentForMouseEvents, true);// 禁止鼠标事件
    this->showStr = showStr;
    hBoxlayout->addWidget(mText);
    InitLayout();
}

void MessageTips::InitLayout()
{
    this->setWindowOpacity(opacityValue);

    //文字显示居中，设置字体，大小，颜色
    font = new QFont("微软雅黑",textSize,QFont::Bold);
    mText->setFont(*font);
    mText->setText(showStr);
    mText->setAlignment(Qt::AlignCenter);
    QPalette label_pe;//设置字体颜色
    label_pe.setColor(QPalette::WindowText, textColor);
    mText->setPalette(label_pe);

    QTimer *mtimer = new QTimer(this);//隐藏的定时器
    mtimer->setTimerType(Qt::PreciseTimer);
    connect(mtimer,&QTimer::timeout,this,[=](){
        if(opacityValue<=0){ this->close(); }
        opacityValue = opacityValue-closeSpeed;
        this->setWindowOpacity(opacityValue);    //设置窗口透明度
        });


    QTimer *mShowtimer = new QTimer(this);//显示时间的定时器
    mShowtimer->setTimerType(Qt::PreciseTimer);// 修改定时器对象的精度
    connect(mShowtimer,&QTimer::timeout,this,[=](){
        mtimer->start(closeTime);//执行延时自动关闭
        });
    mShowtimer->start(showTime);

    //设置屏幕居中显示
    QDesktopWidget* desktop = QApplication::desktop(); // =qApp->desktop();也可以
    this->move((desktop->width() - this->width())/2, (desktop->height() - this->height())/2);
    this->setAttribute(Qt::WA_TransparentForMouseEvents, true);// 禁止鼠标事件
}

void MessageTips::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(QBrush(backgroundColor));//窗体的背景色

    painter.setPen(QPen(frameColor,frameSize));//窗体边框的颜色和笔画大小
    QRectF rect(0, 0, this->width(), this->height());
    painter.drawRoundedRect(rect, 15, 15); // round rect
}




//设置关闭的时间和速度，speed大小限定0~1
void MessageTips::setCloseTimeSpeed(int closeTime, double closeSpeed)
{
    if(closeSpeed>0 && closeSpeed<=1){
       this->closeSpeed = closeSpeed;
    }
   this->closeTime = closeTime;
    InitLayout();
}


int MessageTips::getShowTime() const
{
    return showTime;
}

void MessageTips::setShowTime(int value)
{
    showTime = value;
    InitLayout();
}

int MessageTips::getFrameSize() const
{
    return frameSize;
}

void MessageTips::setFrameSize(int value)
{
    frameSize = value;
}

QColor MessageTips::getFrameColor() const
{
    return frameColor;
}

void MessageTips::setFrameColor(const QColor &value)
{
    frameColor = value;
}

QColor MessageTips::getBackgroundColor() const
{
    return backgroundColor;
}

void MessageTips::setBackgroundColor(const QColor &value)
{
    backgroundColor = value;
}

QColor MessageTips::getTextColor() const
{
    return textColor;
}

void MessageTips::setTextColor(const QColor &value)
{
    textColor = value;
    InitLayout();
}

int MessageTips::getTextSize() const
{
    return textSize;
}

void MessageTips::setTextSize(int value)
{
    textSize = value;
    InitLayout();
}

double MessageTips::getOpacityValue() const
{
    return opacityValue;
}

void MessageTips::setOpacityValue(double value)
{
    opacityValue = value;
    InitLayout();
}


