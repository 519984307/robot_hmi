#include "shadowwidget.h"
#include <QHBoxLayout>
#include <QGraphicsDropShadowEffect>

ShadowWidget::ShadowWidget(QWidget* parent) : QWidget(parent)
{
    QGraphicsDropShadowEffect * effect = new QGraphicsDropShadowEffect(this);
    effect->setOffset(0, 0);//设置阴影距离
    effect->setColor(QColor(0,0,0,90));//设置阴影颜色
    effect->setBlurRadius(15);//设置阴影圆角
    this->setStyleSheet(".QWidget{background-color:#FFFFFF;border-radius:6px;}");
    this->setGraphicsEffect(effect);
//    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
//    setAttribute(Qt::WA_TranslucentBackground);
//    QWidget *pCentralWidget = new QWidget(this);
//    pCentralWidget->setStyleSheet("background-color: white");
//    QHBoxLayout *pLayout = new QHBoxLayout(this);
//    pLayout->addWidget(pCentralWidget);
//    pLayout->setContentsMargins(20, 20, 20, 20);

//    QGraphicsDropShadowEffect *pEffect = new QGraphicsDropShadowEffect(pCentralWidget);
//    pEffect->setOffset(0, 0);
//    pEffect->setColor(QColor(QStringLiteral("black")));
//    pEffect->setBlurRadius(30);
//    pCentralWidget->setGraphicsEffect(pEffect);
}
