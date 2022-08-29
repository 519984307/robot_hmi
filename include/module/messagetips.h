#ifndef MESSAGETIPS_H
#define MESSAGETIPS_H

#include <QWidget>
# pragma execution_character_set("utf-8")

class QHBoxLayout;
class QLabel;
class MessageTips : public QWidget
{
    Q_OBJECT
public:
    explicit MessageTips(QString showStr="默认显示", QWidget *parent = nullptr);

    double getOpacityValue() const;
    void setOpacityValue(double value);

    int getTextSize() const;
    void setTextSize(int value);

    QColor getTextColor() const;
    void setTextColor(const QColor &value);

    QColor getBackgroundColor() const;
    void setBackgroundColor(const QColor &value);

    QColor getFrameColor() const;
    void setFrameColor(const QColor &value);

    int getFrameSize() const;
    void setFrameSize(int value);

    int getShowTime() const;
    void setShowTime(int value);

    void setCloseTimeSpeed(int closeTime = 100,double closeSpeed = 0.1);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void InitLayout();//初始化窗体的布局和部件
    QHBoxLayout *hBoxlayout;//布局显示控件布局
    QLabel *mText;//用于显示文字的控件
    QString showStr;//显示的字符串

    double opacityValue;//窗体初始化透明度

    QFont *font;
    int     textSize;//显示字体大小
    QColor  textColor;//字体颜色

    QColor  backgroundColor;//窗体的背景色
    QColor  frameColor;//边框颜色
    int     frameSize;//边框粗细大小

    int     showTime;//显示时间

    int     closeTime;//关闭需要时间
    double  closeSpeed;//窗体消失的平滑度，大小0~1


signals:

};

#endif // MESSAGETIPS_H


