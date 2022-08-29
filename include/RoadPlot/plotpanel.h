#ifndef PLOTPANEL_H
#define PLOTPANEL_H

#include "axistag.h"
#include "qcustomplot.h"
#include <QWidget>

//
class PlotRoad : public QWidget {
    Q_OBJECT
public:
    explicit PlotRoad(QWidget* parent = nullptr);
    ~PlotRoad();
    void plotxy(QVector<double> px,QVector<double> py);
    void plotDot(double x, double y);
    void plotCar(double x, double y, double theta);
    void clearCurve();
    void clearPath();
    void plotPath(double x, double y);
    void plotPolyLine(QVector<QVector<double>> fb);
    void myMoveEvent(QMouseEvent *event);       // 实时显示鼠标在图像的位置
    void addPath(QVector<QVector<double>> path);    //

//public slots:
//    void update_mainmap_slots();
//    void update_focusmap_slots(double x, double y);
//    void read_data_slots();

protected:
    QPushButton* button;
    QLabel* label;
    QCustomPlot* mPlotMain;
    QPointer<QCPCurve> mCurveMain;
    QPointer<QCPCurve> mDotMain;
    QPointer<QCPCurve> mCarMain;
    QPointer<QCPCurve> mPath;
    QVector<QCPCurve*> addCurve;
//    QVector<QPointer<QCPCurve>> polyLine;
    QVector<QCPAbstractItem*> arrows;

    QPixmap *pmap;
    bool isInit=false;
};

#endif // PLOTPANEL_H
