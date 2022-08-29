
#include "plotpanel.h"
#include <Eigen/Core>
#include <QFileDialog>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <fstream>
#include <iostream>
#include<QDebug>

PlotRoad::PlotRoad(QWidget* parent)
{
    mPlotMain = new QCustomPlot(this);
    mPlotMain->setInteraction(QCP::iRangeDrag, true);
    mPlotMain->setInteraction(QCP::iRangeZoom, true);
    //xAxis（下）yAxis（左）xAxis2（上）yAxis2（右）
    mPlotMain->xAxis->setTickLabels(true);
    mPlotMain->yAxis->setTickLabels(true);
    mPlotMain->xAxis2->setVisible(true);
    mPlotMain->yAxis2->setVisible(true);
    mPlotMain->xAxis2->setTickLabels(false);
    mPlotMain->yAxis2->setTickLabels(false);

    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(mPlotMain);
    setLayout(main_layout);

    // create graphs:
    // 全局路径
    mCurveMain = new QCPCurve(mPlotMain->xAxis, mPlotMain->yAxis);
    mDotMain = new QCPCurve(mPlotMain->xAxis, mPlotMain->yAxis);
    mCarMain = new QCPCurve(mPlotMain->xAxis, mPlotMain->yAxis);
    mPath = new QCPCurve(mPlotMain->xAxis, mPlotMain->yAxis);

    QPen pen;
    pen.setColor(QColor(46, 204, 250));
    pen.setWidthF(5);
    mCurveMain->setPen(pen);

    pen.setColor(QColor(40, 41, 42));
    pen.setWidthF(8);
    mDotMain->setPen(pen);
    mDotMain->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    connect(mPlotMain,&QCustomPlot::mouseMove,this,&PlotRoad::myMoveEvent);

    pen.setColor(QColor(153, 50, 204));
    pen.setWidthF(3);
    mPath->setPen(pen);
//    mPath->setLineStyle()
}

PlotRoad::~PlotRoad()
{
}

void PlotRoad::plotPolyLine(QVector<QVector<double>> fb){
    isInit=true;
//    mPlotMain->xAxis->setTickLabels(false);
    mPlotMain->yAxis->setTickLabels(false);
    QCPRange range;
    range.lower=fb[0][0];
    range.upper=fb[0][0];
    for(int i=0; i<fb.size(); i++){
        QCPItemLine *arrow = new QCPItemLine(mPlotMain);
        QPen pen;
        pen.setColor(QColor(46, 204, 250));
        pen.setWidthF(5);
        arrow->setPen(pen);
        arrow->start->setCoords(fb[i].first(), i);
        arrow->end->setCoords(fb[i].back(), i);
        arrow->setHead(QCPLineEnding::esSpikeArrow);
        arrows.append(arrow);

        // add the text label at the top:
        QCPItemText *textLabel = new QCPItemText(mPlotMain);
        textLabel->position->setCoords(fb[i].first(), i); // place position at center/top of axis rect
        if(i%2==0){
            textLabel->setPositionAlignment(Qt::AlignLeft);
            textLabel->setText("F"+QString::number(i/2));
        }
        else{
            textLabel->setPositionAlignment(Qt::AlignRight);
            textLabel->setText("B"+QString::number(i/2));
        }
        textLabel->setFont(QFont(font().family(), 16)); // make font a bit larger
        arrows.append(textLabel);

        range.lower= fmin(range.lower, fmin(fb[i][0],fb[i][1]));
        range.upper= fmax(range.upper, fmax(fb[i][0],fb[i][1]));
    }

    mPlotMain->xAxis->rescale();
    mPlotMain->yAxis->rescale();
    range.lower -= 2;
    range.upper += 2;
    mPlotMain->xAxis->setRange(range);
    mPlotMain->yAxis->setRange({-1, fb.size()+1.0});
    mPlotMain->replot();    // 更新画布

}

void PlotRoad::plotxy(QVector<double> px, QVector<double> py){
    isInit=true;
    mPlotMain->xAxis->setTickLabels(true);
    mPlotMain->yAxis->setTickLabels(true);
    mCurveMain->setData(px,py);
    // make key axis range scroll with the data:
    mPlotMain->xAxis->rescale();
    mPlotMain->yAxis->rescale();
    mCurveMain->rescaleValueAxis(true, true);

    QCPRange range;
    range = mPlotMain->xAxis->range();
    range.lower -= 2;
    range.upper += 2;
    mPlotMain->xAxis->setRange(range);
    range = mPlotMain->yAxis->range();
    range.lower -= 2;
    range.upper += 2;
    mPlotMain->yAxis->setRange(range);
    mPlotMain->replot();    // 更新画布
}

void PlotRoad::plotDot(double x, double y){
    isInit=true;
    mDotMain->setData({x},{y});
    mPlotMain->replot();    // 更新画布
}

void PlotRoad::addPath(QVector<QVector<double>> path){
    QCPCurve *p = new QCPCurve(mPlotMain->xAxis, mPlotMain->yAxis);
    p->addData(path[0],path[1]);
    addCurve.push_back(p);
    mPlotMain->replot();
}

void PlotRoad::plotCar(double x, double y, double theta){
    QMatrix matrix;
    matrix.rotate(-180*theta/M_PI);  // 将图片旋转 顺时针旋转为正
    QCPScatterStyle a;
    a.setPixmap(QPixmap("://images/car.png")
                .transformed(matrix, Qt::SmoothTransformation)
                .scaled(40, 40,Qt::KeepAspectRatio));   // 添加车辆图标
    mCarMain->setScatterStyle(a);
    mCarMain->setData({x},{y});
//    qDebug() << "car x:" << x << ",y:" << y << ",theta:" << endl;
    mPlotMain->replot();    // 更新画布
}

void PlotRoad::plotPath(double x, double y){
    mPath->addData(x,y);
    mPlotMain->replot();    // 更新画布
}

void PlotRoad::clearCurve(){
    if(isInit==false) return;
    mCurveMain->setData({},{}); // 有些曲线本来就是空的情况，用data()->clear() 会闪退
    mDotMain->setData({},{});
    mPath->setData({},{});

    if(!arrows.isEmpty()){
        for (auto arrow : arrows) {
            mPlotMain->removeItem(arrow);
        }
    }
    while(!arrows.isEmpty()) arrows.pop_back();

    if(!addCurve.isEmpty()){
        for(auto p: addCurve){
            mPlotMain->removePlottable(p);
        }
    }
    while(!addCurve.isEmpty()) addCurve.pop_back();
    mPlotMain->replot();    // 更新画布
}

void PlotRoad::clearPath(){
    mPath->setData({},{});
    mPlotMain->replot();    // 更新画布
}


void PlotRoad::myMoveEvent(QMouseEvent *event)
{

    double x = event->pos().x();
    double y = event->pos().y();

    double x_ = mPlotMain->xAxis->pixelToCoord(x);
    double y_ = mPlotMain->yAxis->pixelToCoord(y);

    QString str = QString("x:%1,y:%2").arg(QString::number(x_,'f',3))
            .arg(QString::number(y_,'f',3));

    QToolTip::showText(cursor().pos(),str,mPlotMain);

}
