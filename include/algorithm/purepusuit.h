#ifndef PUREPUSUIT_H
#define PUREPUSUIT_H

#include "common.hpp"



/**
 * @brief The PurePusuit class
 */
class PurePusuit:public QThread
{
     Q_OBJECT
public:
    PurePusuit(STATE s={0,0,0});
    QPointF closest(QPointF point);
    QPair<QPointF, QPair<double, double>> cal_point_area(QPointF npos);    // 计算面积
    void setPath(QVector<QVector<double>> path);
    void setVelRange(double vmin, double vmax);
    void setAngVelRange(double amin, double amax);
    void ReadIni(QString path);
    QVector<QVector<double>> SmoothPath(QVector<QVector<double>> path);
    QPair<double, double> track_path(STATE sv);    // 返回控制量，线速度与角速度
    void reset(){lastInd=0;};
    QPair<double, double> word2vehicle(QPointF wpos, STATE vpos);

    void operator()(STATE s){
        start = s;
    };
    void operator()(QVector<QVector<double>> *tj){
        traj = tj;
    };

Q_SIGNALS:
    void track_finish();

private:
    STATE start;
    STATE goal;
    QVector<QVector<double>> *traj;
    UGV ugv;
    int lastInd=0;  // 记录最近点下标，防止倒退
    double SV, ST;  // 面积与速度阈值
};

#endif // PUREPUSUIT_H
