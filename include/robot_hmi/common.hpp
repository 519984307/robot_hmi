#ifndef COMMON_HPP
#define COMMON_HPP

#include <math.h>

#include <QString>
#include <QPair>
#include <QMetaType>
#include <QDataStream>
#include <QVector>
#include <QSettings>
#include <QDebug>
#include <QDir>
#include <QPoint>
#include <QThread>


//#include "
namespace robot_hmi {
struct road_data{
    int type=0;    // 0 is straight line
    struct straight_road{   // 直线
        double len=0;
        int num=0;
        QVector<QPair<double,double>> combin; // 前进与后退组合
    }st_road;
    struct cur_road{
        double len=0;
        double cur=0;
    }c_road;
    double radius=0;  // 半径
    friend QDataStream & operator << (QDataStream &arch, const road_data & object)
    {
        arch << object.type;
        arch << object.st_road.len << object.st_road.num << object.st_road.combin;
        arch << object.c_road.len << object.c_road.cur << object.radius;
        return arch;
    }

    friend QDataStream & operator >> (QDataStream &arch, road_data & object)
    {
        arch >> object.type;
        arch >> object.st_road.len >> object.st_road.num >> object.st_road.combin;
        arch >> object.c_road.len >> object.c_road.cur >> object.radius;
        return arch;
    }
};


template <typename T>
struct state{   // 车辆的状态量
    T x;
    T y;
    T t;

    inline qreal manhattanLength() const{
        return sqrtf(x*x+y*y);
    };
    state &operator<<(state &s){ x=s.x;  y=s.y;  t=s.t; }
    state &operator+=(state &s){ x+=s.x; y+=s.y; t+=s.t;}
    state &operator-=(state &s){ x-=s.x; y-=s.y; t-=s.t;}
    state operator-(const state&s){
        state s0;
        s0.x = x-s.x;
        s0.y = y-s.y;
        s0.t = t-s.t;
        return s0;
    }
    state operator+(const state&s){
        state s0;
        s0.x = x+s.x;
        s0.y = y+s.y;
        s0.t = t+s.t;
        return s0;
    }
    state operator*(const state&s){
        return x*s.x + y*s.y;
    }
};
}

Q_DECLARE_METATYPE(robot_hmi::road_data)

#define STATE robot_hmi::state<double>

/**
 * @brief 纯跟踪算法的数据结构体
 */
struct CPP{
    double ld;  // lookahead
    double L;   // car length

};
Q_DECLARE_METATYPE(CPP)


struct UGV{
    double vmin, vmax;  // vel
    double amin, amax;  // turn vel
    STATE s;  // 车辆当前状态
    double lmax;    // 最大预瞄距离
    UGV(){
        vmin=-100;vmax=100;amin=-100; amax=100;
        lmax = 100;
    }
    void setRange(QPair<double, double> vel){
        vmax = vel.first;
        vmin = vel.second;
    }
    void setRange(QPair<double, double> vel, QPair<double, double> ang){
        vmin = vel.first;
        vmax = vel.second;
        amin = ang.first;
        amax = ang.second;
    }
};

#endif // COMMON_HPP
