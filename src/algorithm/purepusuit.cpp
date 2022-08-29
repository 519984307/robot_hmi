/*  梁荣敏
 *  2022.08.25
 *  纯跟踪算法
*/

#include "purepusuit.h"
#include <math.h>

/**
 * @brief PurePusuit::PurePusuit
 *
 */
PurePusuit::PurePusuit(STATE s)
{
//    qDebug() << QDir::currentPath() << endl;
//    ReadIni("");
    start = s ;
    traj = new QVector<QVector<double>>(2);
    ugv.setRange({-1,1},{-1,1}); // 分别设置速度与角速度的范围 m/s rad/s
    ugv.lmax = 2;   // m    最大的预瞄距离
    SV = 0.15;   // 面积预瞄阈值
    ST = 0.2;  // 速度的面积预瞄阈值
}

/**
 * @brief 查找路径上距离车辆当前位置最近点
 * @param 车辆当前位置
 * @return 最近点位置
 */
QPointF PurePusuit::closest(QPointF point){
    double mindis=10000;
    double index=0;
    for(int i=lastInd; i<traj->at(0).size(); i++){
        double x = traj->at(0)[i];
        double y = traj->at(1)[i];
        double dis =sqrtf((point.x()-x)*(point.x()-x)+(point.y()-y)*(point.y()-y));
        if(dis < mindis){
            mindis=dis;
            lastInd=i;
        }
    }
    return {traj->at(0)[lastInd],traj->at(1)[lastInd]};
}

QPair<QPointF, QPair<double, double>> PurePusuit::cal_point_area(QPointF npos){
    auto tx = traj->at(0);
    auto ty = traj->at(1);
    int startid = lastInd;
    while(1){
        auto [velx, vely] = word2vehicle({tx[startid],ty[startid]}, ugv.s);
        if( velx >= 0 || tx.size()-startid < 5 ){  // 从x正半轴开始记录面积
            break;
        }
        startid++;
    }
    QVector<double> area;
    QVector<QPointF> areaPath;
    for(int i=startid; i<tx.size();i++){
        auto [v_x, v_y] = word2vehicle({tx[i],ty[i]},ugv.s);
        areaPath.push_back({v_x,v_y});
        if(i>fmin(100+startid,tx.size())
           || (tx[i]-ugv.s.x)*(tx[i]-ugv.s.x) + (ty[i]-ugv.s.y)*(ty[i]-ugv.s.y)>100 ){  // 预瞄距离最远10米
            break;
        }
    }
    double areaS = -1;  // 面积
    double areaV = -1;  //
    double s=0;
    QPointF lpoint; // 角速度的预瞄点
    QPointF vpoint; // 速度的预瞄点
    for(int i=1; i<areaPath.size(); i++){
        s=0; //
        for( int j= 1; j<i-1; j++){
            double dis = fabs(areaPath[j-1].x()-areaPath[j].x());
            double e = areaPath[j-1].y()+areaPath[j].y();   //y轴就是垂线
            s = s + fabs(e*dis)/2;
        }
        area.push_back(s);

        QPointF tmp = areaPath[i];
        double len = sqrtf(tmp.x()*tmp.x()+tmp.y()*tmp.y());
        if(lpoint.isNull() && ( fabs(s)> ST || len > ugv.lmax)){
            lpoint = {tx[startid+i], ty[startid+i]};
            areaS = s;
        }
        if(vpoint.isNull() && ( fabs(s)> SV || len > ugv.lmax)){
            vpoint = {tx[startid+i], ty[startid+i]};
            areaV = s;
        }
        if(!lpoint.isNull()&& !vpoint.isNull() && ( len > ugv.lmax)){
            break;
        }
    }
    if(lpoint.isNull())
        lpoint = {tx[startid+areaPath.size()-1],ty[startid+areaPath.size()-1]};
    if(vpoint.isNull())
        vpoint = {tx[startid+areaPath.size()-1],ty[startid+areaPath.size()-1]};

    if( areaS == -1 )
        areaS = s;
    if( areaV == -1)
        areaV = s;

    return {lpoint, {areaS,areaV} };
}

/**
 * @brief 设置跟踪路径
 * @param path
 */
void PurePusuit::setPath(QVector<QVector<double>> path){

    qDebug()<<traj->size() << endl;
    traj->clear();
    traj->append(path[0]);
    traj->append(path[1]);
}

/**
 * @brief 对路径预处理，平滑
 * @param path,
 */
QVector<QVector<double>> PurePusuit::SmoothPath(QVector<QVector<double>> path){
    return path;
}

/**
 * @brief 根据车辆当前点，计算出预瞄点与控制量
 * @param 车辆当前位置
 * @return 控制量
 */
QPair<double, double> PurePusuit::track_path(STATE sv){
    auto np = closest({sv.x,sv.y});   //
    ugv.s = sv; // 车辆当前状态


    auto [lpoint, area] = cal_point_area(np);
    double s = area.first;
    QPointF dis = {lpoint.x()-traj->at(0).back(),lpoint.y()-traj->at(1).back()};
    if(dis.manhattanLength() < 2 ){ //

    }
    double alpha = atan2(lpoint.y()-sv.y, lpoint.x()-sv.x) ;
    qDebug() << "area:" << s << " alpha:" << alpha << " vt:" << sv.t << endl;
    alpha -= sv.t;
    if( alpha<-M_PI) alpha += 2*M_PI;
    else if(alpha>M_PI) alpha -= 2*M_PI;

    double ref_v;
    if (fabs(alpha) < 0.0001)   //% 偏差角小于0.01,近似为0,a/sin(a) = 1
        ref_v = ugv.vmax*SV/(s+SV);
    else
        ref_v = fabs(alpha)*ugv.vmax*SV/( (s+SV)*sin(fabs(alpha)) );
    qDebug() << "alpha:" << alpha << " vel:" << ref_v << " x/sinx " << fabs(alpha)/sin(fabs(alpha)) << endl;
    ref_v = fmax(ugv.vmin, fmin(ref_v, ugv.vmax) );
    QPointF a = {sv.x-lpoint.x(),sv.y-lpoint.y()};
    double Ld = sqrtf( (sv.x-lpoint.x())*(sv.x-lpoint.x())+(sv.y-lpoint.y())*(sv.y-lpoint.y()) );//a.manhattanLength();

    if(Ld < 0.05){
        ref_v = 0;
        emit track_finish();
    }
    else if( Ld < 0.5 ){
        ref_v = fmin(0.1, ref_v);
    }
//     转向角速度
    double omega;
    if(Ld >0.001)
        omega = 2*ref_v*sin(alpha)/Ld;
    else
        omega = 0.001;
    qDebug() << "v:" << ref_v << "omega:" << omega << endl;
    return {ref_v, omega};
}

/**
 * @brief PurePusuit::ReadIni
 *        读取配置文件
 * @param path
 */
void PurePusuit::ReadIni(QString path){
    QSettings* settings = new QSettings("config.ini",QSettings::IniFormat);
    settings->beginGroup("user");
    QStringList player_number = settings->childGroups(); // returns 0, 1, 2 - OK !
    const QStringList childKeys = settings->childKeys(); // should return name, wins, ... right ?
    foreach(const QString &childKey, childKeys)
    {
        qDebug() << settings->value(childKey).toString();  // should add lukasz, 3, 3, pawel...., but it doesn`t work

    }
    settings->endGroup();
}

/**
 * @brief 将坐标转换到车辆坐标系
 * @param wpos 局部坐标系
 * @param vpos 车辆坐标系
 * @return
 */
QPair<double, double> PurePusuit::word2vehicle(QPointF wpos, STATE vpos){
    double t = vpos.t;
    double x = (wpos.x()-vpos.x)*cos(t)+(wpos.y()-vpos.y)*sin(t);
    double y = -(wpos.x()-vpos.x)*sin(t)+(wpos.y()-vpos.y)*cos(t);
    return {x,y};
}
