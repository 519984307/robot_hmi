/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date 2022.08
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QtGlobal>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QHostAddress>
#include <QFileDialog>
#include <QGraphicsDropShadowEffect>
#include <iostream>

#include <QtConcurrent/QtConcurrent>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "../include/robot_hmi/main_window.hpp"
#include "messagetips.h"



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    ui_init();

//    获取rosmaster ip 和rosip ip
     foreach (QHostAddress address, QNetworkInterface::allAddresses()) { //
       if (address.protocol() == QAbstractSocket::IPv4Protocol) {
         QString addre = address.toString();
         if (addre.split(".")[0] == "192") {
           car0_qRosIp = addre;
           car0_qMasterIp = "http://" + addre + ":11311";
         } else if (addre.split(".")[0] == "10") {
           car0_qRosIp = addre;
           car0_qMasterIp = "http://" + addre + ":11311";
         } else if (addre.split(".")[0] == "172") {
           car0_qRosIp = addre;
           car0_qMasterIp = "http://" + addre + ":11311";
         }
       }
     }
//    car0_qRosIp = "127.0.0.1";
//    car0_qMasterIp = "http://127.0.0.1:11311";
    
    ReadSettings();

    track = new PurePusuit();   //

    connect_init(); // initial connect


}

void MainWindow::ui_init(){

//    setShadowEffect(ui.param_widget);
//        setShadowEffect(ui.info_widget);
    setShadowEffect(ui.state_widget);   // 设置阴影
    setShadowEffect(ui.widget_plat);
//        setShadowEffect(ui.widget_view);
    setShadowEffect(ui.widget_plot);
//        setShadowEffect(ui.rviz_widget);
    setShadowEffect(ui.map_widget);
//    setShadowEffect(ui.rviz_img);
    setShadowEffect(ui.panel_widget);
//    setShadowEffect(ui.rviz_panel);

//    QFile qss("://images/qss/whiteBackground.qss");  //Ubuntu.qss
    QFile qss(":/qdarkstyle/light/lightstyle.qss"); // qss 设置界面皮肤
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();

    // 摇杆控制初始化
    vel_joy = new JoyStick(ui.velControl);
    omg_joy = new JoyStick(ui.omgControl);
    setShadowEffect(vel_joy);
    setShadowEffect(omg_joy);
    vel_joy->show();
    omg_joy->show();

    spacer = ui.space_path; // 弹簧
    rpanel = ui.rviz_panel; // rviz panel
    plot = ui.plot_road;
}

void MainWindow::connect_init(){
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(ui.image_file, &QAction::triggered,this, [this]{
        QString existPath= img_filepath;//==""?"":img_filepath;
        img_filepath = QFileDialog::getExistingDirectory(this,tr("截图保存路径"),existPath);
    });     // set where to save image

    connect(ui.car0_connect, &MyPushButton::doubleClicked,this, [this]{slot_car_connect(1);});
    connect(ui.car1_connect, &MyPushButton::doubleClicked,this, [this]{slot_car_connect(2);});
    connect(ui.indoor, &MyPushButton::doubleClicked,this, [this]{
        ui.indoor->setChecked(false);
        ui.outdoor->setStyleSheet("color:black;");
        ui.indoor->setStyleSheet("color:blue;");
    });
    connect(ui.outdoor, &MyPushButton::doubleClicked,this,[this]{
        ui.outdoor->setChecked(false);
        ui.indoor->setStyleSheet("color:black;");
        ui.outdoor->setStyleSheet("color:blue;");
    });

    // 左摇杆 线速度控制
    connect(vel_joy, &JoyStick::keyPosChanged, this, [this](QPointF pos){
        vel.first = pos.y();
        ui.pos_x->setText(QString::number(pos.x()));
        ui.pos_y->setText(QString::number(pos.y()));
    });
    //  右摇杆 加速度控制
    connect(omg_joy, &JoyStick::keyPosChanged, this, [this](QPointF pos){
        vel.second = pos.x();
        ui.pos_x_2->setText(QString::number(pos.x()));
        ui.pos_y_2->setText(QString::number(pos.y()));
    });
    connect(ui.refresh_vel,&QPushButton::clicked,this, [this]{
        if(!qnode.initFlag) return; //还未连接，不能刷新
        changeComboex(ui.topics_vel,"geometry_msgs/Twist");
    });
    connect(ui.topics_vel, QOverload<int>::of(&QComboBox::currentIndexChanged), this,[this]{
        if(!qnode.initFlag) return;

    });
    cmd_time = new QTimer(this);
    cmd_time->start(50);    // 控制周期为50 ms
    connect(cmd_time,&QTimer::timeout, this, &MainWindow::set_track_path);

    connect(ui.refresh, SIGNAL(clicked(bool)), this, SLOT(updateImgTopicList()));
    connect(ui.refresh_map,&QPushButton::clicked, this, &MainWindow::updateMapTopicList);
    connect(ui.topics_map, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicMapChanged(int)) );
    connect(ui.topics_laser, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicLaserChanged(int)) );
    connect(ui.topics_img, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
    connect(ui.snapshoot, SIGNAL(pressed()), this, SLOT(saveImage()));  // save image
    connect(ui.set_goal, &QPushButton::pressed,this, [this]{
        if(!qnode.initFlag) return;
        rpanel->Set_Goal_Pose();
    });


    //connect ros node
    connect(&qnode, SIGNAL(connectMasterSuccess(int)), this, SLOT(connectSuccess(int)));
    connect(&qnode, SIGNAL(connectMasterFailed(int)), this, SLOT(connectFailed(int)));
    connect(&qnode, SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));
    connect(&qnode, &QNode::gps_pos, this, &MainWindow::slot_gps);
    connect(&qnode, &QNode::position, this, &MainWindow::local_coor);
    connect(&qnode, &QNode::speed_vel, this, &MainWindow::slot_update_dashboard);


    slot_set_param(road.type);
    ui.selected_path->setCurrentIndex(road.type);   //
    connect(ui.selected_path, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index){
        isTracking=false;
        st_state=0;
        slot_set_param(index);
    });
    connect(ui.stop_track, &QPushButton::clicked, this,&MainWindow::slot_stopTrack );
    connect(ui.track_path, &QPushButton::clicked,this, [this]{  // lambda function
        if(!qnode.initFlag) return;
        if(!track_state){
            track_state++;
            plot->clearPath();
            ui.track_path->setText("暂停跟踪");
            ui.track_path->setChecked(true);
            ui.track_path->setStyleSheet("color:blue;");
            pos0=posr; isTracking = true;
            plot->plotCar(0,0,0);
        }
        else if(track_state==1){
            if(!isTracking){
                ui.track_path->setText("暂停跟踪");
                ui.track_path->setChecked(true);
                ui.track_path->setStyleSheet("color:blue;");
                isTracking=true;
            } else {
                ui.track_path->setText("继续跟踪");
                ui.track_path->setChecked(false);
                ui.track_path->setStyleSheet("color:black;");
                isTracking=false;
            }
        }
    });
    connect(track,&PurePusuit::track_finish, this, [this]{
        isTracking=false;
        st_state=0;
        track_state=0;
        ui.track_path->setText("开始跟踪");
        ui.track_path->setChecked(false);
        ui.track_path->setStyleSheet("color:black;");
    });


    timer = new QTimer(this);   // image timer
    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

void MainWindow::slot_stopTrack(){
    if(!qnode.initFlag) return;
    track_state=0;isTracking=false;
    ui.track_path->setText("开始跟踪");
    ui.track_path->setChecked(false);
    ui.track_path->setStyleSheet("color:black;");
    plot->clearPath();
    st_state=0;plot->plotCar(0,0,0);
    track->reset();
}

void MainWindow::slot_gps(int posqual, int headingqual, double x, double y){
    QString text[]={"定位不可用","单点定位","RTK 浮点解","RTK 固定解"};
    ui.posqual->setText(text[posqual]);
    ui.headingqual->setText(text[headingqual]);
    ui.gps_east->setText("east: "+QString::number(x,'f',3)+"m ");
    ui.gps_north->setText("north: " + QString::number(y,'f',3)+"m ");
}

/**
 * @brief 利用定时器，将图片消息类型转换为QImage
 */
void MainWindow::timerSlot(){

    cv_bridge::CvImagePtr cv_ptr;
    auto msg = ui.rviz_img->getImage();
    if( msg == NULL) {
        //qDebug() << "img NULL!" << endl;
        return;
    }
    cv::Mat img;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '32fc1'.", msg->encoding.c_str());
        return;
    }
    QImage dest(img.cols, img.rows, QImage::Format_ARGB32);
    const float scale = 255.0;

    if (img.depth() == CV_8U) {
      if (img.channels() == 1) {
          for (int i = 0; i < img.rows; ++i) {
              for (int j = 0; j < img.cols; ++j) {
                  int level = img.at<quint8>(i, j);
                  dest.setPixel(j, i, qRgb(level, level, level));
              }
          }
      } else if (img.channels() == 3) {
          for (int i = 0; i < img.rows; ++i) {
              for (int j = 0; j < img.cols; ++j) {
                  cv::Vec3b bgr = img.at<cv::Vec3b>(i, j);
                  dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
              }
          }
      }
    } else if (img.depth() == CV_32F) {
      if (img.channels() == 1) {
          for (int i = 0; i < img.rows; ++i) {
              for (int j = 0; j < img.cols; ++j) {
                  int level = scale * img.at<float>(i, j);
                  dest.setPixel(j, i, qRgb(level, level, level));
              }
          }
      } else if (img.channels() == 3) {
          for (int i = 0; i < img.rows; ++i) {
              for (int j = 0; j < img.cols; ++j) {
                  cv::Vec3f bgr = scale * img.at<cv::Vec3f>(i, j);
                  dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
              }
          }
      }
    }
    img_vec.push_back(dest);
    //qDebug() << "save img by timer,state:" << img_state << "img number:" << img_vec.size() << "\n";
}

void MainWindow::startTimer(){
    //qDebug()<<"start timer" << endl;
    if(!timer->isActive())
        timer->start(500);  // timer interval
    img_vec.clear();    //
    ui.snapshoot->setText("停止截图");
    ui.snapshoot->setIcon(QIcon(QString::fromLocal8Bit("://images/end.png")));
}

void MainWindow::stopTimer(){
   // qDebug()<<"stop timer" << endl;
    ui.snapshoot->setText("开始截图");
    ui.snapshoot->setIcon(QIcon(QString::fromLocal8Bit("://images/start.png")));
    if(img_state == 2){
        if(timer->isActive())
            timer->stop();
        QString filepath;
        auto runFunc = [&](){
            QDateTime datetime;
            QString timestr=datetime.currentDateTime().toString("yyyy-MM-dd-HH-mm-ss");
            QDir dir;
            if (img_filepath.isEmpty())
            {
              img_filepath=dir.currentPath();
            }
            filepath = QString(img_filepath+"/"+timestr);
            // 检查目录是否存在，若不存在则新建

            if (!dir.exists(filepath))
            {
                bool res = dir.mkpath(filepath);
               // qDebug() << "新建目录成功" << res;
            }
            int cnt=0;
            //qDebug()<< "img number:" <<img_vec.size() << endl;
            for(QImage dest:img_vec){
                dest.save(filepath+"/"+QString::number(cnt)+"."+ui.select_format->currentText().toLower());
                cnt++;
            }
            //qDebug()<< "save images success" << endl;
        };

        QEventLoop loop;  // 创建一个事件循环对象
        // 创建一个线程执行状态观察者
        QFutureWatcher<void> watcher;
        //关联线程执行完毕信号，当线程执行完毕之后，退出QEventLoop的事件循环
        connect(&watcher, &QFutureWatcher<void>::finished, &loop, &QEventLoop::quit);

        // 运行并设置线程状态对象给观察者
        watcher.setFuture(QtConcurrent::run(runFunc));
        // 函数运行到此后阻塞，进入事件循环,等待退出回调
        loop.exec();

        // 退出了事件循环,函数继续往下执行
        // auto p=watcher.future.result();// 获取返回值，因为这里是void，所以没有返回值
        MessageTips *mMessageTips = new MessageTips(
                    "图片保存在路径:"+filepath //+"\n在菜单中设置保存路径"
                    ,this);
        mMessageTips->setStyleSheet("border-radius:5px;text-align:center;background: rgb(211, 215, 207); border: none;");
        mMessageTips->show();
        img_state = 0;
    }
}

/**
 * @brief 给界面设置阴影
 * @param w
 */
void MainWindow::setShadowEffect(QWidget * w)
{
    QGraphicsDropShadowEffect * effect = new QGraphicsDropShadowEffect(w);
    effect->setOffset(0, 0);//设置阴影距离
    effect->setColor(QColor(0,0,0,90));//设置阴影颜色
    effect->setBlurRadius(15);//设置阴影圆角
    w->setStyleSheet(".QWidget{background-color:#FFFFFF;border-radius:6px;}");
    w->setGraphicsEffect(effect);
}

void MainWindow::saveImage()
{
    if(!qnode.initFlag) return;
    switch (img_state) {
    case 0:
        img_state++;
        startTimer();
        break;
    case 1:
        img_state++;
        stopTimer();        
        break;
    default:
        //qDebug() << "state error!" << endl;
        break;
    }
}

void MainWindow::onTopicMapChanged(int index){
    if(!qnode.initFlag) return;
    //qDebug() << "set Map topic \n";
    QStringList parts = ui.topics_map->itemData(index).toString().split(" ");
    QString topic = parts.first();
    if(!topic.isEmpty())
        rpanel->mapDisplaySlot(topic);
}

void MainWindow::onTopicLaserChanged(int index){
    if(!qnode.initFlag) return;
   // qDebug() << "set Laser topic \n";
    QStringList parts = ui.topics_laser->itemData(index).toString().split(" ");
    QString topic = parts.first();
    if(!topic.isEmpty())
        rpanel->LaserDisplaySlot(topic);
}

void MainWindow::onTopicChanged(int index){
    if(!qnode.initFlag) return;
   // qDebug() << "set img topic \n";
    QStringList parts = ui.topics_img->itemData(index).toString().split(" ");
    QString topic = parts.first();
    if(!topic.isEmpty())
        ui.rviz_img->setTopicSlot(topic);
}


/**
 * @brief 将全局坐标转换为局部坐标
 * @param (x,y,\theta)
 * @return 返回局部坐标系下的坐标
 */
state<double> MainWindow::getXYT(state<double> xyt){

    state<double> s;
    s=xyt-pos0;
    s.x = s.x*cos(pos0.t)+s.y*sin(pos0.t);
    s.y = s.y*cos(pos0.t)-s.x*sin(pos0.t);
    return s;
}

void MainWindow::road_value_set(){
    QVector<double> px, py;
    auto vec = road.st_road.combin;
    plot->clearCurve();
    if(road.type==0){   // 直线来回倒退轨迹
        fb.clear();
        st_state=0;
        double left, right;
        left=0, right=vec[0].first;
        fb.push_back({left,right});
        left=right, right-=vec[0].second;
        fb.push_back({left,right});
        for (int i = 1; i < road.st_road.num; ++i) {
            left=right; right += vec[i].first;
            fb.push_back({left,right});
            left=right, right-=vec[i].second;
            fb.push_back({left,right});
        }
        plot->plotPolyLine(fb);
    }else if(road.type==1){ // S形轨迹
        double k=fmax(0.0001,road.c_road.cur), L = road.c_road.len;
        if( L == 0) return;
        double cx=L/4;
        double cy=sqrtf(1/(k*k)-L*L/16);
        double ang0=atan2(0-(-cy),0-cx);   // atan2 返回的是以x轴顺时针旋转方向，范围 (-pi, pi)
        double ang2=atan2(0-(-cy),L/2-cx);
        for(int i=0; i<100; i++){
            double ang = ang0+i*(ang2-ang0)/100;

            px.append(cx+cos(ang)/k);
            py.append(-cy+sin(ang)/k);
            qDebug() <<ang*180/3.14  << "x " << cx+cos(ang)/k <<",y" << -cy+sin(ang)/k <<endl;
        }
        ang0=atan2(0-cy,L/2-3*L/4);   // atan2 返回的是以x轴顺时针旋转方向，范围 (-pi, pi)
        ang2=atan2(0-cy,L-3*L/4);
        for(int i=0; i<=100; i++){
            double ang = ang0+i*(ang2-ang0)/100;
            px.append(3*cx+cos(ang)/k);
            py.append(cy+sin(ang)/k);
            qDebug() <<ang*180/3.14  << "x " << 3*cx+cos(ang)/k <<",y" << cy+sin(ang)/k <<endl;
        }
        qDebug() << "ang:" << ang0*180/3.14 << " " << ang2*180/3.14 << endl;
        plot->plotDot(L,0);
        plot->plotxy(px,py);
        track->setPath({px,py});

    }else if(road.type==2){ // 圆形轨迹
        double ang=2*M_PI/100;
        for(double i=M_PI; i>=-M_PI; i-=ang){
            px.append( road.radius+road.radius*cos(i) );
            py.append( road.radius*sin(i) );
        }
        plot->plotDot(2*road.radius,0);
        plot->plotxy(px,py);
        track->setPath({px,py});
    }
}

void MainWindow::local_coor(double x, double y, double yaw){
    char str[100];
    sprintf(str, "x:%0.3lf, y:%0.3lf, yaw:%0.3lf", x, y, yaw*180/M_PI);
    ui.label_6->setText(tr(str));
    posr={x,y,yaw};
    auto p = getXYT(posr);
    sprintf(str, "x:%0.3lf, y:%0.3lf, yaw:%0.3lf", p.x, p.y, p.t*180/M_PI);
    ui.label_12->setText(tr(str));
}

void MainWindow::set_track_path(){
    if(param_fresh){
        param_fresh=false;
        slot_set_param(ui.selected_path->currentIndex());
    }
    if(!qnode.initFlag) return; //还未连接，不能刷新

    if(!isTracking){ // 如果不是跟踪状态，启动遥控模式
        qnode.set_cmd_vel(vel.first, -vel.second);  // vel, angv
        return ;
    }
    auto p = getXYT(posr);

    if(!road.type){ // 跟踪直线路径的多个路径段
        if(st_state%2){
            qnode.set_cmd_vel(-0.5,0);
            if(p.x<fb[st_state].back())
                st_state++;
        }
        else{
            qnode.set_cmd_vel(0.5,0);
            if(p.x>fb[st_state].back())
                st_state++;
        }
        if(st_state >= fb.size()){
            isTracking=false;
            st_state=0;
            track_state=0;
            ui.track_path->setText("开始跟踪");
            ui.track_path->setChecked(false);
            ui.track_path->setStyleSheet("color:black;");
        }
        else {
            plot->plotPath(p.x,st_state);    //
            plot->plotCar(p.x, st_state, 0);
        }
    }
    else if(road.type==1){
        plot->plotPath(p.x,p.y);    //
        plot->plotCar(p.x, p.y, p.t);
        auto [vx, va] = track->track_path({p.x, p.y, p.t});
        qnode.set_cmd_vel(vx, va);  // vel, angv

    } else if(road.type==2){
        plot->plotPath(p.x,p.y);    //
        plot->plotCar(p.x, p.y, p.t);
        if( st_state==0){
            qnode.set_cmd_vel(0,0.2);   //
            if(p.t >= M_PI/2)
                st_state++;
        } else if( st_state==1){
            auto [vx, va] = track->track_path({p.x, p.y, p.t});
            qnode.set_cmd_vel(vx, va);  // vel, angv
        }
    }

}

void MainWindow::slot_set_param(int index){
    qDebug() << "selected path type " << index << endl;
    auto layout = ui.param_widget->layout();    
    for(auto var: pathWidgets){
        qDebug() << "delete " <<  var->objectName()<< endl;
        layout->removeWidget(var);
        var->deleteLater();
        update();
    }
    while(!pathWidgets.empty()) pathWidgets.pop_back();
    if(!index){
        ui.label_cur->setText("路径长度/m");
        auto len_spinbox = set_spinbox<QDoubleSpinBox, double>(&road.st_road.len);
        auto label_st = new QLabel(ui.param_widget);
        label_st->setText("路径组合数");
        label_st->setObjectName("label_st");
        auto spin_st = set_spinbox<QSpinBox, int>(&road.st_road.num);
        layout->addWidget(len_spinbox);
        layout->addWidget(label_st);
        layout->addWidget(spin_st);
        pathWidgets.push_back(len_spinbox);
        pathWidgets.push_back(label_st);
        pathWidgets.push_back(spin_st);

        auto hlayout = new QHBoxLayout;//ui.widget_com->layout();
        auto blayout = new QHBoxLayout;
        if(road.st_road.combin.isEmpty()){
            QVector<QPair<double, double>> tmp(road.st_road.num);
            road.st_road.combin=tmp;
        }
        for (int i = 0; i < road.st_road.num; ++i) {
            if(road.st_road.combin.size()<= i)
                road.st_road.combin.append({0,0});
            auto label = new QLabel;
            label->setText("F"+QString::number(i));
            label->setMinimumWidth(30);
            auto spin_st = set_spinbox<QDoubleSpinBox, double>( &road.st_road.combin[i].first);
            hlayout->addWidget(label);
            hlayout->addWidget(spin_st);

            auto label2 = new QLabel;
            label2->setText("B"+QString::number(i));
            label2->setMinimumWidth(30);
            auto spin_st2 = set_spinbox<QDoubleSpinBox, double>(&road.st_road.combin[i].second);
            blayout->addWidget(label2);
            blayout->addWidget(spin_st2);

            pathWidgets.push_back(label);
            pathWidgets.push_back(label2);
            pathWidgets.push_back(spin_st);
            pathWidgets.push_back(spin_st2);
        }
        hlayout->addSpacerItem(new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));
        hlayout->setContentsMargins({9,1,9,1}); // left,  top,  right,  bottom
        blayout->setContentsMargins({9,1,9,1}); // left,  top,  right,  bottom
        blayout->addSpacerItem(new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));
        auto ly= ui.vertical_road;
        auto w=new QWidget;
        w->setLayout(hlayout);
        ly->addWidget(w);
        auto w2=new QWidget;
        w2->setLayout(blayout);
        ly->addWidget(w2);
        pathWidgets.push_back(w);
        pathWidgets.push_back(w2);
    }
    else if(index == 1){
        ui.label_cur->setText("路径长度/m");
        auto len_spinbox = set_spinbox<QDoubleSpinBox, double>(&road.c_road.len);
        auto spinbox_cur = set_spinbox<QDoubleSpinBox, double>(&road.c_road.cur);
        spinbox_cur->setDecimals(4);    // 小数点
        spinbox_cur->setSingleStep(0.0001);
        spinbox_cur->setValue(road.c_road.cur);
        auto label_cur = new QLabel(ui.param_widget);
        label_cur->setText("路径曲率/rad");
        label_cur->setObjectName("label_cur");
        layout->addWidget(len_spinbox);
        layout->addWidget(label_cur);
        layout->addWidget(spinbox_cur);
        pathWidgets.push_back(len_spinbox);
        pathWidgets.push_back(label_cur);
        pathWidgets.push_back(spinbox_cur);
    }
    else if(index==2){
        ui.label_cur->setText("半径/m");
        auto len_spinbox = set_spinbox<QDoubleSpinBox,double>(&road.radius);
        pathWidgets.push_back(len_spinbox);
        layout->addWidget(len_spinbox);
    }
    layout->removeItem(spacer);
    spacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout->addItem(spacer);
    road.type=index;
    track->reset(); //
    plot->plotCar(0,0,0);
    road_value_set();
}

template <class T1,class T2>
T1* MainWindow::set_spinbox( T2 *value){
    auto len_spinbox = new T1;
    len_spinbox->setRange(0,1000);
    len_spinbox->setValue(*value);
    len_spinbox->setKeyboardTracking(false); // 全部输入再响应valueChanged
    // 当信号有重载时，需要用QOverload<double>::of来处理，否则无法使用lambda函数
//    qDebug() << "value change : " << *value<< endl;
    connect(len_spinbox,QOverload<T2>::of(&T1::valueChanged),[this,value](T2 val){
        qDebug()  << "set value " << val;
        if(*value!=val){
            *value = val;
            param_fresh=true;
        }
    });
    return len_spinbox;
}

void MainWindow::changeComboex(QComboBox *combox, const QString& messagetype){
    QString selected= combox->currentText();
    combox->clear();
    QList<QString> topics;
    if(messagetype=="images")   // 如果是图像，需要特殊处理
        topics = qnode.getImgTopiclist();
    else
        topics = qnode.getTopics(messagetype);//

    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
        QString label(*it);
        if(topics.size()>1 && label==""){
          continue;
        }
      if(selected==""){
          selected = label;
      }
      label.replace(" ", "/");
      if(label.contains("UGV"+QString::number(qnode.car)))
          selected=label;
      combox->addItem(label, QVariant(*it));
    }
    // restore previous selection
    int index = combox->findText(selected);
  //  qDebug() << "combox_index:" <<index << endl;
    if (index == -1)
    {
      // add topic name to list if not yet in
      QString label(selected);
      label.replace(" ", "/");
      combox->addItem(label, QVariant(selected));
      index = combox->findText(selected);
    }
    combox->setCurrentIndex(index);
}

void MainWindow::updateMapTopicList(){
    //    qDebug() << "refresh image message" << endl;
    if(!qnode.initFlag) return; //还未连接，不能刷新
    changeComboex(ui.topics_map,"nav_msgs/OccupancyGrid");
    changeComboex(ui.topics_laser,"sensor_msgs/LaserScan");
    rpanel->robotModelDisplySlot();
}

void MainWindow::updateImgTopicList()
{
    if(!qnode.initFlag) return; //还未连接，不能刷新
    changeComboex(ui.topics_img,"images");
}

void MainWindow::connectSuccess(int car){
    ui.label_link->setPixmap(
        QPixmap::fromImage(QImage("://images/online1.png")));
    ui.label_state->setStyleSheet("color:green;");
    ui.label_state->setText("在线");
//    qDebug() << "conncet car:" << car << " success!" << endl;
    if( car == 1){
        qnode.set_cmd_vel(0,0);
        ui.car1_connect->setChecked(false);
        ui.car1_connect->setStyleSheet("color:black;");
        ui.car0_connect->setStyleSheet("color:blue;");
    }else {
        qnode.set_cmd_vel(0,0);
        ui.car0_connect->setChecked(false);
        ui.car0_connect->setStyleSheet("color:black;");
        ui.car1_connect->setStyleSheet("color:blue;");
    }
    rpanel->initPanelSlot();
    updateMapTopicList();
    ui.rviz_img->initPanelSlot();   //
    updateImgTopicList();
    connectState = car;   //
    qnode.car=car-1;
    if( isTracking ) slot_stopTrack();   //
}

void MainWindow::connectFailed(int car){
    showNoMasterMessage();
    ui.label_link->setPixmap(
        QPixmap::fromImage(QImage("://images/offline1.png")));
    ui.label_state->setStyleSheet("color:red;");
    ui.label_state->setText("离线");
//    qDebug() << "conncet car:" << car << " failed!" << endl;
}

bool MainWindow::connectMaster(QString master_ip, QString ros_ip, int car) {
    if ( ! qnode.init(master_ip.toStdString(),
               ros_ip.toStdString(), car) ) {   // 如果roscore没打开，通过执行命令打开
        return false;
    }
    else
        connectSuccess(car);
    return true;
}

void MainWindow::slot_car_connect(int car){
    if(connectState==car) return ;    // has connect
    if( car == 1){        
        connectMaster(car0_qMasterIp, car0_qRosIp, 1);
    }
    else{        
        connectMaster(car1_qMasterIp, car1_qRosIp, 2);
    }
}

void MainWindow::slot_set_start_pose()
{
    rpanel->Set_Start_Pose();
}

bool trg=false;
void MainWindow::slot_update_power(float value)
{
    ui.label_voltage->setText(QString::number(value/10,'f',2)+"V");
    double n=(value-210)/(240-210);
    int val=n*100;
    val = val>100?100:val;
    ui.progressBar->setValue(val);
    if(val <= 10 && trg){
        trg=false;
         QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！", QMessageBox::Yes , QMessageBox::Yes);
    }
    //当电量过低时发出提示
    if (val <= 20 && (volState == 0||volState == 2)) {
        volState = 1;
      ui.progressBar->setStyleSheet(
          "QProgressBar::chunk {background-color: red;width: 20px;} "
          "QProgressBar {background: rgb(211, 215, 207);border: none;border-radius: 5px;text-align: center;}");
    } else if( val > 20 && (volState == 0||volState == 1)){
        volState = 2;trg=true;
      ui.progressBar->setStyleSheet(
            "QProgressBar{border-radius:5px;text-align:center;background: rgb(211, 215, 207); border: none; color: rgb(229, 229, 229); }"
            "QProgressBar:chunk{ background-color:rgb(115, 210, 22); }");
    }

}

void MainWindow::slot_update_dashboard(float x,float y)
{
    ui.label_vel->setText(QString::number(x,'f',2));
    ui.label_rad->setText(QString::number(y,'f',2));
}

MainWindow::~MainWindow() {
    if(qnode.initFlag){ // 防止关闭界面时 控制命令还在执行
        qnode.set_cmd_vel(0,0);
    }
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    qRegisterMetaTypeStreamOperators<road_data>("road_data");//注册结构体的操作流,就是重写的输入输出流
    qRegisterMetaType<road_data>("road_data");//注册结构体
    QSettings settings("Qt-Ros Package", "robot_hmi");
    img_filepath = settings.value("img_filepath").toString();
    QVariant v = settings.value("road");
    road = v.value<road_data>();   // struct
}

void MainWindow::WriteSettings() {

    QSettings settings("Qt-Ros Package", "robot_hmi");
    settings.setValue("img_filepath",img_filepath);
    settings.setValue("road",QVariant::fromValue(road));   // struct
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_hmi

