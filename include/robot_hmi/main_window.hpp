/**
 * @file /include/robot_hmi/main_window.hpp
 *
 * @brief Qt based gui for robot_hmi.
 *
 * @date November 2010
 **/
#ifndef robot_hmi_MAIN_WINDOW_H
#define robot_hmi_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "joystick.h"
#include <QImage>
#include <QProcess>
#include <QComboBox>
#include <QQueue>
#include <QSpinBox>
#include "../RvizPanel/rvizpanel.h"
#include "common.hpp"
#include "purepusuit.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_hmi {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    void ui_init();
    void connect_init();
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void changeComboex(QComboBox *combox, const QString& messagetype);
//    void selectTopic(const QString& topic);  // save preview
    bool connectMaster(QString master_ip, QString ros_ip, int car);

    template <class T1,class T2>
    T1* set_spinbox(T2 *value);
    void setShadowEffect(QWidget * w);


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
    *******************************************/

    void connectSuccess(int car); // successful connect ROS master
    void connectFailed(int car); // successful connect ROS master

    /******************************************
    ** Manual connections
    *******************************************/
    void slot_update_dashboard(float,float);    // 更新速度显示
    void slot_update_power(float);          // 更新电池电量
    void slot_set_start_pose();
    void slot_car_connect(int car); // select car
//    void slot_indor_connect();  //
//    void slot_outdor_connect(); //

    void updateImgTopicList();  // refresh img topic list
    void updateMapTopicList();  //
    void onTopicChanged(int index);   // show image
    void onTopicMapChanged(int index);   // show image
    void onTopicLaserChanged(int index);   // show image
    void saveImage();       // s

    void slot_set_param(int index); //
    void road_value_set();   //

    void timerSlot();       // save image timer;
    void startTimer();
    void stopTimer();

    void slot_gps(int posqual, int headingqual, double x, double y);
    void set_track_path();
    void local_coor(double x, double y, double yaw);
    void slot_stopTrack();

    state<double> getXYT(state<double> xyt);

private:
	Ui::MainWindowDesign ui;
    QNode qnode;
    PurePusuit* track;  // pure pursuit algorithm

    JoyStick *vel_joy;  //left vel control
    JoyStick *omg_joy;  // right turn control
    QTimer *cmd_time;    // publish cmd
    QPair<float, float> vel;

    QProcess *laser_cmd;
    RvizMapLidar *rmap;
    RvizPanel   *rpanel;    

    //
    QVector<QWidget *> pathWidgets;    // 记录路径的参数组件
    QSpacerItem* spacer; //
    road_data road;  // 路径
    QVector<QVector<double>> fb;        // 记录直线路径的多个路径段
    int st_state=0;                     // 记录跟踪直线路径的状态，跟踪到哪一段了
    PlotRoad *plot;
    state<double> pos0;   // (x,y,theta) 记录跟踪路径的第一个点作为初始坐标系
    state<double> posr;   // 实时的坐标
    bool isTracking=false;  //
    uint track_state=0;     // 0 start 1 pause
    bool param_fresh=false; // 是否需要刷新数据



    QString car0_qRosIp;
    QString car0_qMasterIp;
    QString car1_qRosIp;
    QString car1_qMasterIp;

    QString img_filepath;

    int connectState = 0; // 0 is not connect, 1 is connect car0, 2 is connect car1;
    int volState = 0;   // 1 is low battery, 2 is enough battery;
    int img_state=0;   // 判断截图是否开始 1 is start, 2 is end
    QVector<QImage> img_vec;   // 批量保存图片
    QTimer *timer;          // record save image timer


};

}  // namespace robot_hmi


#endif // robot_hmi_MAIN_WINDOW_H
