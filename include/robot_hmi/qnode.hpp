/**
 * @file /include/robot_hmi/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_hmi_QNODE_HPP_
#define robot_hmi_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <map>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <QImage>
#include <QProcess>

#include "../robot_msg/four1.h"
#include "GPS_ksxt.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
    bool init(const std::string &master_url, const std::string &host_url, int car);
    void init_set();
    QList<QString> getTopics(const QString& message_types);    //
    QSet<QString> getTopics(const QSet<QString>& message_types,
                                   const QSet<QString>& message_sub_types, const QList<QString>& transports);
    void set_cmd_vel(float linear,float angular);
    void sub_image(QString topic_name);
    void set_goal(double x,double y,double z);
	void run();
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    QList<QString> getImgTopiclist();   //

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void speed_vel(float,float);
    void power_vel(float);
    void position(double x,double y,double z);
    void connectMasterSuccess(int car);
    void connectMasterFailed(int car);
    void gps_pos(int posqual, int headingqual, double x, double y);

public slots:
    void cmd_output(int car);
    void cmd_error_output(int car);

private:
	int init_argc;
    char** init_argv;
    ros::Publisher cmd_vel_pub[2];
    ros::Publisher goal_pub[2];
    QStringListModel logging_model;
    ros::Subscriber odom_sub[2];
    ros::Subscriber power_sub[2];
    ros::Subscriber gps_sub[2];
//    ros::Subscriber odom_sub;   // xiang
    image_transport::Subscriber image_sub;
//    image_transport::ImageTransport *it;
    QProcess *roscore = NULL;   // run roscore cmd

    void power_callback(const ros_four1_msg::four1ConstPtr &msg, int car);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg, int car);
    void gps_callback(const gps_ksxt_msg::GPS_ksxtConstPtr &msg, int car);


public:
    bool initFlag=false;    // 判断是否连接ros master成功
    uint car=0;     //
    int car0=0, car1=1;

};

}  // namespace robot_hmi

#endif /* robot_hmi_QNODE_HPP_ */
