/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/network.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <sstream>
#include "QDebug"
#include "QProcess" // deal cmd
#include "../include/robot_hmi/qnode.hpp"
#include "../include/robot_msg/four1.h"
#include "GPS_ksxt.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robot_hmi");
	if ( ! ros::master::check() ) {
        initFlag = false;
		return false;
	}
    if(!initFlag)
        init_set();
	return true;
}

QList<QString> QNode::getTopics(const QString& message_types){
    ros::master::V_TopicInfo topic_info;  // 获取所有的ros话题
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
      all_topics.insert(it->name.c_str());
    }
    QList<QString> topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
      if (message_types.contains(it->datatype.c_str()))   // 如果话题类型相同
      {
          QString topic = it->name.c_str();
          topics.append(topic);
      }
    }
    return topics;
}


QSet<QString> QNode::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;  // 获取所有的ros话题
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))   // 如果话题类型相同
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.insert(topic);
      //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
    if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");   // 找到"/"最后出现的位置，找不到说明不是消息类型
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void QNode::cmd_output(int car){
    if ( ! ros::master::check() ) { // if still connect ROS master failed
        qDebug() << "connect again and failed!" << endl;
        emit connectMasterFailed(car);
        initFlag = false;
        return ;
    }
    if(!initFlag)
        init_set();
    emit connectMasterSuccess(car);    // connect ROS master success;
}

void QNode::cmd_error_output(int car){
    qDebug() << "connect failed" << endl;
    emit connectMasterFailed(car); // connect ROS master failed;
}

QList<QString> QNode::getImgTopiclist(){
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    // get declared transports
    QList<QString> transports;
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    std::vector<std::string> declared = it.getDeclaredTransports();
    for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
    {
//      qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
      QString transport = it->c_str();

      // strip prefix from transport name
      QString prefix = "image_transport/";
      if (transport.startsWith(prefix))
      {
        transport = transport.mid(prefix.length());
      }
      transports.append(transport);
    }

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    topics.append("");
    qSort(topics);
    return topics;
}

bool QNode::init(const std::string &master_url, const std::string &host_url, int car) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_hmi");
    qDebug() << "test\n";
	if ( ! ros::master::check() ) {
        roscore = new QProcess; // run roscore
        roscore->start("bash");
        QString bash = "roscore\n";
        qDebug()  << "bash" << endl;
        roscore->write(bash.toLocal8Bit());
//        connect(roscore,&QProcess::readyReadStandardOutput,this,[=]{cmd_output(car);});
        roscore->waitForFinished(1500); // 等待1s
//        wait(5000);  // wait roscore run success
        cmd_output(car);
        return false;
	}
    if(!initFlag)
        init_set();
    emit connectMasterSuccess(car);    // connect ROS master success;
    return true;
}

void QNode::init_set(){
    initFlag = true;
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    std::string ugv[2] ={"/UGV0", "/UGV1"};
    cmd_vel_pub[0]=n.advertise<geometry_msgs::Twist>(ugv[0]+"/cmd_vel",1000);
    cmd_vel_pub[1]=n.advertise<geometry_msgs::Twist>(ugv[1]+"/cmd_vel",1000);
    goal_pub[0]=n.advertise<geometry_msgs::PoseStamped>(ugv[0]+"/goal",1000);
    goal_pub[1]=n.advertise<geometry_msgs::PoseStamped>(ugv[0]+"/goal",1000);

    // 利用boost::bind 函数 绑定 额外参数
    power_sub[0] = n.subscribe<ros_four1_msg::four1>(ugv[0]+"four_info",1000,boost::bind(&QNode::power_callback,this, _1, 0));
    power_sub[1] = n.subscribe<ros_four1_msg::four1>(ugv[1]+"four_info",1000,boost::bind(&QNode::power_callback,this,_1, 1));
    odom_sub[0] = n.subscribe<nav_msgs::Odometry>(ugv[0]+"/odom",1000,boost::bind(&QNode::odom_callback,this,_1,0));
    odom_sub[1] = n.subscribe<nav_msgs::Odometry>(ugv[1]+"/odom",1000,boost::bind(&QNode::odom_callback,this,_1,1));
    gps_sub[0] = n.subscribe<gps_ksxt_msg::GPS_ksxt>(ugv[0]+"gps",1000, boost::bind(&QNode::gps_callback,this,_1,0));
    gps_sub[1] = n.subscribe<gps_ksxt_msg::GPS_ksxt>(ugv[1]+"gps",1000, boost::bind(&QNode::gps_callback,this,_1,1));

    start();

}


void QNode::set_goal(double x,double y,double z)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id="map";
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.orientation.z=z;
    goal_pub[car].publish(goal);
}


void QNode::gps_callback(const gps_ksxt_msg::GPS_ksxtConstPtr &msg, int car){

    if(car!=this->car){
        return ;
    }
//    qDebug() << "gps car is " << car << endl;
    emit gps_pos(msg->posqual, msg->headingqual,msg->east, msg->north);
}


void QNode::power_callback(const ros_four1_msg::four1ConstPtr &msg, int car)
{
    if(car!=this->car){
        return ;
    }
//    qDebug() << "gps car is " << car << endl;
    double base_width=470;  // mm 轴距
    double vel = (msg->FRSpeed+msg->FLSpeed)/2000;  // mm/s -> m/s
    double angv = 2*(msg->FRSpeed-msg->FLSpeed)/base_width;
    emit speed_vel(vel,angv);
    emit power_vel(msg->Voltage);
}
void QNode::odom_callback(const nav_msgs::Odometry::ConstPtr &msg, int car)
{
//    qDebug() << "odom car is " << *car << endl;
    if(car!=this->car){
        return ;
    }
//    qDebug() << "gps car is " << car << endl;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
          tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    emit position(msg->pose.pose.position.x,msg->pose.pose.position.y, yaw);
}

void QNode::set_cmd_vel(float linear,float angular)
{
//    qDebug() << "control vel:" << linear << angular << endl;
    geometry_msgs::Twist twist;
    twist.linear.x=1*linear;
    twist.linear.y=1*linear;
    twist.linear.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=angular;
    cmd_vel_pub[car].publish(twist);
}

//void QNode::chatter_callback(const std_msgs::String &msg)
//{
//    log(Info,"I recive"+msg.data);
//}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace robot_hmi
