#ifndef RVIZPANEL_H
#define RVIZPANEL_H

#include "imageview.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/default_plugin/map_display.h"
#include "rviz/default_plugin/laser_scan_display.h"
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QSettings>
#include <QVBoxLayout>

namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
class ViewController;
} // namespace rviz

class RvizPanel : public QWidget {
    Q_OBJECT

public:
    RvizPanel(QWidget* parent = 0/*, QVBoxLayout *layout=NULL*/);
//    RvizPanel(QVBoxLayout *layout=NULL);
    virtual ~RvizPanel();
public slots:
    void setTopicSlot(QString topic);
    void setFixedFrameSlot(QString fixedframe);
    void initPanelSlot();
    void deinitPanelSlot();

    void mapDisplaySlot(const QString& topic);
    void LaserDisplaySlot(const QString& topic);
    void robotModelDisplySlot();
    void Set_Start_Pose();
    void Set_Goal_Pose();

protected:
   QVBoxLayout* main_layout;

    rviz::VisualizationManager* manager;
    rviz::RenderPanel* render_panel;
    rviz::Display* display;
    rviz::ViewController* view;
    rviz::Display *mapDisplay=NULL;
    rviz::Display *laserScanDisplay=NULL;
    rviz::Display* robotModelDisplay=NULL;

    QSettings settings;
    QString type_name;
    bool inited;
};

//
class RvizLaserLidar : public RvizPanel {
    Q_OBJECT

public:
    RvizLaserLidar(QWidget* parent);
    virtual ~RvizLaserLidar();
public slots:
    void initPanelSlot();
};


//
class RvizMapLidar : public RvizPanel {
    Q_OBJECT

public:
    RvizMapLidar(QWidget* parent);
    virtual ~RvizMapLidar();
public slots:
    void initPanelSlot();
};



//
class RvizPC2Lidar : public RvizPanel {
    Q_OBJECT

public:
    RvizPC2Lidar(QWidget* parent);
    virtual ~RvizPC2Lidar();
public slots:
    void initPanelSlot();
};


//
class RvizPC2Colored : public RvizPanel {
    Q_OBJECT

public:
    RvizPC2Colored(QWidget* parent);
    virtual ~RvizPC2Colored();
public slots:
    void initPanelSlot();
};


//
class RvizGridMap : public RvizPanel {
    Q_OBJECT

public:
    RvizGridMap(QWidget* parent);
    virtual ~RvizGridMap();
public slots:
    void initPanelSlot();
};

//
class RvizImage : public QWidget {
    Q_OBJECT

public:
    RvizImage(QWidget* parent);
    virtual ~RvizImage();
public slots:
    void setTopicSlot(QString topic);
    void initPanelSlot();
    void deinitPanelSlot();

private:
    //    QVBoxLayout* main_layout;

    ImageView* imageview;
    QString topic;
    QSettings settings;
    bool inited;

public:
    const sensor_msgs::Image::ConstPtr& getImage();
};

#endif // RVIZPANEL_H
