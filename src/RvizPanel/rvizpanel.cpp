#include <QColor>
#include <QVBoxLayout>

#include "./include/RvizPanel/rvizpanel.h"

//RvizPanel base class for pc2 & gridmap
RvizPanel::RvizPanel(QWidget* parent)
    : QWidget(parent)
    , settings("rviz_topic", "robot_hmi")
    , inited(false)
{
//    if(layout != NULL) main_layout=layout;
}

RvizPanel::~RvizPanel()
{
    qDebug("del Panel"); //TODO
    delete manager;
    delete render_panel;
}

void RvizPanel::setTopicSlot(QString topic)
{
    if (!inited)
        return;
    if (display->subProp("Topic")->getValue().toString() != topic) {
        display->subProp("Topic")->setValue(topic);
    }
}
void RvizPanel::setFixedFrameSlot(QString fixedframe)
{
    if (!inited)
        return;
    if (manager->getFixedFrame() != fixedframe) {
        manager->setFixedFrame(fixedframe);
    }
}

void RvizPanel::initPanelSlot()
{
    if (inited)
        return;

    qDebug("IN Panel"); //TODO

    // Construct and lay out render panel.
    render_panel = new rviz::RenderPanel();
//    if(main_layout==NULL)
        main_layout = new QVBoxLayout;
    main_layout->setMargin(0);
    main_layout->setSpacing(1);
    main_layout->addWidget(render_panel);

    //    setLayout(main_layout);
    this->setLayout(main_layout);

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager = new rviz::VisualizationManager(render_panel);
    render_panel->initialize(manager->getSceneManager(), manager);  //
    manager->initialize();
    manager->startUpdate();
    manager->removeAllDisplays();


    auto viewManager= manager->getViewManager();
    viewManager->setRenderPanel(render_panel);
    viewManager->setCurrentViewControllerType("rviz/Orbit");
    viewManager->getCurrent()->subProp("Target Frame")->setValue("map");
//    viewManager->setCurrentViewControllerType("rviz/FPS");
    view = manager->getViewManager()->getCurrent();
    view->reset();      // 重新回到初始位置
//    view->lookAt()    // 改变视角
    inited=true;

}

void RvizPanel::deinitPanelSlot()
{
    if (!inited)
        return;

    view = nullptr;
    delete manager;
    delete render_panel;
    delete layout();

    inited = false;
}


//void RvizPanel::Display_RobotModel(bool enable)
//{
//    if(RobotModel_!=NULL)
//    {
//        delete RobotModel_;
//        RobotModel_=NULL;
//    }
//    RobotModel_=manager_->createDisplay("rviz/RobotModel","myRobotModel",enable);
//    ROS_ASSERT(RobotModel_!=NULL);
//}


void RvizPanel::mapDisplaySlot(const QString& topic){
    if(mapDisplay!=NULL)
    {
//        auto a = mapDisplay->subProp("Topic")->getValue();
//        if(topic==a)    // topic not change
//            return;
        delete mapDisplay;
        mapDisplay=NULL;
    }
    mapDisplay=manager->createDisplay("rviz/Map","myMap",true);
    ROS_ASSERT(mapDisplay!=NULL);
    mapDisplay->subProp("Topic")->setValue(topic);

}
void RvizPanel::LaserDisplaySlot(const QString& topic){
    if(laserScanDisplay!=NULL)
    {
//        auto a = laserScanDisplay->subProp("Topic")->getValue();
//        if(topic==a)    // topic not change
//            return;
        delete laserScanDisplay;
        laserScanDisplay=NULL;
    }
    laserScanDisplay=manager->createDisplay("rviz/LaserScan","myLaser",true);
    laserScanDisplay->subProp("Topic")->setValue(topic);
    laserScanDisplay->subProp("Size (m)")->setValue(0.1);
    ROS_ASSERT(laserScanDisplay!=NULL);
}

void RvizPanel::robotModelDisplySlot(){
    if(robotModelDisplay!=NULL)
    {
        delete robotModelDisplay;
        robotModelDisplay=NULL;
    }
    robotModelDisplay=manager->createDisplay("rviz/RobotModel","myRobotModel",true);
    ROS_ASSERT(robotModelDisplay!=NULL);
}

void RvizPanel::Set_Start_Pose()
{
    rviz::ToolManager* tool_manager_=manager->getToolManager();
    rviz::Tool* current_tool_= tool_manager_->addTool("rviz/SetInitialPose");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}
void RvizPanel::Set_Goal_Pose()
{
    rviz::ToolManager* tool_manager_=manager->getToolManager();
     rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetGoal");
     //获取属性容器
     rviz::Property* pro=current_tool_->getPropertyContainer();
     //设置发布导航目标点的topic
     pro->subProp("Topic")->setValue("/move_base_simple/goal");
     //设置当前使用的工具
     tool_manager_->setCurrentTool(current_tool_);
}



//RvizLaserLidar
RvizLaserLidar::RvizLaserLidar(QWidget* parent)
    : RvizPanel(parent)
{
}

RvizLaserLidar::~RvizLaserLidar()
{
    qDebug("del PC2"); //TODO
}

void RvizLaserLidar::initPanelSlot()
{
    RvizPanel::initPanelSlot();
    if (inited)
        return;

    type_name = "PC2Lidar";

    // Create a PC2 display.
    qDebug("IN laser"); //TODO
    display = manager->createDisplay("rviz/LaserScan", "Laser", true);
    ROS_ASSERT(display != NULL);
    manager->setFixedFrame("map");
    display->subProp("Topic")->setValue("/scan");
    display->subProp("Size (m)")->setValue(0.1);

    inited = true;
}

//RvizLaserLidar
RvizMapLidar::RvizMapLidar(QWidget* parent)
    : RvizPanel(parent)
{
}

RvizMapLidar::~RvizMapLidar()
{
    qDebug("del PC2"); //TODO
}

void RvizMapLidar::initPanelSlot()
{
    RvizPanel::initPanelSlot();
    if (inited)
        return;

    type_name = "PC2Lidar";

    // Create a PC2 display.
    qDebug("IN map"); //TODO
    display = manager->createDisplay("rviz/Map", "mymap", true);
    ROS_ASSERT(display != NULL);
    manager->setFixedFrame("map");
    display->subProp("Topic")->setValue("/map");
//    display->subProp("Size (m)")->setValue(0.1);

    inited = true;
}

//RvizPC2Lidar
RvizPC2Lidar::RvizPC2Lidar(QWidget* parent)
    : RvizPanel(parent)
{
}

RvizPC2Lidar::~RvizPC2Lidar()
{
    qDebug("del PC2"); //TODO
}

void RvizPC2Lidar::initPanelSlot()
{
    RvizPanel::initPanelSlot();
    if (inited)
        return;

    type_name = "PC2Lidar";

    // Create a PC2 display.
    qDebug("IN PC2"); //TODO
    display = manager->createDisplay("rviz/PointCloud2", "PC2Lidar", true);
    ROS_ASSERT(display != NULL);

    // Configure the PC2Display the way we like it.
    //    manager->setFixedFrame("vehicle_frame");
    manager->setFixedFrame(settings.value("rviz/PC2Lidar/fixedframe").toString());
    //    pc2->subProp("Topic")->setValue("/lidar_cloud_calibrated");
    display->subProp("Topic")->setValue(settings.value("rviz/PC2Lidar/topic"));
    display->subProp("Size (m)")->setValue(0.1);

    inited = true;

    //    RvizPanel::initPanelSlot();
    //    if (inited)
    //        return;

    //    type_name = "PC2Lidar";

    //    // Create a image display.
    //    qDebug("IN image"); //TODO
    //    display = manager->createDisplay("my_image_display/Image", "Image", true);
    //    ROS_ASSERT(display != NULL);

    //    // Configure the PC2Display the way we like it.
    //    display->subProp("Image Topic")->setValue("/car/image");
    //    display->subProp("Transport Hint")->setValue("compressed");

    //    inited = true;
}

//RvizPC2Colored
RvizPC2Colored::RvizPC2Colored(QWidget* parent)
    : RvizPanel(parent)
{
}

RvizPC2Colored::~RvizPC2Colored()
{
    qDebug("del PC2"); //TODO
}

void RvizPC2Colored::initPanelSlot()
{
    RvizPanel::initPanelSlot();
    if (inited)
        return;

    type_name = "PC2Colored";

    // Create a PC2 display.
    qDebug("IN PC2"); //TODO
    display = manager->createDisplay("rviz/PointCloud2", "PC2Colored", true);
    ROS_ASSERT(display != NULL);

    // Configure the PC2Display the way we like it.
    //    manager->setFixedFrame("vehicle_frame");
    manager->setFixedFrame(settings.value("rviz/PC2Colored/fixedframe").toString());
    //    display->subProp("Topic")->setValue("/colored_cloud_toshow");
    display->subProp("Topic")->setValue(settings.value("rviz/PC2Colored/topic"));
    display->subProp("Size (m)")->setValue(0.1);

    inited = true;
}

//RvizGridMap
RvizGridMap::RvizGridMap(QWidget* parent)
    : RvizPanel(parent)
{
}

RvizGridMap::~RvizGridMap()
{
    qDebug("del GM"); //TODO
}

void RvizGridMap::initPanelSlot()
{
    RvizPanel::initPanelSlot();
    if (inited)
        return;

    type_name = "GridMap";

    qDebug("IN GM"); //TODO
    //    gridmap = manager->createDisplay("grid_map_rviz_plugin/GridMap", "GridMap", true);
    display = manager->createDisplay("grid_map_rviz_plugin/GridMap", "GridMap", true);
    ROS_ASSERT(display != NULL);

    //    manager->setFixedFrame("odom");
    manager->setFixedFrame(settings.value("rviz/GridMap/fixedframe").toString());
    view->subProp("Target Frame")->setValue("vehicle_frame");
    //    manager->getViewManager()->getCurrent()->setProperty("Target Frame", "vehicle_frame");

    //    display->subProp("Topic")->setValue("/elevation_mapping/elevation_map");
    display->subProp("Topic")->setValue(settings.value("rviz/GridMap/topic"));
    display->subProp("History Length")->setValue(100);

    inited = true;
}

//ImageView class for camera
RvizImage::RvizImage(QWidget* parent)
    : QWidget(parent)
    , settings("rviz_topic", "cyrobot_monitor")
    , inited(false)
{
}

RvizImage::~RvizImage()
{
    qDebug("del Cam"); //TODO
    //    delete imageview;
}

void RvizImage::setTopicSlot(QString newtopic)
{
    if (!inited)
        return;
    imageview->setTopic(newtopic);
    //    settings.setValue("rviz/Image/topic", newtopic);
}

const sensor_msgs::Image::ConstPtr& RvizImage::getImage(){
//    cv_bridge::CvImagePtr cv_ptr;
    return imageview->getImage();
}

void RvizImage::initPanelSlot()
{
    if (inited)
        return;

    qDebug("IN Cam"); //TODO

    topic = settings.value("rviz/Image/topic").toString();
    //    imageview = new ImageView(0, "/rgb_front/image_raw");
    imageview = new ImageView(0, topic);

    QVBoxLayout* main_layout = new QVBoxLayout();

    main_layout->setMargin(0);
    main_layout->setSpacing(1);

    main_layout->addWidget(imageview);
    imageview->setEnabled(true);

    //    setLayout(main_layout);
    this->setLayout(main_layout);
    inited = true;
}

void RvizImage::deinitPanelSlot()
{
    if (!inited)
        return;
    delete imageview;
    delete layout();

    inited = false;
}
