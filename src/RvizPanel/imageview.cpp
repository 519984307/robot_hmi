/*
  * Copyright (c) 2008, Willow Garage, Inc.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  *     * Redistributions of source code must retain the above copyright
  *       notice, this list of conditions and the following disclaimer.
  *     * Redistributions in binary form must reproduce the above copyright
  *       notice, this list of conditions and the following disclaimer in the
  *       documentation and/or other materials provided with the distribution.
  *     * Neither the name of the Willow Garage, Inc. nor the names of its
  *       contributors may be used to endorse or promote products derived from
  *       this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  */

#include <QTimer>
#include <QtGlobal>

#include "rviz/image/ros_image_texture.h"
#include "rviz/ogre_helpers/initialization.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"

#include "ros/ros.h"
#include <ros/package.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureUnitState.h>
#include <OGRE/OgreViewport.h>

#include "./include/RvizPanel/imageview.h"
#include <QtDebug>
#include <boost/algorithm/string.hpp>

using namespace rviz;

ImageView::ImageView(QWidget* parent, QString topic)
    : QtOgreRenderWindow(parent)
    , texture_it_(nh_)
    , topic(topic)
{

    setAutoRender(false);
    scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC, "TestSceneManager");

    setCamera(scene_manager_->createCamera("Camera"));

    texture_ = new ROSImageTexture();

    try {
        bool compressed = false;
        if (boost::ends_with(topic, "/compressed")) {
            compressed = true;
            topic = QString::fromStdString(boost::erase_last_copy(topic.toStdString(), "/compressed"));
        }
        qDebug() << "Hints: " << (compressed ? "compressed" : "raw");
        texture_->clear();

        texture_sub_.reset(new image_transport::SubscriberFilter());
        texture_sub_->subscribe(texture_it_, topic.toStdString(), 2, image_transport::TransportHints(compressed ? "compressed" : "raw"));
        //        texture_sub_->subscribe(texture_it_, topic.toStdString(), 2, image_transport::TransportHints("compressed"));
        texture_sub_->registerCallback(boost::bind(&ImageView::textureCallback, this, _1));
    } catch (ros::Exception& e) {
        ROS_ERROR("%s", (std::string("Error subscribing: ") + e.what()).c_str());
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setCullingMode(Ogre::CULL_NONE);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
    material->getTechnique(0)->setLightingEnabled(false);

    Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_->getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
    screen_rect_->setMaterial(material->getName());
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    Ogre::AxisAlignedBox aabb;
    aabb.setInfinite();
    screen_rect_->setBoundingBox(aabb);

    node_ = scene_manager_->getRootSceneNode()->createChildSceneNode("Node");
    node_->attachObject(screen_rect_);
    node_->setVisible(true);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    timer->start(33);
}

ImageView::~ImageView()
{
    delete timer;
    node_->getParentSceneNode()->removeAndDestroyChild("Node");
    delete screen_rect_;
    //    Ogre::MaterialManager::getSingleton().destroyAllResourcePools();
    texture_sub_->unsubscribe();
    texture_sub_.reset();
    texture_->clear();
    delete texture_;
    scene_manager_->destroyCamera("Camera");
    ogre_root_->destroySceneManager(scene_manager_);
    //    getRenderWindow()->destroy();
}

//更改Topic TODO
void ImageView::setTopic(QString newtopic)
{
    if (topic != newtopic) {
        topic = newtopic;

        try {
            bool compressed = false;
            if (boost::ends_with(topic, "/compressed")) {
                compressed = true;
                topic = QString::fromStdString(boost::erase_last_copy(topic.toStdString(), "/compressed"));
            }
            qDebug() << "New Hints: " << (compressed ? "compressed" : "raw");

            texture_->clear();

            texture_sub_->unsubscribe();
            texture_sub_.reset(new image_transport::SubscriberFilter());
            texture_sub_->subscribe(texture_it_, topic.toStdString(), 2, image_transport::TransportHints(compressed ? "compressed" : "raw"));
            texture_sub_->registerCallback(boost::bind(&ImageView::textureCallback, this, _1));
        } catch (ros::Exception& e) {
            ROS_ERROR("%s", (std::string("Error subscribing: ") + e.what()).c_str());
        }
    }
}

void ImageView::showEvent(QShowEvent* event)
{
    QtOgreRenderWindow::showEvent(event);
}

void ImageView::onTimer()
{
    ros::spinOnce();
    try {
        texture_->update();

        // <make sure the aspect ratio of the image is preserved
        float win_width = width();
        float win_height = height();
        float img_width = texture_->getWidth();
        float img_height = texture_->getHeight();
        if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0) {
            float img_aspect = img_width / img_height;
            float win_aspect = win_width / win_height;
            if (img_aspect > win_aspect) {
                screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                    -1.0f * win_aspect / img_aspect, false);
            } else {
                screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect,
                    -1.0f, false);
            }
        }
        getRenderWindow()->update();
        // make sure the aspect ratio of the image is preserved>

        ogre_root_->renderOneFrame();
    } catch (UnsupportedImageEncoding& e) {
        ROS_ERROR("%s", e.what());
    }

    if (!nh_.ok()) {
        close();
    }
}

void ImageView::textureCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if (texture_) {
        texture_->addMessage(msg);
    }
}
