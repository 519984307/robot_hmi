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

#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "rviz/image/ros_image_texture.h"
#include "rviz/ogre_helpers/initialization.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"

#include "ros/ros.h"
#include <ros/package.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTextureUnitState.h>
#include <OGRE/OgreViewport.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#endif

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace rviz;

class ImageView : public QtOgreRenderWindow {
    Q_OBJECT
public:
    ImageView(QWidget* parent, QString topic);
    ~ImageView() override;
    void setTopic(QString topic); //更改Topic TODO

protected:
    void showEvent(QShowEvent* event) override;

public:
    const sensor_msgs::Image::ConstPtr& getImage(){return texture_->getImage();};

private Q_SLOTS:
    void onTimer();

private:
    void textureCallback(const sensor_msgs::Image::ConstPtr& msg);

    Ogre::SceneManager* scene_manager_;
    //    Ogre::Camera* camera_;
    Ogre::SceneNode* node_;
    Ogre::Rectangle2D* screen_rect_; // make sure the aspect ratio of the image is preserved
    ROSImageTexture* texture_;

    ros::NodeHandle nh_;

    image_transport::ImageTransport texture_it_;
    boost::shared_ptr<image_transport::SubscriberFilter> texture_sub_;

    QString topic; //保存传入的topic
    QTimer* timer;
};

#endif // IMAGEVIEW_H
