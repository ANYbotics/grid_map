/*
 * message_filter_display.h
 *
 *  Created on: Jan 24, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich
 */

#pragma once

#ifndef Q_MOC_RUN
#include "grid_map_rviz_plugin/modified/frame_manager.h"
#include <rviz/message_filter_display.h>
#endif

namespace grid_map_rviz_plugin {

template<class MessageType>
class MessageFilterDisplayMod : public rviz::MessageFilterDisplay<MessageType>
{
 public:
  typedef MessageFilterDisplayMod<MessageType> MFDClass;

  void onInitialize()
  {
    MFDClass::tf_filter_ = new tf2_ros::MessageFilter<MessageType>(*MFDClass::context_->getTF2BufferPtr(),
                                                                   MFDClass::fixed_frame_.toStdString(),
                                                                   10, MFDClass::update_nh_);

    MFDClass::tf_filter_->connectInput(MFDClass::sub_);
    MFDClass::tf_filter_->registerCallback(
        boost::bind(&MFDClass::incomingMessage, this, _1));
    MFDClass::context_->getFrameManager()->registerFilterForTransformStatusCheck(
        MFDClass::tf_filter_, this);
  }

};

} // end namespace
