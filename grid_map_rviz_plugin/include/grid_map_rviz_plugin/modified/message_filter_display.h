/*
 * message_filter_display.h
 *
 *  Created on: Jan 24, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich
 */

#ifndef GRID_MAP_RVIZ_PLUGIN__MODIFIED__MESSAGE_FILTER_DISPLAY_H_
#define GRID_MAP_RVIZ_PLUGIN__MODIFIED__MESSAGE_FILTER_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include "grid_map_rviz_plugin/modified/frame_manager.h"
#endif

namespace grid_map_rviz_plugin {

template < class MessageType >
class MessageFilterDisplayMod: public rviz::MessageFilterDisplay < MessageType >
{
public:
    typedef MessageFilterDisplayMod < MessageType > MFDClass;

    void onInitialize()
    {
  #if ROS_VERSION_MINIMUM(1, 14, 0)
      MFDClass::tf_filter_ = new tf2_ros::MessageFilter < MessageType >
        (*MFDClass::context_->getTF2BufferPtr(),
        MFDClass::fixed_frame_.toStdString(),
        10, MFDClass::update_nh_);
  #else
      MFDClass::tf_filter_ = new tf::MessageFilter < MessageType >
        (*MFDClass::context_->getTFClient(),
        MFDClass::fixed_frame_.toStdString(),
        10, MFDClass::update_nh_);
  #endif

      MFDClass::tf_filter_->connectInput(MFDClass::sub_);
      MFDClass::tf_filter_->registerCallback(
        boost::bind(&MFDClass::incomingMessage, this, _1));
      MFDClass::context_->getFrameManager()->registerFilterForTransformStatusCheck(
        MFDClass::tf_filter_, this);
    }
  }

}  // namespace grid_map_rviz_plugin
#endif  // GRID_MAP_RVIZ_PLUGIN__MODIFIED__MESSAGE_FILTER_DISPLAY_H_
