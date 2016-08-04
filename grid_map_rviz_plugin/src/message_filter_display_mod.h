/*
* Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef MESSAGE_FILTER_DISPLAY_MOD_H
#define MESSAGE_FILTER_DISPLAY_MOD_H

#ifndef Q_MOC_RUN
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <message_filters/subscriber.h>
#include "message_filter_mod.h"
#endif

#include "rviz/display_context.h"
#include "frame_manager_mod.h"
#include "rviz/properties/ros_topic_property.h"

#include "rviz/display.h"

namespace grid_map_rviz_plugin
{

  class _RosTopicDisplay: public rviz::Display
  {
    Q_OBJECT
    public:
      _RosTopicDisplay()
	{
	  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
						  "", "",
						  this, SLOT( updateTopic() ));
	  unreliable_property_ = new rviz::BoolProperty( "Unreliable", false,
						  "Prefer UDP topic transport",
						  this,
						  SLOT( updateTopic() ));
	}

    protected Q_SLOTS:
      virtual void updateTopic() = 0;

    protected:
      rviz::RosTopicProperty* topic_property_;
      rviz::BoolProperty* unreliable_property_;
  };

  template<class MessageType, class MessageInfoType>
  class MessageFilterDisplayMod: public _RosTopicDisplay
  {
    // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.
    public:
      typedef MessageFilterDisplayMod<MessageType, MessageInfoType> MFDClass;

      MessageFilterDisplayMod()
	: tf_filter_( NULL )
	, messages_received_( 0 )
	{
	  QString message_type = QString::fromStdString( ros::message_traits::datatype<MessageType>() );
	  topic_property_->setMessageType( message_type );
	  topic_property_->setDescription( message_type + " topic to subscribe to." );
	}

      virtual void onInitialize()
	{
	  tf_filter_ = new tf::MessageFilter<MessageType, MessageInfoType>( *context_->getTFClient(),
							  fixed_frame_.toStdString(), 10, update_nh_ );

	  tf_filter_->connectInput( sub_ );
	  tf_filter_->registerCallback( boost::bind( &MessageFilterDisplayMod<MessageType, MessageInfoType>::incomingMessage, this, _1 ));
	  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
	}

      virtual ~MessageFilterDisplayMod()
	{
	  unsubscribe();
	  delete tf_filter_;
	}

      virtual void reset()
	{
	  Display::reset();
	  tf_filter_->clear();
	  messages_received_ = 0;
	}

      virtual void setTopic( const QString &topic, const QString &datatype )
	{
	  topic_property_->setString( topic );
	}

    protected:
      virtual void updateTopic()
	{
	  unsubscribe();
	  reset();
	  subscribe();
	  context_->queueRender();
	}

      virtual void subscribe()
	{
	  if( !isEnabled() )
	  {
	    return;
	  }

	  try
	  {
	    ros::TransportHints transport_hint = ros::TransportHints().reliable();
	    // Determine UDP vs TCP transport for user selection.
	    if (unreliable_property_->getBool())
	    {
	      transport_hint = ros::TransportHints().unreliable();
	    }
	    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 10, transport_hint);
	    setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
	  }
	  catch( ros::Exception& e )
	  {
	    setStatus( rviz::StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
	  }
	}

      virtual void unsubscribe()
	{
	  sub_.unsubscribe();
	}

      virtual void onEnable()
	{
	  subscribe();
	}

      virtual void onDisable()
	{
	  unsubscribe();
	  reset();
	}

      virtual void fixedFrameChanged()
	{
	  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
	  reset();
	}

      void incomingMessage( const typename MessageType::ConstPtr& msg )
	{
	  if( !msg )
	  {
	    return;
	  }

	  ++messages_received_;
	  setStatus( rviz::StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

	  processMessage( msg );
	}

      virtual void processMessage( const typename MessageType::ConstPtr& msg ) = 0;

      message_filters::Subscriber<MessageType> sub_;
      tf::MessageFilter<MessageType, MessageInfoType>* tf_filter_;
      uint32_t messages_received_;
  };

} // end namespace rviz

#endif // MESSAGE_FILTER_DISPLAY_MOD_H