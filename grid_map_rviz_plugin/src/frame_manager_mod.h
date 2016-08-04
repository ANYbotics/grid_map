 /*
  * Copyright (c) 2009, Willow Garage, Inc.
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
 
 #ifndef RVIZ_FRAME_MANAGER_H
 #define RVIZ_FRAME_MANAGER_H
 
 #include <map>
 
 #include <QObject>
 
 #include <ros/time.h>
 
 #include <OgreVector3.h>
 #include <OgreQuaternion.h>
 
 #include <boost/thread/mutex.hpp>
 
 #include <geometry_msgs/Pose.h>
 
 #ifndef Q_MOC_RUN
#include "message_filter_mod.h"
 #endif
 
 namespace tf
 {
 class TransformListener;
 }
 
 namespace rviz
 {
 class Display;
 
 class FrameManager: public QObject
 {
 Q_OBJECT
 public:
 
   enum SyncMode {
     SyncOff = 0,
     SyncExact,
     SyncApprox
   };
 
   FrameManager(boost::shared_ptr<tf::TransformListener> tf = boost::shared_ptr<tf::TransformListener>());
 
   ~FrameManager();
 
    void setFixedFrame(const std::string& frame);
 
    void setPause( bool pause );
 
    bool getPause() { return pause_; }
 
    void setSyncMode( SyncMode mode );
 
    SyncMode getSyncMode() { return sync_mode_; }
 
    void syncTime( ros::Time time );
 
    ros::Time getTime() { return sync_time_; }
 
   template<typename Header>
   bool getTransform(const Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation)
   {
     return getTransform(header.frame_id, header.stamp, position, orientation);
   }
 
   bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation);
 
   template<typename Header>
   bool transform(const Header& header, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation)
   {
     return transform(header.frame_id, header.stamp, pose, position, orientation);
   }
 
   bool transform(const std::string& frame, ros::Time time, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation);
 
   void update();
 
   bool frameHasProblems(const std::string& frame, ros::Time time, std::string& error);
 
   bool transformHasProblems(const std::string& frame, ros::Time time, std::string& error);
 
   template<class M, class MInfo>
   void registerFilterForTransformStatusCheck(tf::MessageFilter<M, MInfo>* filter, Display* display)
   {
     filter->registerCallback(boost::bind(&FrameManager::messageCallback<M>, this, _1, display));
     filter->registerFailureCallback(boost::bind(&FrameManager::failureCallback<M>, this, _1, _2, display));
   }
 
   const std::string& getFixedFrame() { return fixed_frame_; }
 
   tf::TransformListener* getTFClient() { return tf_.get(); }
 
   const boost::shared_ptr<tf::TransformListener>& getTFClientPtr() { return tf_; }
 
   std::string discoverFailureReason(const std::string& frame_id,
                                     const ros::Time& stamp,
                                     const std::string& caller_id,
                                     tf::FilterFailureReason reason);
 
 Q_SIGNALS:
   void fixedFrameChanged();
 
 private:
 
   bool adjustTime( const std::string &frame, ros::Time &time );
 
   template<class M>
   void messageCallback(const ros::MessageEvent<M const>& msg_evt, Display* display)
   {
     boost::shared_ptr<M const> const &msg = msg_evt.getConstMessage();
     std::string authority = msg_evt.getPublisherName();
 
     messageArrived(msg->info.header.frame_id, msg->info.header.stamp, authority, display);
   }
 
   template<class M>
   void failureCallback(const ros::MessageEvent<M const>& msg_evt, tf::FilterFailureReason reason, Display* display)
   {
     boost::shared_ptr<M const> const &msg = msg_evt.getConstMessage();
     std::string authority = msg_evt.getPublisherName();
 
     messageFailed(msg->info.header.frame_id, msg->info.header.stamp, authority, reason, display);
   }
 
   void messageArrived(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, Display* display);
   void messageFailed(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason, Display* display);
 
   struct CacheKey
   {
     CacheKey(const std::string& f, ros::Time t)
     : frame(f)
     , time(t)
     {}
 
     bool operator<(const CacheKey& rhs) const
     {
       if (frame != rhs.frame)
       {
         return frame < rhs.frame;
       }
 
       return time < rhs.time;
     }
 
     std::string frame;
     ros::Time time;
   };
 
   struct CacheEntry
   {
     CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion& o)
     : position(p)
     , orientation(o)
     {}
 
     Ogre::Vector3 position;
     Ogre::Quaternion orientation;
   };
   typedef std::map<CacheKey, CacheEntry > M_Cache;
 
   boost::mutex cache_mutex_;
   M_Cache cache_;
 
   boost::shared_ptr<tf::TransformListener> tf_;
   std::string fixed_frame_;
 
   bool pause_;
 
   SyncMode sync_mode_;
 
   // the current synchronized time, used to overwrite ros:Time(0)
   ros::Time sync_time_;
 
   // used for approx. syncing
   double sync_delta_;
   double current_delta_;
 };
 
 }
 
 #endif // RVIZ_FRAME_MANAGER_H