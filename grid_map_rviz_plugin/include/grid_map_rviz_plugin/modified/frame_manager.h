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
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose.h>

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <tf2_ros/message_filter.h>
#endif

namespace tf2_ros
{
class TransformListener;
}

namespace rviz
{
class Display;

/** @brief Helper class for transforming data into Ogre's world frame (the fixed frame).
 *
 * During one frame update (nominally 33ms), the tf tree stays consistent and queries are cached for
 * speedup.
 */
class FrameManager : public QObject
{
  Q_OBJECT
public:
  enum SyncMode
  {
    SyncOff = 0,
    SyncExact,
    SyncApprox
  };

  /// Constructor, will create a TransformListener (and Buffer) automatically if not provided
  explicit FrameManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                        std::shared_ptr<tf2_ros::TransformListener> tf_listener =
                            std::shared_ptr<tf2_ros::TransformListener>());

  /** @brief Destructor.
   *
   * FrameManager should not need to be destroyed by hand, just
   * destroy the boost::shared_ptr returned by instance(), and it will
   * be deleted when the last reference is removed. */
  ~FrameManager() override;

  /** @brief Set the frame to consider "fixed", into which incoming data is transformed.
   *
   * The fixed frame serves as the reference for all getTransform()
   * and transform() functions in FrameManager. */
  void setFixedFrame(const std::string& frame);

  /** @brief Enable/disable pause mode */
  void setPause(bool pause);

  bool getPause()
  {
    return pause_;
  }

  /** @brief Set synchronization mode (off/exact/approximate) */
  void setSyncMode(SyncMode mode);

  SyncMode getSyncMode()
  {
    return sync_mode_;
  }

  /** @brief Synchronize with given time. */
  void syncTime(ros::Time time);

  /** @brief Get current time, depending on the sync mode. */
  ros::Time getTime()
  {
    return sync_time_;
  }

  /** @brief Return the pose for a header, relative to the fixed frame, in Ogre classes.
   * @param[in] header The source of the frame name and time.
   * @param[out] position The position of the header frame relative to the fixed frame.
   * @param[out] orientation The orientation of the header frame relative to the fixed frame.
   * @return true on success, false on failure. */
  template <typename Header>
  bool getTransform(const Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation)
  {
    return getTransform(header.frame_id, header.stamp, position, orientation);
  }

  /** @brief Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
   * @param[in] frame The frame to find the pose of.
   * @param[in] time The time at which to get the pose.
   * @param[out] position The position of the frame relative to the fixed frame.
   * @param[out] orientation The orientation of the frame relative to the fixed frame.
   * @return true on success, false on failure. */
  bool getTransform(const std::string& frame,
                    ros::Time time,
                    Ogre::Vector3& position,
                    Ogre::Quaternion& orientation);

  /** @brief Transform a pose from a frame into the fixed frame.
   * @param[in] header The source of the input frame and time.
   * @param[in] pose The input pose, relative to the header frame.
   * @param[out] position Position part of pose relative to the fixed frame.
   * @param[out] orientation: Orientation part of pose relative to the fixed frame.
   * @return true on success, false on failure. */
  template <typename Header>
  bool transform(const Header& header,
                 const geometry_msgs::Pose& pose,
                 Ogre::Vector3& position,
                 Ogre::Quaternion& orientation)
  {
    return transform(header.frame_id, header.stamp, pose, position, orientation);
  }

  /** @brief Transform a pose from a frame into the fixed frame.
   * @param[in] frame The input frame.
   * @param[in] time The time at which to get the pose.
   * @param[in] pose The input pose, relative to the input frame.
   * @param[out] position Position part of pose relative to the fixed frame.
   * @param[out] orientation: Orientation part of pose relative to the fixed frame.
   * @return true on success, false on failure. */
  bool transform(const std::string& frame,
                 ros::Time time,
                 const geometry_msgs::Pose& pose,
                 Ogre::Vector3& position,
                 Ogre::Quaternion& orientation);

  /** @brief Clear the internal cache. */
  void update();

  /** @brief Check to see if a frame exists in our tf buffer.
   * @param[in] frame The name of the frame to check.
   * @param[in] time Dummy parameter, not actually used.
   * @param[out] error If the frame does not exist, an error message is stored here.
   * @return true if the frame does not exist, false if it does exist. */
  bool frameHasProblems(const std::string& frame, ros::Time time, std::string& error);

  /** @brief Check to see if a transform is known between a given frame and the fixed frame.
   * @param[in] frame The name of the frame to check.
   * @param[in] time The time at which the transform is desired.
   * @param[out] error If the transform is not known, an error message is stored here.
   * @return true if the transform is not known, false if it is. */
  bool transformHasProblems(const std::string& frame, ros::Time time, std::string& error);

  /** Connect success and failure callbacks to a tf2_ros::MessageFilter.
   * @param filter The tf2_ros::MessageFilter to connect to.
   * @param display The Display using the filter.
   *
   * FrameManager has internal functions for handling success and failure of tf2_ros::MessageFilters,
   * which call Display::setStatus() based on success or failure of the filter, including appropriate
   * error messages. */
  template <class M>
  void registerFilterForTransformStatusCheck(tf2_ros::MessageFilter<M>* filter, Display* display)
  {
    filter->registerCallback(boost::bind(&FrameManager::messageCallback<M>, this, _1, display));
    filter->registerFailureCallback(boost::bind(
        &FrameManager::failureCallback<M, tf2_ros::FilterFailureReason>, this, _1, _2, display));
  }

  /** @brief Return the current fixed frame name. */
  const std::string& getFixedFrame()
  {
    return fixed_frame_;
  }

  const std::shared_ptr<tf2_ros::Buffer> getTF2BufferPtr()
  {
    return tf_buffer_;
  }

  /** Create a description of a transform problem.
   * @param frame_id The name of the frame with issues.
   * @param stamp The time for which the problem was detected.
   * @param caller_id Dummy parameter, not used.
   * @param reason The reason given by the tf2_ros::MessageFilter in its failure callback.
   * @return An error message describing the problem.
   *
   * Once a problem has been detected with a given frame or transform,
   * call this to get an error message describing the problem. */
  std::string discoverFailureReason(const std::string& frame_id,
                                    const ros::Time& stamp,
                                    const std::string& caller_id,
                                    tf2_ros::FilterFailureReason reason);

Q_SIGNALS:
  /** @brief Emitted whenever the fixed frame changes. */
  void fixedFrameChanged();

private:
  bool adjustTime(const std::string& frame, ros::Time& time);

  template <class M>
  void messageCallback(const ros::MessageEvent<M const>& msg_evt, Display* display)
  {
    boost::shared_ptr<M const> const& msg = msg_evt.getConstMessage();
    const std::string& authority = msg_evt.getPublisherName();
    
    // the following diverges from original rviz sources (noetic) for backward compatibility
    messageArrived(msg->info.header.frame_id, msg->info.header.stamp, authority, display);
  }

  template <class M, class TfFilterFailureReasonType>
  void failureCallback(const ros::MessageEvent<M const>& msg_evt,
                       TfFilterFailureReasonType reason,
                       Display* display)
  {
    boost::shared_ptr<M const> const& msg = msg_evt.getConstMessage();
    const std::string& authority = msg_evt.getPublisherName();

    // the following diverges from original rviz sources (noetic) for backward compatibility
    messageFailed(msg->info.header.frame_id, msg->info.header.stamp, authority, reason, display);
  }

  void messageArrived(const std::string& frame_id,
                      const ros::Time& stamp,
                      const std::string& caller_id,
                      Display* display);

  void messageFailedImpl(const std::string& caller_id, const std::string& status_text, Display* display);

  template <class TfFilterFailureReasonType>
  void messageFailed(const std::string& frame_id,
                     const ros::Time& stamp,
                     const std::string& caller_id,
                     TfFilterFailureReasonType reason,
                     Display* display)
  {
    std::string status_text = discoverFailureReason(frame_id, stamp, caller_id, reason);
    messageFailedImpl(caller_id, status_text, display);
  }

  struct CacheKey
  {
    CacheKey(const std::string& f, ros::Time t) : frame(f), time(t)
    {
    }

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
    CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion& o) : position(p), orientation(o)
    {
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };
  typedef std::map<CacheKey, CacheEntry> M_Cache;

  boost::mutex cache_mutex_;
  M_Cache cache_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string fixed_frame_;
  tf2::CompactFrameID fixed_frame_id_;

  bool pause_;

  SyncMode sync_mode_;

  // the current synchronized time, used to overwrite ros:Time(0)
  ros::Time sync_time_;

  // used for approx. syncing
  double sync_delta_;
  double current_delta_;
};

} // namespace rviz

#endif // RVIZ_FRAME_MANAGER_H
