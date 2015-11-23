/*
 * GridMapRegistrationDemo.cpp
 *
 *  Created on: Sept 18, 2015
 *      Author: Elena Stumm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_demos/GridMapRegistrationDemo.hpp"

namespace grid_map_demos {

GridMapRegistrationDemo::GridMapRegistrationDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      localMap_(grid_map::GridMap({"elevation"})),
      globalMap_(grid_map::GridMap({"elevation"}))
{
  readParameters();
  loadGlobalMap();
  gridMapSubscriber_ = nodeHandle_.subscribe(localBagTopic_, 1, &GridMapRegistrationDemo::gridMapCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(publishTopic_, 1, true);
}

GridMapRegistrationDemo::~GridMapRegistrationDemo()
{
}

bool GridMapRegistrationDemo::readParameters()
{
  nodeHandle_.param("global_bag_topic", globalBagTopic_, std::string("grid_map"));
  nodeHandle_.param("local_bag_topic", localBagTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  nodeHandle_.param("publish_topic", publishTopic_, std::string("/grid_map"));
  nodeHandle_.param("bag_file_path", bagFilePath_, std::string());
  return true;
}
bool GridMapRegistrationDemo::loadGlobalMap()
{
  ROS_INFO_STREAM("Loading grid map from path " << bagFilePath_ << ".");
  return grid_map::GridMapRosConverter::loadFromBag(bagFilePath_, globalBagTopic_, globalMap_);
}

void GridMapRegistrationDemo::gridMapCallback(const grid_map_msgs::GridMap& msg)
{
  ROS_INFO("local grid map callback");
  grid_map::GridMapRosConverter::fromMessage(msg, localMap_);
  cv::Mat globalElevationImage;
  cv::Mat localElevationImage;
  grid_map::GridMapRosConverter::toCvImage(globalMap_, "elevation", globalElevationImage);
  grid_map::GridMapRosConverter::toCvImage(localMap_, "elevation",  localElevationImage);

  // blur the two map images
  const int blur = 5;
  cv::GaussianBlur(globalElevationImage, globalElevationImage, cv::Size(blur,blur), 0);
  cv::GaussianBlur(localElevationImage, localElevationImage, cv::Size(blur,blur), 0);

  // compute match scores:
  const int scoringMethod = cv::TM_CCORR; //only ccorr works without handeling alpha explicitly
  const int interpolationMethod = cv::INTER_NEAREST; //or cubic, linear, nearest

  int num_query_rows = localElevationImage.rows;
  int num_query_cols = localElevationImage.cols;

  cv::copyMakeBorder(globalElevationImage, globalElevationImage, num_query_rows/2, num_query_rows/2, num_query_cols/2, num_query_cols/2, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

  int scores_cols = globalElevationImage.cols - localElevationImage.cols + 1;
  int scores_rows = globalElevationImage.rows - localElevationImage.rows + 1;
  cv::Mat scores;
  scores = cv::Mat::zeros(scores_rows, scores_cols, CV_64FC1);


  float bestMatchVal = 0;
  cv::Point bestMatchLoc;
  int bestRot;
  for (int r = -30; r < 30; r=r+2) {
    scores = cv::Mat::zeros(scores_rows, scores_cols, CV_64FC1);
    cv::Mat R = cv::getRotationMatrix2D(cv::Point(num_query_cols/2, num_query_rows/2), (double)r, 1.);
    cv::Mat rotated;
    rotated = cv::Mat::zeros(num_query_rows, num_query_cols, CV_8UC4);
    cv::warpAffine(localElevationImage, rotated, R, cv::Point(num_query_cols, num_query_rows), interpolationMethod, cv::BORDER_TRANSPARENT);
    cv::matchTemplate( globalElevationImage, rotated, scores, scoringMethod );

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    cv::minMaxLoc(scores, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    if (maxVal >= bestMatchVal) {
      bestMatchVal = maxVal;
      bestMatchLoc = maxLoc;
      bestRot = r;
    }
  }

  // Display info.
  ROS_INFO("match score: %f, and match location: (%d, %d), angle: %d", bestMatchVal, bestMatchLoc.x, bestMatchLoc.y, bestRot);
  cv::rectangle( globalElevationImage, bestMatchLoc, cv::Point( bestMatchLoc.x + num_query_cols , bestMatchLoc.y + num_query_rows ), cv::Scalar(255,0,0), 1);
  cv::rectangle( localElevationImage, cv::Point(0,0), cv::Point(num_query_cols-1, num_query_rows-1 ), cv::Scalar(0,255,0), 1);

  cv::resize(globalElevationImage, globalElevationImage, cv::Size(), 2, 2);
  cv::resize(localElevationImage, localElevationImage, cv::Size(), 2, 2);
  cv::imshow( "global elevation map", globalElevationImage );
  cv::imshow( "local elevation map", localElevationImage );
  cv::waitKey(50);

  // Publish the global grid map for display in Rviz.
  grid_map_msgs::GridMap globalMapMessage;
  grid_map::GridMapRosConverter::toMessage(globalMap_, globalMapMessage);
  gridMapPublisher_.publish(globalMapMessage);
}

} /* namespace */
