/*
 * PointCloud2_to_GridMap_msg_node.hpp
 *
 *  Created on: July 03, 2021
 *      Author: Maximilian Stölzle
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include "grid_map_pcl/GridMapPclLoader.hpp"


//namespace grid_map_pcl {
class PointCloud2ToGridMapMsgNode
{
    public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        PointCloud2ToGridMapMsgNode(ros::NodeHandle& nodeHandle);
        ~PointCloud2ToGridMapMsgNode();

        /*!
        * Callback function for the point cloud 2.
        * @param message the point cloud2 message to be converted to a grid map msg
        */
        void sub_callback(const sensor_msgs::PointCloud2 & msg);

    private:
        std::string gridMapTopic_;
        std::string pointCloudTopic_;

        //! ROS nodehandle.
        ros::NodeHandle nodeHandle_;

        ros::Subscriber sub_;
        ros::Publisher pub_;
};
//}
