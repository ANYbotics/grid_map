/*
 * PointCloud2_to_GridMap_msg_node.hpp
 *
 *  Created on: July 03, 2021
 *      Author: Maximilian Stölzle
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>


//namespace grid_map_pcl {
class PointCloud2ToGridMapMsgNode
{
    private:
        std::string gridMapTopic_;
        std::string pointCloudTopic_;

        ros::Subscriber sub_;
        ros::Publisher pub_;
    public:
        PointCloud2ToGridMapMsgNode(ros::NodeHandle& nodeHandle);
        ~PointCloud2ToGridMapMsgNode();
};
//}
