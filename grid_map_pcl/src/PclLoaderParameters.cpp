/*
 * PclLoaderParameters.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/PclLoaderParameters.hpp"

#include <ros/console.h>


namespace grid_map {

namespace grid_map_pcl {

void PclLoaderParameters::handleYamlNode(const YAML::Node &yamlNode) {

	const std::string prefix = "pcl_grid_map_extraction";

	parameters_.numThreads_ =
			yamlNode[prefix]["num_processing_threads"].as<int>();

	parameters_.cloudTransformation_.translation_.x() =
			yamlNode[prefix]["cloud_transform"]["translation"]["x"].as<double>();
	parameters_.cloudTransformation_.translation_.y() =
			yamlNode[prefix]["cloud_transform"]["translation"]["y"].as<double>();
	parameters_.cloudTransformation_.translation_.z() =
			yamlNode[prefix]["cloud_transform"]["translation"]["z"].as<double>();

	parameters_.cloudTransformation_.rpyIntrinsic_.x() =
			yamlNode[prefix]["cloud_transform"]["rotation"]["r"].as<double>();
	parameters_.cloudTransformation_.rpyIntrinsic_.y() =
			yamlNode[prefix]["cloud_transform"]["rotation"]["p"].as<double>();
	parameters_.cloudTransformation_.rpyIntrinsic_.z() =
			yamlNode[prefix]["cloud_transform"]["rotation"]["y"].as<double>();

	parameters_.clusterExtraction_.clusterTolerance_ =
			yamlNode[prefix]["cluster_extraction"]["cluster_tolerance"].as<
					double>();
	parameters_.clusterExtraction_.minNumPoints_ =
			yamlNode[prefix]["cluster_extraction"]["min_num_points"].as<int>();
	parameters_.clusterExtraction_.maxNumPoints_ =
			yamlNode[prefix]["cluster_extraction"]["max_num_points"].as<int>();

	parameters_.outlierRemoval_.isRemoveOutliers_ =
			yamlNode[prefix]["outlier_removal"]["is_remove_outliers"].as<bool>();
	parameters_.outlierRemoval_.meanK_ =
			yamlNode[prefix]["outlier_removal"]["mean_K"].as<int>();
	parameters_.outlierRemoval_.stddevThreshold_ =
			yamlNode[prefix]["outlier_removal"]["stddev_threshold"].as<double>();

	parameters_.gridMap_.resolution_ =
			yamlNode[prefix]["grid_map"]["resolution"].as<double>();
	parameters_.gridMap_.minCloudPointsPerCell_ =
			yamlNode[prefix]["grid_map"]["min_num_points_per_cell"].as<int>();

	parameters_.downsampling_.isDownsampleCloud_ =
			yamlNode[prefix]["downsampling"]["is_downsample_cloud"].as<bool>();
	parameters_.downsampling_.voxelSize_.x() =
			yamlNode[prefix]["downsampling"]["voxel_size"]["x"].as<double>();
	parameters_.downsampling_.voxelSize_.y() =
			yamlNode[prefix]["downsampling"]["voxel_size"]["y"].as<double>();
	parameters_.downsampling_.voxelSize_.z() =
			yamlNode[prefix]["downsampling"]["voxel_size"]["z"].as<double>();

}

bool PclLoaderParameters::loadParameters(const std::string &filename) {

  YAML::Node yamlNode = YAML::LoadFile(filename);

	const bool loadingFailed = yamlNode.IsNull();
	if (loadingFailed) {
		ROS_ERROR_STREAM("PclLoaderParameters: Reading from file failed");
		return false;
	}

	try {
		handleYamlNode(yamlNode);
	}catch(const std::runtime_error &exception){
		ROS_ERROR_STREAM("PclLoaderParameters: Loading parameters failed: " << exception.what());
		return false;
	}

	return true;

}

const PclLoaderParameters::Parameters &PclLoaderParameters::get() const{
	return parameters_;
}

} // namespace grid_map_pcl

} // namespace grid_map

