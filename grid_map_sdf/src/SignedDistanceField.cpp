/*
 * Signeddistancefield.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Takahiro Miki
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_sdf/SignedDistanceField.hpp>
#include <grid_map_core/GridMap.hpp>

// signed distance field
#include "dt.h"


using namespace grid_map;
using namespace Eigen;

namespace grid_map_sdf{

// template<typename T>
SignedDistanceField::SignedDistanceField():maxDistance_(100)
{

}

// template<typename T>
SignedDistanceField::~SignedDistanceField()
{

}

void SignedDistanceField::calculateSignedDistanceField(GridMap &map, std::string layer, double heightClearance){
  resolution_ = map.getResolution();
  mapPosition_ = map.getPosition();
  size_ = map.getSize();
  MatrixXf mat = map.get(layer); 
  double min_height = 10000;
  double max_height = -10000;
  std::vector<Eigen::Array2i> nan_indices;
  for (int y = 0; y < mat.rows(); y++) {
    for (int x = 0; x < mat.cols(); x++) {
      if(!std::isnan(mat(x, y))){
        if(mat(x, y) > max_height)
          max_height = mat(x, y);
        if(mat(x, y) < min_height)
          min_height = mat(x, y);
      }
      else{
        nan_indices.push_back(Eigen::Array2i(x, y));
      }
    }
  }
  for(Eigen::Array2i index: nan_indices){
    mat(index.x(), index.y()) = max_height;
  }
  max_height += heightClearance;

  MatrixXf SDF_elevation = MatrixXf::Ones(mat.rows(), mat.cols()) * maxDistance_; 
  MatrixXf SDF_elevation_reverse = MatrixXf::Ones(mat.rows(), mat.cols()) * maxDistance_; 
  MatrixXf SDF_layer(mat.rows(), mat.cols()); 
  std::vector<MatrixXf> SDF_elevation_reverse_list;
  std::vector<MatrixXf> SDF;

  // calculate SDF from elevation downwards
  for(float j = max_height; j > min_height; j-=resolution_){
    for (int y = 0; y < SDF_elevation.rows(); y++) {
      for (int x = 0; x < SDF_elevation.cols(); x++) {
        if(SDF_elevation_reverse(x, y) == maxDistance_ && mat(x, y) > j)
          SDF_elevation_reverse(x, y) = j - mat(x, y);
        else if(SDF_elevation_reverse(x, y) != maxDistance_)
          SDF_elevation_reverse(x, y) -= resolution_;
      }
    }
    SDF_elevation_reverse_list.push_back(SDF_elevation_reverse);
  }
  zIndexStart_ = min_height;

  for(float i = min_height; i < max_height; i+=resolution_){
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>  obstacle_free_field = mat.array() < i;
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>  obstacle_field = obstacle_free_field.array() < 1;
    MatrixXf sdf_obs = get2dSDF(obstacle_field);
    MatrixXf sdf_obs_free = get2dSDF(obstacle_free_field);
    MatrixXf SDF_2d;
    if ((sdf_obs_free.array() > 100000).any())
      SDF_2d = sdf_obs;
    else
      SDF_2d = sdf_obs - sdf_obs_free;
    SDF_2d *= resolution_;
    MatrixXf SDF_elevation_reverse = SDF_elevation_reverse_list.back();
    SDF_elevation_reverse_list.pop_back();
    for (int y = 0; y < SDF_elevation.rows(); y++) {
      for (int x = 0; x < SDF_elevation.cols(); x++) {
        if(SDF_elevation(x, y) == maxDistance_ && mat(x, y) < i)
          SDF_elevation(x, y) = i - mat(x, y);
        else if(SDF_elevation(x, y) != maxDistance_ && mat(x, y) < i)
          SDF_elevation(x, y) = SDF_layer(x, y) + resolution_;
        SDF_layer(x, y) = std::min({SDF_2d(x, y), SDF_elevation(x, y), SDF_elevation_reverse(x, y)});
      }
    }
    SDF.push_back(SDF_layer);
  }
  data_ = SDF;
}

Eigen::MatrixXf SignedDistanceField::get2dSDF(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& mat){

  image<uchar> *input = new image<uchar>(mat.cols(), mat.rows(), true);

  for (int y = 0; y < input->height(); y++) {
    for (int x = 0; x < input->width(); x++) {
      imRef(input, x, y) = mat(x, y);
    }
  }
  // compute dt
  image<float> *out = dt(input);

  Eigen::MatrixXf result(mat.rows(), mat.cols());

  // take square roots
  for (int y = 0; y < out->height(); y++) {
    for (int x = 0; x < out->width(); x++) {
      result(x, y) = sqrt(imRef(out, x, y));
    }
  }
  return result;
}

double SignedDistanceField::getDistanceAt(Eigen::Vector3f position){
  double xCenter = size_.x() / 2;
  double yCenter = size_.y() / 2;
  int i = std::round(size_.x() - xCenter - (position.x() - mapPosition_.x()) / resolution_);
  int j = std::round(size_.y() - yCenter - (position.y() - mapPosition_.y()) / resolution_);
  int k = std::round((position.z() - zIndexStart_) / resolution_);
  // if( i < 0 || i >= size_.x())
  //   return -10000;
  // if( j < 0 || j >= size_.y())
  //   return -10000;
  // if( k < 0 || k >= (int)data_.size())
  //   return -10000;
  i = std::max(i, 0);
  i = std::min(i, size_.x() - 1);
  j = std::max(j, 0);
  j = std::min(j, size_.y() - 1);
  k = std::max(k, 0);
  k = std::min(k, (int)data_.size() - 1);
  return data_[k](i, j);
}


Eigen::Vector3f SignedDistanceField::getDistanceGradientAt(Eigen::Vector3f position){
  double dx = (getDistanceAt(position + Vector3f(resolution_, 0, 0)) - getDistanceAt(position - Vector3f(resolution_, 0, 0))) / (2 * resolution_);
  double dy = (getDistanceAt(position + Vector3f(0, resolution_, 0)) - getDistanceAt(position - Vector3f(0, resolution_, 0))) / (2 * resolution_);
  double dz = (getDistanceAt(position + Vector3f(0, 0, resolution_)) - getDistanceAt(position - Vector3f(0, 0, resolution_))) / (2 * resolution_);
  // std::cout << "position " << position << std::endl;
  // std::cout << "dx " << dx << std::endl;
  // std::cout << "dy " << dy << std::endl;
  // std::cout << "dz " << dz << std::endl;
  return Vector3f(dx, dy, dz);
}

void SignedDistanceField::convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points){
  double xCenter = size_.x() / 2;
  double yCenter = size_.y() / 2;
  for (int z = 0; z < data_.size(); z++){
    for (int y = 0; y < size_.y(); y++) {
      for (int x = 0; x < size_.x(); x++) {
        double xp = mapPosition_.x() + ((size_.x() - x) - xCenter) * resolution_;
        double yp = mapPosition_.y() + ((size_.y() - y) - yCenter) * resolution_;
        double zp = zIndexStart_ + z * resolution_;
        pcl::PointXYZI p;
        p.x = xp;
        p.y = yp;
        p.z = zp;
        p.intensity = data_[z](x, y);
        points.push_back(p);
      }
    }
  }
  return;
}


} /* namespace */
