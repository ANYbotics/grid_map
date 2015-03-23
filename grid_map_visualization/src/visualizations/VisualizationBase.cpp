/*
 * VisualizationBase.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization {

VisualizationBase::VisualizationBase(ros::NodeHandle& nodeHandle, const std::string& name)
    : nodeHandle_(nodeHandle),
      name_(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::readParameters(XmlRpc::XmlRpcValue& config)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("A filter configuration must be a map with fields name, type, and params.");
    return false;
  }

  // Check to see if we have parameters in our list.
  if (config.hasMember("params")) {
    XmlRpc::XmlRpcValue params = config["params"];
    if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("Params must be a map.");
      return false;
    } else {
      for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
        ROS_DEBUG("Loading param %s\n", it->first.c_str());
        parameters_[it->first] = it->second;
      }
    }
  }

  return true;
}

bool VisualizationBase::getParam(const std::string& name, std::string& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  value = std::string(it->second);
  return true;
}

bool VisualizationBase::getParam(const std::string& name, double& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if (it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble
      && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ?
          (int) (it->second) : (double) (it->second);
  return true;
}

bool VisualizationBase::getParam(const std::string&name, int& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  value = it->second;
  return true;
}

///** \brief Get a filter parameter as an unsigned int
// * \param name The name of the parameter
// * \param value The int to set with the value
// * \return Whether or not the parameter of name/type was set */
//bool VisualizationBase::getParam(const std::string&name, unsigned  int& value)
//{
//  int signed_value;
//  if (!getParam(name, signed_value))
//    return false;
//  if (signed_value < 0)
//    return false;
//  value = signed_value;
//  return true;
//};
//
///** \brief Get a filter parameter as a std::vector<double>
// * \param name The name of the parameter
// * \param value The std::vector<double> to set with the value
// * \return Whether or not the parameter of name/type was set */
//bool VisualizationBase::getParam(const std::string&name, std::vector<double>& value)
//{
//  string_map_t::iterator it = params_.find(name);
//  if (it == params_.end())
//  {
//    return false;
//  }
//
//  value.clear();
//
//  if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
//  {
//    return false;
//  }
//
//  XmlRpc::XmlRpcValue double_array = it->second;
//
//  for (int i = 0; i < double_array.size(); ++i){
//    if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
//    {
//      return false;
//    }
//
//    double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
//    value.push_back(double_value);
//  }
//
//  return true;
//}
//
///** \brief Get a filter parameter as a std::vector<string>
// * \param name The name of the parameter
// * \param value The std::vector<sgring> to set with the value
// * \return Whether or not the parameter of name/type was set */
//bool VisualizationBase::getParam(const std::string&name, std::vector<std::string>& value)
//{
//  string_map_t::iterator it = params_.find(name);
//  if (it == params_.end())
//  {
//    return false;
//  }
//
//  value.clear();
//
//  if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
//  {
//    return false;
//  }
//
//  XmlRpc::XmlRpcValue string_array = it->second;
//
//  for (unsigned int i = 0; i < string_array.size(); ++i){
//    if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString)
//    {
//      return false;
//    }
//
//    value.push_back(string_array[i]);
//  }
//
//  return true;
//}

} /* namespace */
