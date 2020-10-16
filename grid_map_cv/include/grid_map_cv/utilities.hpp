#ifndef GRID_MAP_CV__UTILITIES_HPP_
#define GRID_MAP_CV__UTILITIES_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace grid_map
{

/*!
 * This is class is meant as a replacement for the filters::FilterBase<T>::getParam method.
 * This class offer declaring and reading paramters.
 * It only returns a value if the the user has set the parameter in the parameter
 * configuration yaml file and does not return a default value if the parameter
 * is not found (unlike the FilterBase method mentioned above).
 */
class ParameterReader
{
public:
  /*!
   * Constructor
   * @param param_prefix The parameter prefix.
   * @param params_interface The node parameters interface to use.
   */
  ParameterReader(
    std::string param_prefix,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface)
  : param_prefix_(param_prefix),
    params_interface_(params_interface) {}

  /*!
   * Retrives parameter value when defined by user.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @param param_type The excpected type of the parameter.
   * @return - true if a parameter defined by the user is found.
   */
  template<typename T>
  bool get_template(const std::string & name, T & value_out, rclcpp::ParameterType param_type)
  {
    rclcpp::Parameter param;

    params_interface_->declare_parameter(param_prefix_ + name, rclcpp::ParameterValue());
    params_interface_->get_parameter(param_prefix_ + name, param);

    if (param.get_type() != param_type) {
      return false;
    } else {
      value_out = param.get_value<T>();
      return true;
    }
  }

  /*!
   * Reads parameter of type int.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, int & value_out)
  {
    return get_template<int>(name, value_out, rclcpp::PARAMETER_INTEGER);
  }

  /*!
   * Reads parameter of type double.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, double & value_out)
  {
    return get_template<double>(name, value_out, rclcpp::PARAMETER_DOUBLE);
  }

  /*!
   * Reads parameter of type std::string.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, std::string & value_out)
  {
    return get_template<std::string>(name, value_out, rclcpp::PARAMETER_STRING);
  }

  /*!
   * Reads parameter of type boolean.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, bool & value_out)
  {
    return get_template<bool>(name, value_out, rclcpp::PARAMETER_BOOL);
  }

  /*!
   * Reads parameter of type array of double.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, std::vector<double> & value_out)
  {
    return get_template(name, value_out, rclcpp::PARAMETER_DOUBLE_ARRAY);
  }

  /*!
   * Reads parameter of type array of string.
   * @param name The name of the parameter to read.
   * @param value_out The value of the parameter, will only be changed if the parameter is found.
   * @return - true if a parameter defined by the user is found.
   */
  bool get(const std::string & name, std::vector<std::string> & value_out)
  {
    return get_template<std::vector<std::string>>(name, value_out, rclcpp::PARAMETER_STRING_ARRAY);
  }

private:
  //! Layer the threshold should be applied to.
  std::string param_prefix_;

  //! Parameters interface
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;
};
}  // namespace grid_map

#endif  // GRID_MAP_CV__UTILITIES_HPP_
