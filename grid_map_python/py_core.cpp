#include <grid_map_core/GridMap.hpp>

// Python binding
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using grid_map::GridMap;

void init_core(py::module m) {
  py::module& base_m(m);

  py::class_<GridMap, std::shared_ptr<GridMap>>(base_m, "GridMap")
    .def(py::init<>())
    .def(py::init<const std::vector<std::string>>())
    .def("setGeometry",     py::overload_cast<const grid_map::Length&, const double, const grid_map::Position&>(&GridMap::setGeometry), py::arg("length"), py::arg("resolution"), py::arg("position"))
    .def("add",             py::overload_cast<const std::string&, const double>(&GridMap::add), py::arg("layer"), py::arg("value"))
    .def("add",             py::overload_cast<const std::string&, const GridMap::Matrix&>(&GridMap::add), py::arg("layer"), py::arg("data"))
    .def("exists",          &GridMap::exists, py::arg("layer"))
    .def("get",             py::overload_cast<const std::string&>(&GridMap::get, py::const_), py::arg("layer"))
    .def("get",             py::overload_cast<const std::string&>(&GridMap::get), py::arg("layer"))
    .def("__getitem__",     py::overload_cast<const std::string&>(&GridMap::operator[], py::const_), py::arg("layer"))
    .def("__getitem__",     py::overload_cast<const std::string&>(&GridMap::operator[]), py::arg("layer"))
    .def("erase",           &GridMap::erase, py::arg("layer"));
    .def("getLayers",       &GridMap::getLayers)
    .def("hasSameLayers",   &GridMap::hasSameLayers, py::arg("other"))
    .def("atPosition",      py::overload_cast<const std::string&, const grid_map::Position&>(&GridMap::atPosition), py::arg("layer"), py::arg("position"))
    .def("atPosition",      py::overload_cast<const std::string&, const grid_map::Position&, grid_map::InterpolationMethods>(&GridMap::atPosition, py::const_), py::arg("layer"), py::arg("position"), py::arg("interpolationMethod"))
    .def("addDataFrom",     &GridMap::addDataFrom, py::arg("other"), py::arg("extendMap"), py::arg("overwriteData"), py::arg("copyAllLayers"), py::arg("layers"))
    .def("extendToInclude", &GridMap::extendToInclude, py::arg("other"))
    .def("clear",           &GridMap::clear, py::arg("layer"))
    .def("clearAll",        &GridMap::clearAll)
    .def("setTimestamp",    &GridMap::setTimestamp, py::arg("timestamp"))
    .def("getTimestamp",    &GridMap::getTimestamp)
    .def("setFrameId",      &GridMap::setFrameId, py::arg("frameId"))
    .def("getFrameId",      &GridMap::getFrameId)
    .def("getLength",       &GridMap::getLength)
    .def("getPosition",     py::overload_cast<>(&GridMap::getPosition, py::const_))
    .def("getResolution",   &GridMap::getResolution)
    .def("getSize",         &GridMap::getSize)
    .def("setStartIndex",   &GridMap::setStartIndex, py::arg("startIndex"))
    .def("getStartIndex",   &GridMap::getStartIndex)
}
