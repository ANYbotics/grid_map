#include <grid_map_core/GridMap.hpp>

// Python binding
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using grid_map::GridMap;
using grid_map::Position;
using grid_map::Index;
using grid_map::Length;
using grid_map::Time;
using grid_map::InterpolationMethods;
constexpr static auto pyref = py::return_value_policy::reference_internal;

void init_core(py::module m) {
  py::module& core(m);

  py::enum_<InterpolationMethods>(core, "InterpolationMethods")
    .value("INTER_NEAREST", InterpolationMethods::INTER_NEAREST)
    .value("INTER_LINEAR", InterpolationMethods::INTER_LINEAR)
    .value("INTER_CUBIC_CONVOLUTION", InterpolationMethods::INTER_CUBIC_CONVOLUTION)
    .value("INTER_CUBIC", InterpolationMethods::INTER_CUBIC)
    .export_values();

  py::class_<GridMap, std::shared_ptr<GridMap>>(core, "GridMapBinding")
    // Constructors (copy handled below and destruction is done by pybind11 itself)
    // (move constructor and operator= are not handled in python)
    .def(py::init<const std::vector<std::string>>())
    .def(py::init<>())

    // Binding public functions of GridMap.hpp
    .def("setGeometry",       py::overload_cast<const Length&, const double, const Position&>(&GridMap::setGeometry), py::arg("length"), py::arg("resolution"), py::arg("position") = Position::Zero())
    .def("add",               py::overload_cast<const std::string&, const double>(&GridMap::add), py::arg("layer"), py::arg("value")=NAN)
    .def("add",               py::overload_cast<const std::string&, const GridMap::Matrix&>(&GridMap::add), py::arg("layer"), py::arg("data"))
    .def("exists",            &GridMap::exists, py::arg("layer"))
    .def("getc",              py::overload_cast<const std::string&>(&GridMap::get, py::const_), py::arg("layer")) // Constness is not supported in python, return a copy
    .def("get",               py::overload_cast<const std::string&>(&GridMap::get), py::arg("layer"), pyref)
    .def("__getitem__",       py::overload_cast<const std::string&>(&GridMap::operator[]), py::arg("layer"), pyref)
    .def("erase",             &GridMap::erase, py::arg("layer"))
    .def("getLayers",         &GridMap::getLayers)
    .def("setBasicLayers",    &GridMap::setBasicLayers, py::arg("basicLayers"))
    .def("getBasicLayers",    &GridMap::getBasicLayers)
    .def("hasBasicLayers",    &GridMap::hasBasicLayers)
    .def("hasSameLayers",     &GridMap::hasSameLayers, py::arg("other"))
    .def("atPosition",        py::overload_cast<const std::string&, const Position&>(&GridMap::atPosition), py::arg("layer"), py::arg("position"), pyref)
    .def("atPosition",        py::overload_cast<const std::string&, const Position&, InterpolationMethods>(&GridMap::atPosition, py::const_), py::arg("layer"), py::arg("position"), py::arg("interpolationMethod")=InterpolationMethods::INTER_NEAREST)
    .def("atc",               py::overload_cast<const std::string&, const Index&>(&GridMap::at, py::const_), py::arg("layer"), py::arg("index")) // Constness is not supported in python, return a copy
    .def("at",                py::overload_cast<const std::string&, const Index&>(&GridMap::at), py::arg("layer"), py::arg("index"), pyref)
    .def("getIndex",          &GridMap::getIndex, py::arg("position"), py::arg("index"))
    .def("getPosition",       py::overload_cast<const Index&, Position&>(&GridMap::getPosition, py::const_), py::arg("index"), py::arg("position"))
    .def("isInside",          &GridMap::isInside, py::arg("position"))
    .def("isValidAt",         py::overload_cast<const Index&>(&GridMap::isValid, py::const_), py::arg("index"))
    .def("isLayerValidAt",    py::overload_cast<const Index&, const std::string&>(&GridMap::isValid, py::const_), py::arg("index"), py::arg("layer"))
    .def("isLayersValidAt",   py::overload_cast<const Index&, const std::vector<std::string>&>(&GridMap::isValid, py::const_), py::arg("index"), py::arg("layers"))
    .def("getPosition3",      &GridMap::getPosition3, py::arg("layer"), py::arg("index"), py::arg("position"))
    .def("getVector",         &GridMap::getVector, py::arg("layerPrefix"), py::arg("index"), py::arg("vector"))
    .def("getSubmap",         py::overload_cast<const Position&, const Length&, bool&>(&GridMap::getSubmap, py::const_), py::arg("position"), py::arg("length"), py::arg("success"))
    .def("getSubmap",         py::overload_cast<const Position&, const Length&, Index&, bool&>(&GridMap::getSubmap, py::const_), py::arg("position"), py::arg("length"), py::arg("indexInSubmap"), py::arg("success"))
    .def("getTransformedMap", &GridMap::getTransformedMap, py::arg("transform"), py::arg("heightLayerName"), py::arg("newFrameId"), py::arg("sampleRatio") = 0.0)
    .def("setPosition",       &GridMap::setPosition, py::arg("position"))
    .def("move",              py::overload_cast<const Position&>(&GridMap::move), py::arg("position"))
    .def("addDataFrom",       &GridMap::addDataFrom, py::arg("other"), py::arg("extendMap"), py::arg("overwriteData"), py::arg("copyAllLayers"), py::arg("layers"))
    .def("extendToInclude",   &GridMap::extendToInclude, py::arg("other"))
    .def("clear",             &GridMap::clear, py::arg("layer"))
    .def("clearBasic",        &GridMap::clearBasic)
    .def("clearAll",          &GridMap::clearAll)
    .def("setTimestamp",      &GridMap::setTimestamp, py::arg("timestamp"))
    .def("getTimestamp",      &GridMap::getTimestamp)
    .def("setFrameId",        &GridMap::setFrameId, py::arg("frameId"))
    .def("getFrameId",        &GridMap::getFrameId)
    .def("getLength",         &GridMap::getLength)
    .def("getPosition",       py::overload_cast<>(&GridMap::getPosition, py::const_))
    .def("getResolution",     &GridMap::getResolution)
    .def("getSize",           &GridMap::getSize)
    .def("setStartIndex",     &GridMap::setStartIndex, py::arg("startIndex"))
    .def("getStartIndex",     &GridMap::getStartIndex)
    .def("isDefaultStartIndex",        &GridMap::isDefaultStartIndex)
    .def("convertToDefaultStartIndex", &GridMap::convertToDefaultStartIndex)
    .def("getClosestPositionInMap",    &GridMap::getClosestPositionInMap, py::arg("position"))

    // Copy support
    .def("__copy__",  [](const GridMap& self) { return GridMap(self); })
    .def("__deepcopy__", [](const GridMap& self, py::dict) { return GridMap(self); })

    // Pickle support
    .def(py::pickle(
      [](const GridMap& gm) {
        // Equivalent to __getstate__, return a tuple fully encoding the object
        const std::vector<std::string>& layers = gm.getLayers();
        std::vector<GridMap::Matrix> grids;
        std::transform(layers.cbegin(), layers.cend(), std::back_inserter(grids), [&gm](const std::string& layer){ return gm[layer]; });

        return py::make_tuple(
          gm.getFrameId(),
          gm.getTimestamp(),
          layers,
          grids,
          gm.getBasicLayers(),
          gm.getLength(),
          gm.getResolution(),
          gm.getPosition(),
          gm.getStartIndex()
        );
      },
      [](const py::tuple& t) {
        // Equivalent to __setstate__, return an instance from tuple
        if(t.size()!=9) {
          throw std::runtime_error("Invalid pickled state!");
        }

        GridMap gm;
        gm.setFrameId(t[0].cast<std::string>());
        gm.setTimestamp(t[1].cast<Time>());
        gm.setGeometry(t[5].cast<Length>(), t[6].cast<double>(), t[7].cast<Position>());
        gm.setStartIndex(t[8].cast<Index>());
        const auto layers = t[2].cast<std::vector<std::string>>();
        const auto grids = t[3].cast<std::vector<GridMap::Matrix>>();
        for(size_t i=0; i<layers.size(); i++) gm.add(layers[i], grids[i]);
        gm.setBasicLayers(t[4].cast<std::vector<std::string>>());
        return gm;
      }
    ));
}
