#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// Forward declaration
void init_core(pybind11::module m);

PYBIND11_MODULE(grid_map_python, m) {
  init_core(m);
}
