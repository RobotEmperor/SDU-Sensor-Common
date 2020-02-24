#include "ft_filter/ft_filter.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(ft_filter, m)
{
m.doc() = R"pbdoc(
        UR10e Force torque sensor filtering and the signal process library.
        -----------------------
        .. currentmodule:: ft_filter
        .. autosummary::
           :toctree: _generate
           parse_init_data
           initialize
           offset_init
           filter_processing
           collision_detection
           get_filtered_data
           get_offset_data
           get_collision_detection_data

    )pbdoc";

py::class_<FTfilter>(m, "FTfilter")
.def(py::init<>())
.def("parse_init_data", &FTfilter::parse_init_data, R"pbdoc(To load sensor filter gains, you can insert the path of init_data.yaml.)pbdoc")
.def("initialize", &FTfilter::initialize, R"pbdoc(Variables are initialized in this function.)pbdoc")
.def("offset_init", &FTfilter::offset_init, R"pbdoc(This function is to get offest value by sampling raw data of FT sensor for a certain period of time.)pbdoc")
.def("filter_processing", &FTfilter::filter_processing, R"pbdoc(Signal processing using a kalman filter.)pbdoc")
.def("collision_detection", &FTfilter::collision_detection_processing, R"pbdoc(This function can detect collision from raw force torque data by using CUSUM method. It returns int value 1 and -1 when collision is detected. Value 0 is default and non-contact)pbdoc")
.def("get_filtered_data", &FTfilter::get_filtered_data, R"pbdoc(Returns filtered force torque data.)pbdoc")
.def("get_offset_data", &FTfilter::get_offset_data, R"pbdoc(Returns offset data.)pbdoc")
.def("get_collision_detection_data", &FTfilter::get_collision_detection_data, R"pbdoc(Returns collision detection data.)pbdoc")

.def("__repr__", [](const FTfilter &a) { return "<FTfilter>"; });

#ifdef VERSION_INFO
m.attr("__version__") = VERSION_INFO;
#else
m.attr("__version__") = "dev";
#endif
}