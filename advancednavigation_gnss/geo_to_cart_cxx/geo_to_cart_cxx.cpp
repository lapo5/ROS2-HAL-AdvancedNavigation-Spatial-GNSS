#include <chrono>
#include <iostream>
#include <numeric>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "xtensor/xmath.hpp"              // xtensor import for the C++ universal functions

#include "modules/GeotedicToLocalCartesian.hpp"

namespace py = pybind11;

class Py_GeoToCart{
private:
	GeotedicToLocalCartesian* geoToCart;

public:
	Py_GeoToCart(py::array_t<double, py::array::c_style | py::array::forcecast> geotedic_pose, 
			py::array_t<double, py::array::c_style | py::array::forcecast> goetedic_confidences, 
			py::array_t<double, py::array::c_style | py::array::forcecast> cartesian_pose, 
			py::array_t<double, py::array::c_style | py::array::forcecast> cartesian_confidences){

		geoToCart = new GeotedicToLocalCartesian(geotedic_pose, goetedic_confidences, cartesian_pose, cartesian_confidences);
	}

	void compute(){
		geoToCart->compute();
	}

	void reset(){
		geoToCart->reset();
	}

	double get_starting_lat(){
		return geoToCart->start_lat;
	}

	double get_starting_lon(){
		return geoToCart->start_lon;
	}

	double get_starting_alt(){
		return geoToCart->start_alt;
	}
};

PYBIND11_MODULE(geo_to_cart_cxx, m) {
	py::class_<Py_GeoToCart>(m, "geo_to_cart")
		.def(py::init<
			py::array_t<double, py::array::c_style | py::array::forcecast>,
            py::array_t<double, py::array::c_style | py::array::forcecast>, 
            py::array_t<double, py::array::c_style | py::array::forcecast>, 
            py::array_t<double, py::array::c_style | py::array::forcecast>
			>())
        .def("compute", &Py_GeoToCart::compute)
        .def("get_starting_lat", &Py_GeoToCart::get_starting_lat)
        .def("get_starting_lon", &Py_GeoToCart::get_starting_lon)
        .def("get_starting_alt", &Py_GeoToCart::get_starting_alt)
        .def("reset", &Py_GeoToCart::reset);
}
