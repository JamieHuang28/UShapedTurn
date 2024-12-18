#include <cstring>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <vector>
#include "u_shaped_turn.h"

namespace py = pybind11;

py::array_t<double> py_drive(const py::array_t<double>& pts, Eigen::Ref<Eigen::Vector3d> start_pose) {
	py::buffer_info buf = pts.request();
	
	if (buf.ndim != 2) {
		throw std::runtime_error("Number of dimensions must be two");
	}
	
	if (buf.shape[1] != 2) {
		throw std::runtime_error("Second dimension's size must be two");
	}

	double *ptr = static_cast<double *>(buf.ptr);

	std::vector<double> ch = drive({ptr, ptr+buf.size}, start_pose);

	py::buffer_info resbuf = py::buffer_info(
		ch.data(),
		buf.itemsize,
		buf.format,
		2,
		{(size_t)(ch.size() >> 1), (size_t)2},
		{sizeof(double)*2, sizeof(double)}
		);
	return py::array_t<double>(resbuf);
}

PYBIND11_MODULE(u_shaped_turn, m) {
	m.def("drive", &py_drive, "drive vehicle model for test",
	      py::arg("pts"), py::arg("start_pose"));
}