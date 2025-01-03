#include <cstring>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <vector>
#include "u_shaped_turn.h"
#include "hybrid_astar_algorithm/include/planner.h"
#include "planner.h"
#include "drive_path.h"

namespace py = pybind11;

py::array_t<double> py_drive(const py::array_t<double>& pts, Eigen::Ref<Eigen::Vector3d> start_pose) {
	py::buffer_info buf = pts.request();
	
	// if (buf.ndim != 2) {
	// 	throw std::runtime_error("Number of dimensions must be two");
	// }
	
	// if (buf.shape[1] != 2) {
	// 	throw std::runtime_error("Second dimension's size must be two");
	// }
	std::cout << "buf.ndim: " << buf.ndim << ", buf.shape[1]:" << buf.shape[1] << std::endl;

	double *ptr = static_cast<double *>(buf.ptr);

	std::vector<double> ch = drive({ptr, ptr+buf.size}, start_pose);

	py::buffer_info resbuf = py::buffer_info(
		ch.data(),
		ch.size(),
		"d",
		1,
		{(size_t)(ch.size())},
		{sizeof(double)}
		);
	return py::array_t<double>(resbuf);
}

py::array_t<double> planHybridAStar(double width, double height, py::array_t<double>& start_values, py::array_t<double>& end_values) {
	py::buffer_info buf = start_values.request();
	if (buf.ndim != 1) {
		throw std::runtime_error("Number of dimensions must be 1");
	}
	if (buf.shape[0] != 5) {
		throw std::runtime_error("Number of elements must be 5");
	}
	double *start_values_ptr = static_cast<double *>(buf.ptr);

	buf = end_values.request();
	if (buf.ndim != 1) {
		throw std::runtime_error("Number of dimensions must be 1");
	}
	if (buf.shape[0] != 5) {
		throw std::runtime_error("Number of elements must be 5");
	}
	double *end_values_ptr = static_cast<double *>(buf.ptr);


	using namespace HybridAStar;

	Planner planner;
    planner.initializeLookups();
    // planner.setMap(?);
    int depth = Constants::headings;
    Node3D nStart(start_values_ptr[0], start_values_ptr[1], start_values_ptr[2], start_values_ptr[3], start_values_ptr[4], nullptr);
    Node3D nGoal(end_values_ptr[0], end_values_ptr[1], end_values_ptr[2], end_values_ptr[3], end_values_ptr[4], nullptr);
    std::vector<Node3D> path_node3d;
    std::vector<Node3D> smoothed_path_node3d;
    planner.plan(width, height, depth, nStart, nGoal, path_node3d, smoothed_path_node3d);

	printf("path:\n");
    for (const Node3D& node : path_node3d) {
        printf("%f, %f, %f\n", node.getX(), node.getY(), node.getT());
    }
    printf("smoothed path:\n");
    for (const Node3D& node : smoothed_path_node3d) {
        printf("%f, %f, %f\n", node.getX(), node.getY(), node.getT());
    }
	
	std::vector<Eigen::Vector3d> res_path(smoothed_path_node3d.size());
	std::transform(smoothed_path_node3d.begin(), smoothed_path_node3d.end(), res_path.begin(),
		[](const Node3D& node) {
			return Eigen::Vector3d({node.getX(), node.getY(), node.getT()});
		}
	);

	py::buffer_info resbuf = py::buffer_info(
		res_path.data(),
		res_path.size(),
		"d",
		2,
		{(size_t)(res_path.size()), (size_t)3},
		{sizeof(double) * 3, sizeof(double)}
		);
	return py::array_t<double>(resbuf);
}

py::array_t<double> planUShapedTurn(py::array_t<double> start_values, py::array_t<double> end_values) {
	py::buffer_info buf = start_values.request();
	if (buf.ndim != 1) {
		throw std::runtime_error("Number of dimensions must be 1");
	}
	if (buf.shape[0] != 3) {
		throw std::runtime_error("Number of elements must be 3");
	}
	double *start_values_ptr = static_cast<double *>(buf.ptr);

	buf = end_values.request();
	if (buf.ndim != 1) {
		throw std::runtime_error("Number of dimensions must be 1");
	}
	if (buf.shape[0] != 3) {
		throw std::runtime_error("Number of elements must be 3");
	}
	double *end_values_ptr = static_cast<double *>(buf.ptr);

	Eigen::Vector3d start_pose(start_values_ptr[0], start_values_ptr[1], start_values_ptr[2]);
	Eigen::Vector3d end_pose(end_values_ptr[0], end_values_ptr[1], end_values_ptr[2]);

	u_shaped_turn::Planner planner;
	u_shaped_turn::DrivePath drive_path;

	std::vector<Eigen::Vector3d> res_path = planner.plan(start_pose, end_pose, drive_path);

	py::buffer_info resbuf = py::buffer_info(
		res_path.data(),
		res_path.size(),
		"d",
		2,
		{(size_t)(res_path.size()), (size_t)3},
		{sizeof(double) * 3, sizeof(double)}
		);
	return py::array_t<double>(resbuf);
}

PYBIND11_MODULE(u_shaped_turn, m) {
	m.def("drive", &py_drive, "drive vehicle model for test",
		py::arg("pts"), py::arg("start_pose"));
	m.def("planHybridAStar", &planHybridAStar, "plan with HybridAStar",
		py::arg("width"), py::arg("height"), py::arg("start"), py::arg("end")
	);
	m.def("planUShapedTurn", &planUShapedTurn, "plan with UShapedTurn",
		py::arg("start"), py::arg("end")
	);
}