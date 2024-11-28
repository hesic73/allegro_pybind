#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "consts.h"
#include <BHand/BHand.h>
#include "allegro_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(allegro_pybind, m)
{
     py::class_<AllegroInterface>(m, "AllegroInterface")
         .def(py::init<>()) // Bind the constructor
         .def("start", &AllegroInterface::start, "Start the Allegro Hand")
         .def("stop", &AllegroInterface::stop, "Stop the Allegro Hand")
         .def("set_joint_positions", &AllegroInterface::set_joint_positions,
              "Set the desired joint positions",
              py::arg("positions"), py::arg("use_delta") = false)
         .def("get_joint_positions", &AllegroInterface::get_joint_positions,
              "Get the current joint positions")
         .def("set_pd_gains", &AllegroInterface::set_pd_gains,
              "Set the PD gains", py::arg("kp"), py::arg("kd"))
         .def("set_motion_type", &AllegroInterface::set_motion_type,
              "Set the motion type", py::arg("motionType"));

     py::enum_<eMotionType>(m, "eMotionType")
         .value("NONE", eMotionType_NONE)
         .value("HOME", eMotionType_HOME)
         .value("READY", eMotionType_READY)
         .value("GRAVITY_COMP", eMotionType_GRAVITY_COMP)
         .value("PRE_SHAPE", eMotionType_PRE_SHAPE)
         .value("GRASP_3", eMotionType_GRASP_3)
         .value("GRASP_4", eMotionType_GRASP_4)
         .value("PINCH_IT", eMotionType_PINCH_IT)
         .value("PINCH_MT", eMotionType_PINCH_MT)
         .value("OBJECT_MOVING", eMotionType_OBJECT_MOVING)
         .value("ENVELOP", eMotionType_ENVELOP)
         .value("JOINT_PD", eMotionType_JOINT_PD)
         .value("MOVE_OBJ", eMotionType_MOVE_OBJ)
         .value("FINGERTIP_MOVING", eMotionType_FINGERTIP_MOVING)
         .export_values();
}
