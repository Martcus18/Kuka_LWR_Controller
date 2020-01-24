#ifndef KUKA_UTILITIES
#define KUKA_UTILITIES

//Joints limits in Radians
#define QL1 3.14
#define QL2 3.14
#define QL3 3.14
#define QL4 3.14
#define QL5 3.14
#define QL6 3.14
#define QL7 3.14

//Joints velocities limits in Radians/s
#define VL1 1.0
#define VL2 1.0
#define VL3 1.0
#define VL4 1.0
#define VL5 1.0
#define VL6 1.0
#define VL7 1.0

#include<controller/controller.hpp>

Eigen::Vector3d D_kin(Kuka_Vec q);

Eigen::MatrixXd Jacobian(Kuka_Vec q);

Eigen::MatrixXd diff_Jacobian(Kuka_Vec q, Kuka_Vec dq);

Eigen::MatrixXd diff_Jacobian2(Kuka_Vec q, Kuka_Vec qdot);

#endif // KUKA_UTILITIES
