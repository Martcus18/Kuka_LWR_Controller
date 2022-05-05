#ifndef KUKA_UTILITIES
#define KUKA_UTILITIES

//Joints limits in Radians
#define QL1 3.14
#define QL2 3.14
#define QL3 3.14
#define QL4 3.14
#define QL5 1.74
#define QL6 1.74
#define QL7 1.74

//Joints velocities limits in Radians/s
#define VL1 0.5
#define VL2 0.5
#define VL3 0.5
#define VL4 0.5
#define VL5 0.5
#define VL6 0.5
#define VL7 0.5


#define FILTER_LENGTH 7

#define FILTER_LENGTH_LEARNING 7

#define FILTER_LENGTH_PREDICTING 10


/*
#define FILTER_LENGTH 3

#define FILTER_LENGTH_LEARNING 3

#define FILTER_LENGTH_PREDICTING 10
*/

#include<controller/controller.hpp>

Eigen::Vector3d D_kin(Kuka_Vec q);

Eigen::MatrixXd Jacobian(Kuka_Vec q);

Eigen::MatrixXd diff_Jacobian(Kuka_Vec q, Kuka_Vec dq);

Eigen::MatrixXd diff_Jacobian2(Kuka_Vec q, Kuka_Vec qdot);

Kuka_Vec Filter2(std::vector<Kuka_Vec> &signal, int filter_length);

Eigen::VectorXd Filter3(std::vector<Eigen::VectorXd> &signal, int filter_length);

void dampedPseudoInverse(const Eigen::MatrixXd& A,double dampingFactor,double e,Eigen::MatrixXd& Apinv,unsigned int computationOptions);


#endif // KUKA_UTILITIES
