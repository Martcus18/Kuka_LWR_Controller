#ifndef KUKA_UTILITIES
#define KUKA_UTILITIES

#include<controller/controller.hpp>

Eigen::Vector3d D_kin(Kuka_Vec q);

Eigen::MatrixXd Jacobian(Kuka_Vec q);

Eigen::MatrixXd diff_Jacobian(Kuka_Vec q, Kuka_Vec dq);

Eigen::MatrixXd diff_Jacobian2(Kuka_Vec q, Kuka_Vec qdot);

/*
template <typename Derived1, typename Derived2>
void dampedPseudoInverse(const Eigen::MatrixBase<Derived1>& A,double dampingFactor,double e,Eigen::MatrixBase<Derived2>& Apinv,unsigned int computationOptions);
*/
void dampedPseudoInverse(const Eigen::MatrixXd& A,double dampingFactor,double e,Eigen::MatrixXd& Apinv,unsigned int computationOptions);
Eigen::MatrixXd compute_Damped_pinv(Eigen::MatrixXd j, double f, double e);

//MatrixXd compute_Weighted_Damped_pinv(MatrixXd j,MatrixXd Q);

#endif // KUKA_UTILITIES
