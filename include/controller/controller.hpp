#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include<learning/learning.hpp>

#include<tensorflow/c/tf_network.hpp>

#include <FastResearchInterface.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>
#include <TypeIRML.h>
#include <LWR_Dynamic_Model_Lib.h>
#include <controller/kuka_utilities.h>

class controller
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        controller()
        {
                data_manager writer();
                Kp.diagonal() << 55,55,55,55,55,55,55;
                //Kp.diagonal() << 75,75,75,75,75,75,75;
                //Kd.diagonal() << 8,8,8,8,8,8,8;
                Kd.diagonal() << 15,15,15,15,15,15,15;
                //Ki.diagonal() << 3,3,3,3,3,3,3;
                Ki.diagonal() << 0,0,0,0,0,0,0;
        };

        ~controller(){};

        //Feedback Linearization VIRTUAL

        virtual Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference) = 0;

        //PD Controller VIRTUAL

        virtual Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd) = 0;

        //Communication VIRTUAL
        
        virtual void SetTorques(Kuka_Vec torques) = 0;
        
        double Ts;

        //In order to be generic with respect to the number of DOF of the manipulator

        /*
        std::vector<Kuka_Vec> Qsave;
        std::vector<Kuka_Vec> dQsave;
        std::vector<Kuka_Vec> d2Qsave;

        std::vector<Kuka_Vec> Qsave_filtered;
        std::vector<Kuka_Vec> dQsave_filtered;
        std::vector<Kuka_Vec> d2Qsave_filtered;
        std::vector<Kuka_Vec> foo;
        std::vector<Kuka_Vec> foo2;
        std::vector<Eigen::VectorXd> fooXd;

        std::vector<Eigen::VectorXd> end_eff_pos;
        

        std::vector<Kuka_Vec> Tor_th;
        std::vector<Kuka_Vec> Tor_meas;
        std::vector<Kuka_Vec> Tor_meas_filtered;
        */
        
        VectorKukaVec Qsave;
        VectorKukaVec dQsave;
        VectorKukaVec d2Qsave;

        VectorKukaVec Qsave_filtered;
        VectorKukaVec dQsave_filtered;
        VectorKukaVec d2Qsave_filtered;
        VectorKukaVec foo;
        VectorKukaVec foo2;

        std::vector<Eigen::VectorXd> fooXd;

        std::vector<Eigen::VectorXd> end_eff_pos;
        
        VectorKukaVec Tor_th;
        VectorKukaVec Tor_meas;
        VectorKukaVec Tor_meas_filtered;

        data_manager writer;

        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Ki;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp_torque;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd_torque;

};
#endif /* CONTROLLER_HPP_ */
