#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <utils/lib.hpp>
#include <utils/data_utils.hpp>
#include <FastResearchInterface.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>
#include <TypeIRML.h>
#include <LWR_Dynamic_Model_Lib.h>
#include <TypeIRML.h>
#include <controller/kuka_utilities.h>

class controller
{
    public:
        
        controller()
        {
                data_manager writer();
                Kp.diagonal() << 50,50,50,50,50,50,50;
                Kd.diagonal() << 1,1,1,1,1,1,1;
        };

        ~controller(){};

        //Feedback Linearization VIRTUAL

        virtual Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference) = 0;

        //PD Controller VIRTUAL

        virtual Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd) = 0;

        //Communication VIRTUAL
        
        virtual void SetTorques(Kuka_Vec torques) = 0;

        //Communication VIRTUAL
        
        virtual Kuka_State GetState() = 0;
        
        double Ts;

        //In order to be generic with respect to the number of DOF of the manipulator

        std::vector<Kuka_Vec> Qsave;
        std::vector<Kuka_Vec> dQsave;
        std::vector<Kuka_Vec> d2Qsave;

        std::vector<Kuka_Vec> Qsave_filtered;
        std::vector<Kuka_Vec> dQsave_filtered;
        std::vector<Kuka_Vec> d2Qsave_filtered;

        std::vector<Eigen::VectorXd> end_eff_pos;

        std::vector<Kuka_Vec> Tor_th;
        std::vector<Kuka_Vec> Tor_meas;
        
        data_manager writer;
        
        data_manager X_manager;
        data_manager Y_manager;

        std::vector<Eigen::VectorXd> DatasetY;
        std::vector<Eigen::VectorXd> DatasetX;

        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd;

};
#endif /* CONTROLLER_HPP_ */