#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

//#include<learning/learning.hpp>

//#include<tensorflow/c/tf_network.hpp>

#include <FastResearchInterface.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>
#include <cmath>
#include <TypeIRML.h>
#include <LWR_Dynamic_Model_Lib.h>
#include <controller/kuka_utilities.h>

class controller
{
    public:
        
        controller()
        {
                data_manager writer();

                // FOR REAL ROBOT
                //Kp.diagonal() << 500,500,500,500,500,500,500;
                //Kd.diagonal() << 15,15,15,15,15,15,15;
                
                Kp.diagonal() << 105,105,105,105,105,105,105;
                Kd.diagonal() << 5,5,5,5,5,5,5;                


                //FOR SIMULATION
                //Kp.diagonal() << 10,10,10,10,10,10,10;
                //Kd.diagonal() << 1,1,1,1,1,1,1;

                Ki.diagonal() << 0,0,0,0,0,0,0;

                //CARTESIAN PD

                Kp_cart.diagonal() << 100,100,400;
                Kd_cart.diagonal() << 10, 10, 50;

                //DAMPING

                K_damp.diagonal() << 200,200,200,200,200,200,200;

                //IDENTITY MATRIX OF DIMENSION 7x7
                eye << 1,0,0,0,0,0,0,
                        0,1,0,0,0,0,0,
                        0,0,1,0,0,0,0,
                        0,0,0,1,0,0,0,
                        0,0,0,0,1,0,0,
                        0,0,0,0,0,1,0,
                        0,0,0,0,0,0,1;

                //GAIN OF THE RESIDUAL
                K << 50,0,0,0,0,0,0,
                        0,50,0,0,0,0,0,
                        0,0,50,0,0,0,0,
                        0,0,0,50,0,0,0,
                        0,0,0,0,50,0,0,
                        0,0,0,0,0,50,0,
                        0,0,0,0,0,0,50;

                //LINEAR GAIN FOR THE REDUCED OBSERVER
                
                k0 = 10;

                //RESIDUAL THRESHOLDS (SIMULATION): these are defined accordingly to the values the torque assume in nominal condition  

                //th << 0.1,28.70,0.18,3.48,0.08,0.07,0.001; //without noise

                th << 0.75,1.5,0.4,1.5,0.04,0.02,0.0015; //with noise

                //RESIDUAL THRESHOLDS (REAL ROBOT)

                

        };

        ~controller(){};

        //Feedback Linearization VIRTUAL

        virtual Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference) = 0;

        //PD Controller VIRTUAL

        virtual Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd) = 0;

        //Communication VIRTUAL
        
        virtual void SetTorques(Kuka_Vec torques) = 0;
        
        double Ts;

        double k0;

        //In order to be generic with respect to the number of DOF of the manipulator

        std::vector<Kuka_Vec> Qsave;
        std::vector<Kuka_Vec> dQsave;
        std::vector<Kuka_Vec> d2Qsave;

        std::vector<Kuka_Vec> Qsave_meas;
        std::vector<Kuka_Vec> dQsave_meas;
        std::vector<Kuka_Vec> d2Qsave_meas;

        std::vector<Kuka_Vec> dQ_hat_save;
        std::vector<Kuka_Vec> dQ_num_save;

        std::vector<Kuka_Vec> r_save;
        std::vector<Kuka_Vec> r_obs_save;
        std::vector<Kuka_Vec> r_filtered_save;

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
        
        data_manager writer;

        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Ki;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp_torque;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd_torque;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> K_damp;

        Kuka_Mat eye;
        Kuka_Mat K;

        Eigen::DiagonalMatrix<double, 3> Kp_cart;
        Eigen::DiagonalMatrix<double, 3> Kd_cart;

        Kuka_Vec th;

};
#endif /* CONTROLLER_HPP_ */
