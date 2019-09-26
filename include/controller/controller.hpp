#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_
#if defined(_USE_MATH_DEFINES) && !defined(_MATH_DEFINES_DEFINED)
#define _MATH_DEFINES_DEFINED

/* Define _USE_MATH_DEFINES before including math.h to expose these macro
 * definitions for common math constants.  These are placed under an #ifdef
 * since these commonly-defined names are not part of the C/C++ standards.
 */

/* Definitions of useful mathematical constants
 * M_E        - e
 * M_LOG2E    - log2(e)
 * M_LOG10E   - log10(e)
 * M_LN2      - ln(2)
 * M_LN10     - ln(10)
 * M_PI       - pi
 * M_PI_2     - pi/2
 * M_PI_4     - pi/4
 * M_1_PI     - 1/pi
 * M_2_PI     - 2/pi
 * M_2_SQRTPI - 2/sqrt(pi)
 * M_SQRT2    - sqrt(2)
 * M_SQRT1_2  - 1/sqrt(2)
 */

#define M_E        2.71828182845904523536
#define M_LOG2E    1.44269504088896340736
#define M_LOG10E   0.434294481903251827651
#define M_LN2      0.693147180559945309417
#define M_LN10     2.30258509299404568402
#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616
#define M_1_PI     0.318309886183790671538
#define M_2_PI     0.636619772367581343076
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2    1.41421356237309504880
#define M_SQRT1_2  0.707106781186547524401

#endif  /* _USE_MATH_DEFINES */

#define RUN_TIME_IN_SECONDS             300.0
#define DELTAT                  0.005


#ifndef RAD
#define RAD(A)  ((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)  ((A) * 180.0 / PI )
#endif

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS                        7
#endif



#include <Eigen/Dense>
#include<fstream>
#include <iostream>
#include <string.h>
#include <pthread.h>
#include <chrono>
#include <utils/data_utils.hpp>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/sparsified_gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/model/gp/mean_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/text_archive.hpp>
#include <limbo/stop/max_iterations.hpp>

typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , 1> Kuka_Vec;
typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , NUMBER_OF_JOINTS> Kuka_Mat;

using namespace limbo;

class controller
{
    public:
    
        controller(){};
        ~controller(){};
        controller(bool Flag)
        {
                data_manager writer();
                Kp.diagonal() << 50,50,50,50,50,50,50;
                Kd.diagonal() << 1,1,1,1,1,1,1;
                
                for(int i=0;i<NUMBER_OF_JOINTS;i++)
                {
                    GP_t temp_gp(NUMBER_OF_JOINTS*3,1);
                    gp_container.push_back(temp_gp);
                }
                Eigen::MatrixXd temp;
                Eigen::MatrixXd temp2;
                Eigen::MatrixXd temp3;
                Eigen::MatrixXd temp4;
    
                temp = A * B;
                temp2 = B * B.transpose();
                temp3 = temp * temp.transpose();
                W = temp2 + temp3;
                temp4 = -(B.transpose() + temp.transpose());
                Y = temp4 * W.inverse();
        };

        //Feedback Linearization VIRTUAL

        virtual Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference) = 0;

        //PD Controller VIRTUAL

        virtual Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd) = 0;

        //Communication VIRTUAL
        
        virtual void SetTorques(Kuka_Vec torques) = 0;

        //Communication VIRTUAL
        
        virtual Eigen::VectorXd GetState() = 0;
        
        double Ts;

        //In order to be generic with respect to the number of DOF of the manipulator

        std::vector<Kuka_Vec> Qsave;
        std::vector<Kuka_Vec> dQsave;
        std::vector<Kuka_Vec> d2Qsave;

        std::vector<Kuka_Vec> Qsave_filtered;
        std::vector<Kuka_Vec> dQsave_filtered;
        std::vector<Kuka_Vec> d2Qsave_filtered;

        std::vector<Kuka_Vec> Tor_th;
        std::vector<Kuka_Vec> Tor_meas;
        
        data_manager writer;
        
        data_manager X_manager;
        data_manager Y_manager;

        std::vector<Eigen::VectorXd> DatasetY;
        std::vector<Eigen::VectorXd> DatasetX;

        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kp;
        Eigen::DiagonalMatrix<double, NUMBER_OF_JOINTS> Kd;
        
        //GP parameters
        struct Params 
        {
            struct kernel_exp 
            {
                BO_PARAM(double, sigma_sq, 1.0);
                BO_PARAM(double, l, 0.2);
            };
            struct kernel : public defaults::kernel {};
            struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {};
            struct opt_rprop : public defaults::opt_rprop {};    
        };
        using Kernel_t = kernel::SquaredExpARD<Params>;
        using Mean_t = mean::Data<Params>;
        using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
        std::vector<GP_t > gp_container;
    protected:

        Eigen::Matrix2d A;
        Eigen::Matrix<double, 2, 1>  B;
        Eigen::Matrix2d W;
        Eigen::Matrix<double, 1, 2> Y;
        double unwrap_angle(double angle_old, double angle_new);
};
#endif /* CONTROLLER_HPP_ */