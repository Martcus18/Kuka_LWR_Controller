#ifndef LEARNING_HPP_
#define LEARNING_HPP_

#include<utils/lib.hpp>
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


using namespace limbo;

class learning
{
    public:

    learning()
    {
        //TO BE FIXED
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GP_t gp(NUMBER_OF_JOINTS*3,NUMBER_OF_JOINTS);
        
        //gp.load<serialize::TextArchive>("myGP");
        
        gp_container.push_back(gp);

        Eigen::MatrixXd temp;
        Eigen::MatrixXd temp2;
        Eigen::MatrixXd temp3;
        Eigen::MatrixXd temp4;
    
        A << 1.0, DELTAT, 0.0, 1.0 ;
        B << std::pow(DELTAT,2) / 2.0 , DELTAT;

        temp = A * B;
        temp2 = B * B.transpose();
        temp3 = temp * temp.transpose();
        W = temp2 + temp3;
        temp4 = -(B.transpose() + temp.transpose());
        Y = temp4 * W.inverse();       
        
        Max_Vector = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);
        Min_Vector = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

    }

    ~learning(){};

    void DatasetUpdate(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix);
    Kuka_Vec DataPoint(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix);

    void GpUpdate();
    
    

    Kuka_Vec GpPredict(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q_ref);
    
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


    data_manager X_manager;
    data_manager Y_manager;

    std::vector<Eigen::VectorXd> DatasetY;
    std::vector<Eigen::VectorXd> DatasetX;

    Eigen::VectorXd Max_Vector;
    Eigen::VectorXd Min_Vector;

    protected:

    Eigen::Matrix2d A;
    Eigen::Matrix<double, 2, 1>  B;
    Eigen::Matrix2d W;
    Eigen::Matrix<double, 1, 2> Y;
    
    Eigen::VectorXd Normalize(Eigen::VectorXd Y);
    Eigen::VectorXd DeNormalize(Eigen::VectorXd YNormalized);
    
    double UnwrapAngle(double angle_old, double angle_new); 
    double GramianCalc(double qnew, double dqnew, double qold, double dqold);
};

#endif /*LEARNING_HPP_*/