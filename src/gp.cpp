//| Copyright Inria May 2015
//| This project has received funding from the European Research Council (ERC) under
//| the European Union's Horizon 2020 research and innovation programme (grant
//| agreement No 637972) - see http://www.resibots.eu
//|
//| Contributor(s):
//|   - Jean-Baptiste Mouret (jean-baptiste.mouret@inria.fr)
//|   - Antoine Cully (antoinecully@gmail.com)
//|   - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
//|   - Federico Allocati (fede.allocati@gmail.com)
//|   - Vaios Papaspyros (b.papaspyros@gmail.com)
//|   - Roberto Rama (bertoski@gmail.com)
//|
//| This software is a computer library whose purpose is to optimize continuous,
//| black-box functions. It mainly implements Gaussian processes and Bayesian
//| optimization.
//| Main repository: http://github.com/resibots/limbo
//| Documentation: http://www.resibots.eu/limbo
//|
//| This software is governed by the CeCILL-C license under French law and
//| abiding by the rules of distribution of free software.  You can  use,
//| modify and/ or redistribute the software under the terms of the CeCILL-C
//| license as circulated by CEA, CNRS and INRIA at the following URL
//| "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and  rights to copy,
//| modify and redistribute granted by the license, users are provided only
//| with a limited warranty  and the software's author,  the holder of the
//| economic rights,  and the successive licensors  have only  limited
//| liability.
//|
//| In this respect, the user's attention is drawn to the risks associated
//| with loading,  using,  modifying and/or developing or reproducing the
//| software by the user in light of its specific status of free software,
//| that may mean  that it is complicated to manipulate,  and  that  also
//| therefore means  that it is reserved for developers  and  experienced
//| professionals having in-depth computer knowledge. Users are therefore
//| encouraged to load and test the software's suitability as regards their
//| requirements in conditions enabling the security of their systems and/or
//| data to be ensured and,  more generally, to use and operate it in the
//| same conditions as regards security.
//|
//| The fact that you are presently reading this means that you have had
//| knowledge of the CeCILL-C license and that you accept its terms.
//|
#include <fstream>
#include <iostream>
#include <string.h>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include<limbo/mean/constant.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/multi_gp.hpp>
#include <limbo/model/multi_gp/parallel_lf_opt.hpp>
#include <limbo/model/sparsified_gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/model/gp/mean_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>

#include <limbo/serialize/text_archive.hpp>
#include <limbo/stop/max_iterations.hpp>
#include <chrono>

#include <utils/lib.hpp>
#include <utils/data_utils.hpp>

// this tutorials shows how to use a Gaussian process for regression

using namespace limbo;
using namespace std;
using namespace Eigen;
using namespace boost;

struct Params {
    
    struct kernel_exp
    {
            BO_PARAM(double, sigma_sq, 1.0);
            BO_PARAM(double, l, 0.2);
    };
    struct kernel_maternfivehalves 
    {
            BO_PARAM(double, sigma_sq, 1);
            BO_PARAM(double, l, 0.2);
    };

    struct kernel_squared_exp_ard
    {
        BO_PARAM(double,sigma_sq,1.0);
        BO_PARAM(int,k,0);
    };

    struct mean_constant
    {
        BO_PARAM(double,constant,0.0);
    };

    //struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {};
    struct kernel : public defaults::kernel {};
    struct opt_rprop : public defaults::opt_rprop {};
    //In case of sparse GP define the number of maximum points as parameters
    
    struct model_sparse_gp
    {   // FOR 5 ms
        //BO_PARAM(int, max_points, 150);
        // FOR 10 ms
        BO_PARAM(int, max_points, 200);
    };
    
};

//using Kernel_t = kernel::MaternFiveHalves<Params>;
using Kernel_t = kernel::SquaredExpARD<Params>;
//using Mean_t = mean::Data<Params>;
using Mean_t = mean::Constant<Params>;
//Using exact model 
using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
//SPARSE GP
//using GP_t_sparse = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::MeanLFOpt<Params>>;
using GP_t_sparse = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
using MultiGP_t = model::MultiGP<Params, model::SparsifiedGP, Kernel_t, Mean_t, model::multi_gp::ParallelLFOpt<Params, model::gp::KernelLFOpt<Params>>>;


//Using sparse model for accelerating the process
//Optimize hyperparameters for kernel matching
//using GP_t = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
//Optimize hyperparameters for mean matching
//using GP_t = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::MeanLFOpt<Params>>;

int main(int argc, char** argv)
{       

    std::string Xtrain = "X.txt";
    std::string Ytrain = "Y.txt";
    //std::string Xtrain = "Xpatch.txt";
    //std::string Ytrain = "Ypatch.txt";    
    std::string Xtest = "Xtest.txt";
    std::string Ytest = "Ytest.txt";

    std::string Prediction_file = "gp_prediction.dat";
    std::string Ground_truth = "ground_truth.dat";
    data_manager dataX;
    data_manager dataY;
    data_manager dataXtest;
    data_manager dataYtest;


    std::vector<Eigen::VectorXd> X;
    std::vector<Eigen::VectorXd> Y;
    std::vector<Eigen::VectorXd> Xnew;
    std::vector<Eigen::VectorXd> Ynew;    
    std::vector<Eigen::VectorXd> Xtrain_vec;
    std::vector<Eigen::VectorXd> Ytrain_vec;
    std::vector<Eigen::VectorXd> Xshuffled;
    std::vector<Eigen::VectorXd> Yshuffled;


    //GP_t_sparse gp(21, 7);

    //MultiGP_t gp(12,4);

    MultiGP_t gp(21,7);

    //GP_t gp(3, 1);
    //GP_t gp(21, 7);

    //Loading gp hyperparameters
    //gp.load<serialize::TextArchive>("myGP");

    //Specify Input and Output dimension
    //dataX.read_data(Xtrain,X,12);
    dataX.read_data(Xtrain,X,21);
    //dataY.read_data(Ytrain,Y,4);
    dataY.read_data(Ytrain,Y,7);
    //dataXtest.read_data(Xtest,Xnew,21);
    //dataYtest.read_data(Ytest,Ynew,7);

    Xshuffled = X;
    Yshuffled = Y;

    //Xshuffled = Xnew;
    //Yshuffled = Ynew;    

    /*
    auto seed = unsigned ( std::time(0) );
    std::srand ( seed );
    std::random_shuffle ( Xshuffled.begin(), Xshuffled.end() );

    std::srand ( seed );
    std::random_shuffle ( Yshuffled.begin(), Yshuffled.end() );

    auto startx = Xshuffled.begin(); 
    auto endx = Xshuffled.begin() + 101; 

    auto starty = Yshuffled.begin();
    auto endy = Yshuffled.begin() + 101; 

    std::copy(starty, endy,std::back_inserter(Ytrain_vec));
    std::copy(startx, endx,std::back_inserter(Xtrain_vec));
    */

    gp.compute(X, Y, false);    
    
    gp.optimize_hyperparams();

    gp.save<serialize::TextArchive>("myGP"); 
    
    /*
    
    std::vector<Eigen::VectorXd> Prediction;

    Eigen::VectorXd mu;

    Eigen::VectorXd v;

    std::vector<Eigen::VectorXd> Ground;
    
    //double sigma;

    Eigen::VectorXd sigma;
    
    std::cout << "I am here 1" << "\n";
    
    for (int i = 0; i <490 ; i++){
    //for (int i = 1; i <Xshuffled.size()-1 ; ++i)

        v = Xshuffled[i];

        Ground.push_back(Yshuffled[i]);

        //auto start = sc.now(); 
        
        std::tie(mu, sigma) = gp.query(v);

        Prediction.push_back(mu);
    }

    //dataY.de_normalize_data(Prediction);
    
    std::cout << "I am here 2" << "\n";
    
    //Writing limbo format of file
    
    dataY.write_data(Prediction_file,Prediction);
    
    std::cout << "I am here 3" << "\n";
    
    dataY.write_data(Ground_truth,Ground);
    
    std::cout << "I am here 4" << "\n";
    */  
    return 0;
}
