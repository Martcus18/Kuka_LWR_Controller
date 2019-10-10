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
#include <limbo/model/gp.hpp>
#include <limbo/model/sparsified_gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/model/gp/mean_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>

#include <limbo/serialize/text_archive.hpp>
#include <limbo/stop/max_iterations.hpp>
#include <chrono>

//#include<utils/lib.hpp>
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
    struct kernel : public defaults::kernel {
    };
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public defaults::opt_rprop {
    };
    //In case of sparse GP define the number of maximum points as parameters
    /*
    struct model_sparse_gp
    {
        BO_PARAM(int, max_points, 200);
    };
    */
};

using Kernel_t = kernel::SquaredExpARD<Params>;
using Mean_t = mean::Data<Params>;
//Using exact model 
using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;

/*
using Kernel_t = kernel::Exp<Params>;
using Mean_t = mean::Data<Params>;
using GP_t = model::GP<Params, Kernel_t, Mean_t>;
*/

//Using sparse model for accelerating the process
//Optimize hyperparameters for kernel matching
//using GP_t = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params>>;
//Optimize hyperparameters for mean matching
//using GP_t = model::SparsifiedGP<Params, Kernel_t, Mean_t, model::gp::MeanLFOpt<Params>>;

int main(int argc, char** argv)
{       

    std::string Xtrain = "X.txt";
    std::string Ytrain = "Y.txt";
    std::string Prediction_file = "gp_prediction.dat";
    data_manager dataX;
    data_manager dataY;
    
    data_manager dataX_opti;
    data_manager dataY_opti;

    std::vector<Eigen::VectorXd> X;
    std::vector<Eigen::VectorXd> Y;

    GP_t gp(21, 7);

    //Loading gp hyperparameters
    //gp.load<serialize::TextArchive>("myGP");

    //Specify Input and Output dimension
    dataX.read_data(Xtrain,X,21);

    dataY.read_data(Ytrain,Y,7);
    //std::cout << "I am here 4 \n";
    
    dataX.normalize_data(X);
    //dataX.de_normalize_data(X);
    dataY.normalize_data(Y);
    //dataY.de_normalize_data(Y);

    //std::vector<Eigen::VectorXd>::iterator first = X.begin();
    //std::vector<Eigen::VectorXd>::iterator last =  X.begin() + 800;
    //std::vector<Eigen::VectorXd> X_opti(first, last);
    //dataX.normalize_data(X_opti);

    //first = Y.begin();
    //last =  Y.begin() + 800;
    //std::vector<Eigen::VectorXd> Y_opti(first, last);
    //dataY.normalize_data(Y_opti);

    //gp.compute(X_opti, Y_opti, true);
    gp.compute(X, Y, true);
    //chrono::steady_clock sc;   // create an object of `steady_clock` class
    //auto start = sc.now(); 
    gp.optimize_hyperparams();
    //auto end = sc.now();
    //auto time_span = static_cast<chrono::duration<double>>(end - start);   // measure time span between start & end
    //std::cout<<"Operation took: "<<time_span.count()<<" seconds !!!" << "\n";
    
    // Sometimes is useful to save an optimized GP
    gp.save<serialize::TextArchive>("myGP");

    std::vector<Eigen::VectorXd> Prediction;
    Eigen::VectorXd mu;
    Eigen::VectorXd v;
    double sigma;
    
    for (int i = 1; i < 200; ++i) 
    {
        v = X[i];
        chrono::steady_clock sc;   // create an object of `steady_clock` class
        auto start = sc.now(); 
        std::tie(mu, sigma) = gp.query(v);
        auto end = sc.now();
        auto time_span = static_cast<chrono::duration<double>>(end - start);   // measure time span between start & end
        std::cout<<"Operation took: "<<time_span.count()<<" seconds !!!" << "\n";
        Prediction.push_back(mu);
    }
    //dataY.de_normalize_data(Prediction);
    
    //Writing limbo format of file
    dataY.write_data(Prediction_file,Prediction);
    
    return 0;
}
