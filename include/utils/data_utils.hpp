#include<fstream>
#include <iostream>
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS                        7
#endif

typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , 1> Kuka_Vec;

class data_manager
{
    public:
    
    //Class constructors

    data_manager();

    ~data_manager();

    //Read Data for limbo  GP -> <Xtrain,Ytrain>

    void read_data(std::string &filename, std::vector<Kuka_Vec> &container,int width);

    //Write Data for limbo GP -> <Prediction,Variance>

    void write_data(std::string &filename, std::vector<Kuka_Vec> &container);

    //Conversion from Limbo input to Eigen Matrix

    Eigen::MatrixXd Limbo_to_Eigen(std::vector<Kuka_Vec> &Dataset);

    //Conversion from Eigen Matrix to Limbo input
    std::vector<Kuka_Vec> Eigen_to_Limbo(Eigen::MatrixXd &Dataset);

    //Normalize Dataset

    void normalize_data(std::vector<Kuka_Vec> &Dataset);
    
    //DeNormalize Dataset

    void de_normalize_data(std::vector<Kuka_Vec> &Dataset);
        
    Eigen::VectorXd min_values;
    Eigen::VectorXd max_values;
    Eigen::VectorXd Delta;
};