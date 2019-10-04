#include <learning/learning.hpp>

double learning::UnwrapAngle(double angle_old, double angle_new)
{
    double threshold = 0.1;
    
    if((std::fabs((std::fabs(angle_old) - M_PI)) < threshold) && (std::signbit(angle_old * angle_new)))
    {
        if(std::signbit(angle_old))
        {
            angle_new = -2*M_PI + std::fabs(angle_new);
        }
        else
        {
            angle_new = +2*M_PI - std::fabs(angle_new);
        }
    }
    return angle_new;
};

double learning::GramianCalc(double qnew, double dqnew,double qold, double dqold)
{
    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 1> xold;
    double u;
    
    qnew = UnwrapAngle(qold,qnew);
    
    x << qnew, dqnew;
    xold << qold, dqold;
    u = Y * (xold - x);    
    return u;
};

Kuka_Vec learning::DatasetCreation(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix)
{
    Kuka_Vec  acc;
    Kuka_Vec Yk;

    for(int i=0; i < NUMBER_OF_JOINTS; i++)
    {
        acc(i) = GramianCalc(State(i),State(i+NUMBER_OF_JOINTS),OldState(i),OldState(i+NUMBER_OF_JOINTS));
    }
    
    Yk = MassMatrix * (reference - acc);
    Yk = Yk + prediction;

    return Yk;
};