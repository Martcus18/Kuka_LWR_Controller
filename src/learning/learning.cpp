#include <learning/learning.hpp>

double learning::unwrap_angle(double angle_old, double angle_new)
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

double learning::gramian_calc(double qnew, double dqnew,double qold, double dqold)
{
    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 1> xold;
    double u;
    qnew = unwrap_angle(qold,qnew);
    x << qnew, dqnew;
    xold << qold, dqold;
    u = Y * (xold - x);
    return u;
};