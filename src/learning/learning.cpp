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

Kuka_Vec learning::DataPoint(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix)
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

void learning::DatasetUpdate(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix)
{
    Eigen::VectorXd Y;
    Eigen::VectorXd X(NUMBER_OF_JOINTS*3);
    
    Y = this->DataPoint(State, OldState, reference, prediction, MassMatrix);
    this->DatasetY.push_back(Y);

    X << OldState,reference;
    this->DatasetX.push_back(X);
};

void learning::GpUpdate()
{
    int i;
    Eigen::VectorXd normalized_Y;

    //FOR 7 Gps
    /*
    Eigen::VectorXd tempX(3);
    Eigen::VectorXd tempY(1);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        tempX << DatasetX.back()(i),DatasetX.back()(i+8), DatasetX.back()(i+14);
        tempY << DatasetY.back()(i);
        gp_container[i].add_sample(tempX,tempY);
    }
    */

   //FOR 1 Multidimensional GP
   //gp_container.back().add_sample(DatasetX.back(),DatasetY.back());
   gp_container.back().add_sample(DatasetX.back(),Normalize(DatasetY.back()));

};

Kuka_Vec learning::GpPredict(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q_ref)
{
    Kuka_Vec prediction(NUMBER_OF_JOINTS);
    
    int i;
    double sigma;
    Eigen::VectorXd mu;
    /*
    //FOR 7 Gps
    Eigen::VectorXd query_point(3);
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        query_point << Q(i),dQ(i),d2Q_ref(i);
        std::tie(mu, sigma) = gp_container[i].query(query_point);
        //prediction(i) = gp_container[i].query(query_point);
        prediction(i) = mu(0);
    }
    */

   //FOR 1 GP multidimensional
   Eigen::VectorXd query_point(NUMBER_OF_JOINTS*3);
   query_point << Q,dQ,d2Q_ref;
   std::tie(mu,sigma) = gp_container.back().query(query_point);
   prediction = mu;

   prediction = DeNormalize(prediction);
   
   return prediction;
};

Eigen::VectorXd learning::Normalize(Eigen::VectorXd Y)
{
    int i;
    Kuka_Vec Normalized;

    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        Normalized(i) = (Y(i) - MIN_GP) / (2.0*(MAX_GP-MIN_GP));
    }
    return Normalized;
};

Eigen::VectorXd learning::DeNormalize(Eigen::VectorXd YNormalized)
{
    int i;
    Kuka_Vec DeNormalized;
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        DeNormalized(i) = YNormalized(i) * (2.0*(MAX_GP-MIN_GP)) + MIN_GP;
    }
    return DeNormalized;
};