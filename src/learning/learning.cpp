#include <learning/learning.hpp>

void learning::InitializeGp(std::string &XFile,std::string &YFile)
{

        std::vector<Eigen::VectorXd> Y;
        std::vector<Eigen::VectorXd> X;
        std::chrono::time_point<std::chrono::system_clock> Tic, Toc;
        double elapsed_time;

        if(!(this->X_manager.read_data(XFile,X,DIMX)))
        {
                std::cout << "Not present a X dataset with that name for the GP \n";
                return;
        }
        
        if(!(this->Y_manager.read_data(YFile,Y,DIMY)))
        {
                std::cout << "Not present a Y dataset with that name for the GP \n";
                return;
        }
        
        this->DatasetY = Y;
        this->DatasetX = X;
        
        std::cout << "Loading GP Optimized hyperparameters \n";
        
        this->gp->load<serialize::TextArchive>("myGP");

        //std::cout << "Computing the kernel for the datasets \n";
        Tic = std::chrono::system_clock::now();
        //this->gp->compute(this->DatasetX,this->DatasetY,true);
        //this->gp->compute(this->DatasetX,this->DatasetY,false);

        Toc = std::chrono::system_clock::now();

        elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Toc - Tic).count();
        std::cout << "elapsed = " << elapsed_time << "\n";

        //std::cout << "Optimizing the GP hyperparameters \n";
        //gp->optimize_hyperparams();
        std::cout << "Dataset correctly loaded and hyperparameters optimized \n "; 
};


void learning::InitializeMultiGP(std::string &Dir)
{
        std::cout << "Loading MultiGP Optimized hyperparameters \n";
        std::string str1 = "myGP/gp_";
        std::string str2;
        std::string buffer;

        if(!(boost::filesystem::exists("myGP/observations.dat")))
        {
            std::cout << "Not present files for loading MultiGp hyperparameters \n";
            return;
        }

        int dim_y_patch = 4;

        //for(int i=0;i<dim_y_patch;i++)
        for(int i=0;i<DIMY;i++)
        {   
            str2 = std::to_string(i);
            buffer = str1 + str2;
            
            std::cout << "loading the " << i << "-th gp" << "\n";
            
            this->gp->_gp_models[i].load<serialize::TextArchive>(buffer);
        }
};


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

Kuka_Vec learning::DataPoint(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix,std::vector<Kuka_Vec> d2Qvec,bool FLAG)
{
    // FILTER THE DATASET ONLY FOR REAL ROBOT
    Kuka_Vec  acc;
    Kuka_Vec Yk;

    std::vector<Kuka_Vec> d2Qvec_temp = d2Qvec;

    for(int i=0; i < NUMBER_OF_JOINTS; i++)
    {
        acc(i) = GramianCalc(State(i),State(i+NUMBER_OF_JOINTS),OldState(i),OldState(i+NUMBER_OF_JOINTS));
    }
    
    d2Qvec_temp.push_back(acc);

    Yk = MassMatrix * (reference - acc);

    Yk = Yk + prediction;



    return Yk;
};

void learning::DatasetUpdate(Kuka_State State, Kuka_State OldState, Kuka_Vec reference, Kuka_Vec prediction, Kuka_Mat MassMatrix,std::vector<Kuka_Vec> d2Qvec,bool FLAG)
{
    Eigen::VectorXd Y(DIMY);
    Eigen::VectorXd X(DIMX);
    Eigen::VectorXd tempY(DIMY);
    Eigen::VectorXd tempX(DIMX);
    
    Kuka_Vec  acc;

    std::vector<Kuka_Vec> d2Qvec_temp = d2Qvec;

    std::vector<Eigen::VectorXd> Y_temp_vec = this->DatasetY;

    d2Qvec_temp.pop_back();

    //Y of datapoint
    tempY = this->DataPoint(State, OldState, reference, prediction, MassMatrix,d2Qvec_temp,FLAG);
    
    Y = tempY;

    // Y normalization
    //Y = this->NormalizeY(tempY);

    Y_temp_vec.push_back(Y);

    //FILTERING THE Y
    
    
    if(FLAG)
    {
        if(this->DatasetY.size() > FILTER_LENGTH_LEARNING)
        {
            Y = Filter3(Y_temp_vec,FILTER_LENGTH_LEARNING);
        }
    }
    

    this->DatasetY.push_back(Y);

    //X of datapoint
    for(int i=0; i < NUMBER_OF_JOINTS; i++)
    {
        acc(i) = GramianCalc(State(i),State(i+NUMBER_OF_JOINTS),OldState(i),OldState(i+NUMBER_OF_JOINTS));
    }

    d2Qvec_temp.push_back(acc);
    
    // FILTER THE DATASET ONLY FOR REAL ROBOT
    
    if(FLAG)
    {
        acc = Filter2(d2Qvec_temp,FILTER_LENGTH_LEARNING);
    }
    

    tempX << OldState,acc;

    //tempX << OldState,reference;

    //std::cout << "OldState for Xtrain = " << OldState << "\n";

    X = tempX;
    // X normalization
    //X = this->NormalizeX(tempX);

    this->DatasetX.push_back(X);
};

void learning::GpUpdate()
{
    Kuka_Vec  Y;
    Input_Vec X;
    

    Y = this->DatasetY.back();
    X = this->DatasetX.back();
    this->gp->add_sample(X,Y);
    
    /*
    Eigen::VectorXd Xtemp(12);
    Eigen::VectorXd Ytemp(4);
    
    Xtemp(0) = X(0);
    Xtemp(1) = X(1);
    Xtemp(2) = X(2);
    Xtemp(3) = X(3);

    Xtemp(4) = X(7);
    Xtemp(5) = X(8);
    Xtemp(6) = X(9);
    Xtemp(7) = X(10);
    
    Xtemp(8) = X(14);
    Xtemp(9) = X(15);
    Xtemp(10) = X(16);
    Xtemp(11) = X(17);

    Ytemp(0) = Y(0);
    Ytemp(1) = Y(1);
    Ytemp(2) = Y(2);
    Ytemp(3) = Y(3);

    this->gp->add_sample(Xtemp,Ytemp);
    */
    
};


Kuka_Vec learning::GpPredict(Kuka_Vec Qquery, Kuka_Vec dQquery, Kuka_Vec d2Q_ref)
{
    Kuka_Vec prediction = Kuka_Vec::Constant(0.0);

    //Kuka_Vec prediction;
    
    double sigma;
    
    Eigen::VectorXd mu;

    Eigen::VectorXd Xtemp(12);

    Eigen::VectorXd Ytemp(4);

    Input_Vec query_point;
   
    query_point << Qquery,dQquery,d2Q_ref;

    prediction = this->gp->mu(query_point);
    
    /*
    Xtemp(0) = query_point(0);
    Xtemp(1) = query_point(1);
    Xtemp(2) = query_point(2);
    Xtemp(3) = query_point(3);

    Xtemp(4) = query_point(7);
    Xtemp(5) = query_point(8);
    Xtemp(6) = query_point(9);
    Xtemp(7) = query_point(10);
    
    Xtemp(8) = query_point(14);
    Xtemp(9) = query_point(15);
    Xtemp(10) = query_point(16);
    Xtemp(11) = query_point(17);  

    Ytemp = this->gp->mu(Xtemp);

    prediction(0) = Ytemp(0);
    prediction(1) = Ytemp(1);
    prediction(2) = Ytemp(2);
    prediction(3) = Ytemp(3);
    */
   
   return prediction;
};

Eigen::VectorXd learning::NormalizeY(Eigen::VectorXd Y)
{
    int i;
    Eigen::VectorXd Normalized(DIMY);

    for(i=0;i<DIMY;i++)
    {
        Normalized(i) = (Y(i) - this->Y_Min_Vector(i)) / (this->DeltaY_Vector(i));
    }
    return Normalized;
};

Eigen::VectorXd learning::DeNormalizeY(Eigen::VectorXd YNormalized)
{
    int i;
    
    Eigen::VectorXd DeNormalized(DIMY);
    
    for(i=0;i<DIMY;i++)
    {
        DeNormalized(i) = YNormalized(i) * (this->DeltaY_Vector(i)) + this->Y_Min_Vector(i);
    }
    return DeNormalized;
};


Eigen::VectorXd learning::NormalizeX(Eigen::VectorXd X)
{
    int i;
    Eigen::VectorXd Normalized(DIMX);

    for(i=0;i<DIMX;i++)
    {
        Normalized(i) = (X(i) - this->X_Min_Vector(i)) / (this->DeltaX_Vector(i));
    }
    return Normalized;
};

Eigen::VectorXd learning::DeNormalizeX(Eigen::VectorXd XNormalized)
{
    int i;
    Eigen::VectorXd DeNormalized(DIMX);
    
    for(i=0;i<DIMX;i++)
    {
        DeNormalized(i) = XNormalized(i) * (this->DeltaX_Vector(i)) + this->X_Min_Vector(i);
    }
    return DeNormalized;
};
