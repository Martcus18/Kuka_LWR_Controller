#include <controller/controller_kuka.hpp>


Kuka_Vec controller_kuka::FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference)
 {  
    float* q = new float[7];
    float* dq = new float[7];
    float* C = new float[7];
    float* g = new float[7];
    float* friction = new float[7];
    float** B = new float*[7];
    
    int i,j;

    Eigen::MatrixXd B_eig(NUMBER_OF_JOINTS,NUMBER_OF_JOINTS);
    Eigen::VectorXd C_eig(NUMBER_OF_JOINTS);
    Eigen::VectorXd g_eig(NUMBER_OF_JOINTS);
    Eigen::VectorXd friction_eig(NUMBER_OF_JOINTS);
    //Eigen::VectorXd TauFl(NUMBER_OF_JOINTS);
    Kuka_Vec TauFl;
    
    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        B[i] = new float[NUMBER_OF_JOINTS];
        q[i] = Q(i);
        dq[i] = dQ(i);
    }
    
    dyn->get_B(B,q);
    dyn->get_c(C,q,dq);
    dyn->get_g(g,q);
    dyn->get_friction(friction,dq);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        for(int j=0;j<NUMBER_OF_JOINTS;j++)
        {
            B_eig(i,j) = B[i][j];
        }
        C_eig(i) = C[i];
        g_eig (i)= g[i];
        friction_eig(i) = friction[i];
    }

    TauFl = B_eig * reference + C_eig + g_eig + friction_eig;
    return TauFl;
 };

void controller_kuka::EigToArray(Kuka_Vec IN,float *OUT)
{
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        OUT[i] = IN(i);
    }
};

void controller_kuka::ArrayToEig(float *IN, Kuka_Vec& OUT)
{
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        OUT(i) = IN[i];
    }
};


void controller_kuka::MeasureJointPositions()
{
    this->FRI->GetMeasuredJointPositions(JointValuesInRad);
    if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
    {
        std::cout<< "No connection was established, please check!"<<"\n";
    }
};

void controller_kuka::MeasureJointTorques()
{
    this->FRI->GetMeasuredJointTorques(MeasuredTorquesInNm);
    if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
    {
        std::cout<< "No connection was established, please check!"<<"\n";
    }
};

Kuka_Vec controller_kuka::PD_controller(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd)
{
    Kuka_Vec e(NUMBER_OF_JOINTS);
    Kuka_Vec de(NUMBER_OF_JOINTS);
    Kuka_Vec Torque(NUMBER_OF_JOINTS);
    
    e = Q - Qd;
    de = dQ - dQd;
    
    Torque = Kp * e + Kd * de;

    return Torque;
};

Kuka_Vec controller_kuka::TorqueAdjuster(Kuka_Vec torques, Kuka_Vec dQ)
{
    double bias = 0.5;
    int i;

    for(i=0;i<dQ.rows();i++)
    {
        if(dQ(i)>0.0)
        {
            torques(i) = torques(i) + bias;
        }
        else if(dQ(i)<0.0)
        {
            torques(i) = torques(i) - bias;
        }
        else
        {
            torques(i) = torques(i);
        }
    }
    return torques;
};

Kuka_Vec controller_kuka::VelocityCalculator(Kuka_Vec Q, Kuka_Vec Qold)
{
    Kuka_Vec velocity = Kuka_Vec(NUMBER_OF_JOINTS);
    
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        velocity(i) = (Q(i) - Qold(i)) / DELTAT;
    }
    return velocity;
};

Kuka_Vec controller_kuka::AccCalculator(Kuka_Vec dQ, Kuka_Vec dQold)
{
    Kuka_Vec acc = Kuka_Vec(NUMBER_OF_JOINTS);
    
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        acc(i) = (dQ(i) - dQold(i)) / DELTAT;
    }
    
    return acc;    
};

void controller_kuka::SetTorques(Kuka_Vec torques)
{
    Kuka_Vec torque_biased(NUMBER_OF_JOINTS);
    this->EigToArray(torques, CommandedTorquesInNm);
    
    this->FRI->SetCommandedJointTorques(CommandedTorquesInNm);
};

void controller_kuka::SetJointsPositions(Kuka_Vec positions)
{   
    this->EigToArray(positions,CommandedJointPositions);
    this->FRI->SetCommandedJointPositions(CommandedJointPositions);
};

Kuka_Vec controller_kuka::GetState()
{
    auto state = Kuka_Vec(NUMBER_OF_JOINTS*2);

    old_robot_state = robot_state;

    MeasureJointPositions();

    for(int i=0;i<NUMBER_OF_JOINTS; i++)
    {
        Qold(i) = Q(i);
        Q(i) = JointValuesInRad[i];
    }
    
    dQold = dQ;
    dQ = VelocityCalculator(Q,Qold);
    state<<Q,dQ;
    robot_state = state;
    return state;
};

Kuka_Vec controller_kuka::GetGravity()
{
    Kuka_Vec Gravity = Kuka_Vec(NUMBER_OF_JOINTS);
    this->FRI->GetCurrentGravityVector(GravityVector);
    this->ArrayToEig(GravityVector, Gravity);
    return Gravity;
};

Kuka_Vec controller_kuka::GetMass()
{
    Eigen::MatrixXd Mass = Eigen::MatrixXd(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
    int i,j;
    float** B = new float*[7];

    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        B[i] = new float[NUMBER_OF_JOINTS];
    }

    this->FRI->GetCurrentMassMatrix(B);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        for(j=0;j<NUMBER_OF_JOINTS;j++)
        {
            Mass(i,j) = B[i][j];
        }
    }
    return Mass;
}

Kuka_Vec controller_kuka::Filter(std::vector<Kuka_Vec> &signal)
{
    int filter_length = 200;
    int signal_length = signal.size();
    
    //Eigen::VectorXd output = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);
    Kuka_Vec output;

    if(signal_length > filter_length)
    {
            for(int i=1;i<filter_length+1;i++)
            {
                output = output + signal[signal_length-i];
            }
            output = output / filter_length;
    }
    return output;
};

void controller_kuka::dataset_creation(Eigen::VectorXd State, Eigen::VectorXd OldState, Eigen::VectorXd reference, Eigen::VectorXd prediction){};

void controller_kuka::state_filtering()
{
    
    Kuka_Vec temp(NUMBER_OF_JOINTS);
    int minimum_size = 3;

    if(Qsave.size() > minimum_size)
    {
        temp = Filter(Qsave);
        Qsave_filtered.push_back(temp);
    
        temp = Filter(dQsave);
        dQsave_filtered.push_back(temp);

        //temp = AccCalculator(dQsave_filtered[Qsave_filtered.size()-1], dQsave_filtered[Qsave_filtered.size()-2]);
        temp = Filter(d2Qsave);
        d2Qsave_filtered.push_back(temp);
        
    }
    else
    {
        Qsave_filtered.push_back(Qsave.back());
        dQsave_filtered.push_back(dQsave.back());
    }
};