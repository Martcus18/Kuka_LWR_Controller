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
    
    Kuka_Mat B_eig;

    Kuka_Vec C_eig;
    Kuka_Vec g_eig;
    Kuka_Vec friction_eig;
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

Kuka_Vec controller_kuka::PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd)
{
    Kuka_Vec e;
    Kuka_Vec de;
    Kuka_Vec control;
    
    e = Qd - Q;
    de = dQd - dQ;
    
    integralsum = integralsum + e;

    control = Kp * e + Kd * de + Ki * DELTAT *  integralsum;

    return control;
};

void controller_kuka::RLSTorque()
{
    Kuka_Vec alpha_new;
    Kuka_Vec epsilon_new;
    Kuka_Vec K_new;
    Kuka_Vec P_new;
    Kuka_Vec temp = Kuka_Vec::Constant(1.0);
    Kuka_Vec temp2;


    //DA AGGIUSTARE

    epsilon_new = torque_assigned - torque_measured - (dQ.array().sign() * alpha.back().array()).matrix();
    
    //FOR UPDATING ALPHA
    K_new = ((P.back().array() * dQ.array().sign())  / (temp.array() + P.back().array())).matrix();
    alpha_new = alpha.back() + (K_new.array() * epsilon_new.array()).matrix();
    P_new = (P.back().array() - K_new.array() * P.back().array() * dQ.array().sign()).matrix();

    

    alpha.push_back(alpha_new);
    epsilon.push_back(epsilon_new);
    K.push_back(K_new);
    P.push_back(P_new);
};

Kuka_Vec controller_kuka::TorqueAdjuster(Kuka_Vec torques, Kuka_Vec dQ)
{
    Kuka_Vec torques_adjusted;
    int i;
    Kuka_Vec alpha_temp = alpha.back();
    Kuka_Vec beta_temp = beta.back();
    Kuka_Vec gamma_temp = gamma.back();
    Kuka_Vec eta_temp = eta.back();
    int temp_sign;
    //torques_adjusted = torques + (alpha.back().array() * dQ.array().sign()).matrix();
    //TO BE FIXED
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {
        if(torques(i) <= 0.0)
        {
            temp_sign = -1;
        }
        else
        {
            temp_sign = +1;
        }
        
        torques_adjusted(i) = torques(i) * alpha_temp(i) + beta_temp(i) + gamma_temp(i) * 0.5 * (temp_sign+1) + eta_temp(i) * 0.5 * (-temp_sign+1);
    }
    //torques_adjusted = (torques.array() * alpha.back().array() + beta.back().array() + torques.array().sign() * gamma.back().array()).matrix();
    return torques_adjusted;
};

Kuka_Vec controller_kuka::EulerDifferentiation(Kuka_Vec X, Kuka_Vec Xold)
{
    Kuka_Vec dX;
    
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        dX(i) = (X(i) - Xold(i)) / DELTAT;
    }
    return dX;
};

void controller_kuka::SetTorques(Kuka_Vec torques)
{
    this->EigToArray(torques, CommandedTorquesInNm);
    this->FRI->SetCommandedJointTorques(CommandedTorquesInNm);
};

void controller_kuka::SetJointsPositions(Kuka_Vec positions)
{   
    this->EigToArray(positions,CommandedJointPositions);
    this->FRI->SetCommandedJointPositions(CommandedJointPositions);
};

Kuka_State controller_kuka::GetState()
{
    Kuka_State state;

    old_robot_state = robot_state;

    MeasureJointPositions();

    for(int i=0;i<NUMBER_OF_JOINTS; i++)
    {
        Qold(i) = Q(i);
        Q(i) = JointValuesInRad[i];
    }
    
    dQold = dQ;
    dQ = EulerDifferentiation(Q,Qold);
    state<<Q,dQ;
    robot_state = state;
    return state;
};

Kuka_Vec controller_kuka::GetGravity()
{
    Kuka_Vec Gravity;

    this->FRI->GetCurrentGravityVector(GravityVector);
    this->ArrayToEig(GravityVector, Gravity);
    
    return Gravity;
};

Kuka_Vec controller_kuka::GetFriction(Kuka_Vec Q, Kuka_Vec dQ)
{
    float* friction = new float[7];
    float* q = new float[7];
    float* dq = new float[7];
    int i;
    
    Kuka_Vec friction_eig;
    
    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        q[i] = Q(i);
        dq[i] = dQ(i);
    }
    
    dyn->get_friction(friction,dq);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        friction_eig(i) = friction[i];
    }
    return friction_eig;
}

Kuka_Mat controller_kuka::GetMass(Kuka_Vec Q)
{    
    Kuka_Mat Mass;
    float** B = new float*[7];
    float* q = new float[7];

    int i,j;

    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        q[i] = Q(i);
        B[i] = new float[NUMBER_OF_JOINTS];
    }
    
    dyn->get_B(B,q);

    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        for(int j=0;j<NUMBER_OF_JOINTS;j++)
        {
            Mass(i,j) = B[i][j];
        }
    }
    
    return Mass;
}

Kuka_Vec controller_kuka::Filter(std::vector<Kuka_Vec> &signal, int filter_length)
{
    int signal_length = signal.size();
    
    Kuka_Vec output = Kuka_Vec::Constant(0.0);

    if(signal_length > filter_length)
    {
            for(int i=0;i<filter_length;i++)
            {
                output = output + signal[signal_length-i];
            }
            output = output / filter_length;
    }
    else
    {
        output = signal.back();
    }
    return output;
};

void controller_kuka::dataset_creation(Eigen::VectorXd State, Eigen::VectorXd OldState, Eigen::VectorXd reference, Eigen::VectorXd prediction){};

void controller_kuka::FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT)
{
    int length = OUT.size();
    
    IN.clear();

    for(int i=0;i<length;i++)
    {
        IN.push_back(OUT[i]);
    }
};

Kuka_Vec controller_kuka::GearDiff(std::vector<Kuka_Vec> &signal)
{
    Kuka_Vec dsignal;
    Kuka_Vec temp;
    int length = signal.size();
    
    if(length > 4)
    {
        for(int i=0;i<NUMBER_OF_JOINTS;i++)
        {
            dsignal(i) = (1.0 / DELTAT) *  ((25.0/12.0) * signal[length](i) - 4.0 * signal[length-1](i) + 3*signal[length-2](i) - (4.0/3.0) * signal[length-3](i) + (1.0/4.0) * signal[length-4](i));
        }
    }
    return dsignal;
};