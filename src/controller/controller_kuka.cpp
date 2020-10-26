#include <controller/controller_kuka.hpp>


Kuka_Vec controller_kuka::FeedbackLinearization(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec reference)
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
        q[i] = Qnow(i);
        dq[i] = dQnow(i);
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

    //TauFl = B_eig * reference + C_eig + g_eig;

    //REMEMBER TO UNCOMMENT FOR REAL ROBOT PERFORMANCE
    
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

Kuka_Vec controller_kuka::PDController(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec d2Qnow, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd)
{
    Kuka_Vec e;
    Kuka_Vec de;
    Kuka_Vec control;
    
    e = Qd - Qnow;
    de = dQd - dQnow;
    
    integralsum = integralsum + e;

    control = Kp * e + Kd * de + Ki * DELTAT *  integralsum;

    return control;
};


Kuka_Vec controller_kuka::SignalAdjuster(Kuka_Vec signal, double threshold)
{
    Kuka_Vec signal_adjusted;

    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        if(std::fabs(signal(i)) > threshold)
        {
            if(std::signbit(signal(i)))
            {
                signal_adjusted(i) = -threshold;
            }
            else
            {
                signal_adjusted(i) = threshold;
            }
        }
        else
        {
            signal_adjusted(i) = signal(i);
        }
    }
    
    return signal_adjusted;
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

Kuka_State controller_kuka::GetState(bool FLAG)
{
    Kuka_State state;
    Kuka_Vec gear_prova;

    std::vector<Kuka_Vec> Qtemp = this->Qsave;

    this->old_robot_state = this->robot_state;

    //MOVED HERE OTHERWISE NOT WORKING ON THE REAL ROBOT
    
    //this->dQold = this->dQ;

    // IN CASE OF ROBOT CONTROL READING THE ENCODERS
    if(FLAG)
    {
        this->dQold = this->dQ;

        // IN ROBOT LOOP THIS READS THE ENCODERS
        this->MeasureJointPositions();

        for(int i=0;i<NUMBER_OF_JOINTS; i++)
        {
            this->Qold(i) = this->Q(i);
            this->Q(i) = JointValuesInRad[i];
        }
    

    // NUMERICAL DIFFERENTIATION  
    this->dQ = EulerDifferentiation(this->Q,this->Qold);
    //this->dQ = GearDiff(Qtemp, 500);

    }

    state<<this->Q,this->dQ;

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

Kuka_Vec controller_kuka::GetGravityFL(Kuka_Vec Qnow)
{
    float* q = new float[7];
    float* g = new float[7];
    int i;
    Kuka_Vec g_eig;
    
    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        q[i] = Qnow(i);
    }
    
    dyn->get_g(g,q);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        g_eig (i)= g[i];
    }
    
    return g_eig;
}

Kuka_Vec controller_kuka::GetFriction(Kuka_Vec Qnow, Kuka_Vec dQnow)
{
    float* friction = new float[7];
    float* q = new float[7];
    float* dq = new float[7];
    int i;
    
    Kuka_Vec friction_eig;
    
    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        q[i] = Qnow(i);
        dq[i] = dQnow(i);
    }
    
    dyn->get_friction(friction,dq);
    
    for(i=0;i<NUMBER_OF_JOINTS;i++)
    {    
        friction_eig(i) = friction[i];
    }
    return friction_eig;
}

Kuka_Mat controller_kuka::GetMass(Kuka_Vec Qnow)
{    
    Kuka_Mat Mass;
    float** B = new float*[7];
    float* q = new float[7];

    int i,j;

    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        q[i] = Qnow(i);
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
                output = output + signal[signal_length - i -1];
            }
            output = output / filter_length;
    }
    else
    {
        output = signal.back();
    }
    return output;
};

void controller_kuka::FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT)
{
    int length = OUT.size();
    
    IN.clear();

    for(int i=0;i<length;i++)
    {
        IN.push_back(OUT[i]);
    }
};


Kuka_Vec controller_kuka::GearDiff(std::vector<Kuka_Vec> &signal, int filter_length)
{
    
    Kuka_Vec dsignal;
    int length = signal.size();

    if(length > filter_length)
    {
        for(int i=0;i<NUMBER_OF_JOINTS;i++)
        {
            dsignal(i) = (1.0 / DELTAT) *  ((25.0/12.0) * signal[length-1](i) - 4.0 * signal[length-2](i) + 3*signal[length-3](i) - (4.0/3.0) * signal[length-4](i) + (1.0/4.0) * signal[length-5](i));
        }
    }
    else
    {
        //In case of Q seq length < filter_length use Euler
        dsignal = (1.0 / DELTAT) * (signal[length-1] - signal[length-2]);
    }

    return dsignal;
};

bool controller_kuka::JointSafety(Kuka_Vec Qnow)
{
    bool flag = true;
    if((std::fabs(Qnow(0))>QL1)||(std::fabs(Qnow(1))>QL2)||(std::fabs(Qnow(2))>QL3)||(std::fabs(Qnow(3))>QL4)||(std::fabs(Qnow(4))>QL5)||(std::fabs(Qnow(5))>QL6)||(std::fabs(Qnow(6))>QL7))
    {
            flag = false;
            std::cout << "Joints bounds violation" << "\n";
            return flag;
    }
    return flag;
};

bool controller_kuka::VelocitySafety(Kuka_Vec dQnow)
{
    bool flag = true;
    if((std::fabs(dQnow(0))>VL1)||(std::fabs(dQnow(1))>VL2)||(std::fabs(dQnow(2))>VL3)||(std::fabs(dQnow(3))>VL4)||(std::fabs(dQnow(4))>VL5)||(std::fabs(dQnow(5))>VL6)||(std::fabs(dQnow(6))>VL7))
    {
            flag = false;
            std::cout << "Velocities bounds violation" << "\n";
            return flag;
    }
    return flag;
};

bool controller_kuka::TorqueSafety(Kuka_Vec dQnow)
{
    bool flag = true;
    return flag;
};


//DYNAMIC MODEL FOR SIMULATION
Kuka_Vec controller_kuka::SimDynamicModel(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque)
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
    Kuka_Vec Torque_temp;

    Kuka_Vec result;

    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        B[i] = new float[NUMBER_OF_JOINTS];
        q[i] = Qnow(i);
        dq[i] = dQnow(i);
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

    //Torque_temp = Torque - C_eig - g_eig - 0.01*friction_eig - 0.0001 * dQnow;
    //Torque_temp(6) = Torque(6) - C_eig(6) - g_eig(6);
    //Torque_temp(5) = Torque(5) - C_eig(5) - g_eig(5);
    
    Torque_temp = Torque - C_eig - g_eig;



    //result = B_eig.inverse() * (Torque - C_eig - g_eig - 0.01*friction_eig - 0.0001 * dQnow);
    //Torque_temp = Torque - C_eig - g_eig;
    result = B_eig.inverse() * (Torque_temp);

    return result;
};

Kuka_Vec controller_kuka::EulerIntegration(Kuka_Vec dX,Kuka_Vec X)
{
    Kuka_Vec Xnew;

    Xnew = X + dX * DELTAT;
    
    return Xnew;
};


//FAKE DYNAMIC MODEL FOR SIMULATION
Kuka_Vec controller_kuka::SimDynamicModelFake(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque)
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
    Kuka_Vec Torque_temp;

    Kuka_Vec result;

    for(i=0; i<NUMBER_OF_JOINTS; i++)
	{
        B[i] = new float[NUMBER_OF_JOINTS];
        q[i] = Qnow(i);
        dq[i] = dQnow(i);
    }
    
    dyn->get_B_fake(B,q);
    dyn->get_c(C,q,dq);
    // TO FIX get_c
    dyn->get_g_fake(g,q);
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
    
    result = B_eig.inverse() * (Torque - C_eig - g_eig - 0.01*friction_eig);

    return result;
};
