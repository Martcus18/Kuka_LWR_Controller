#include <controller/controller_kuka.hpp>


Eigen::VectorXd controller_kuka::FeedbackLinearization(Eigen::VectorXd Q, Eigen::VectorXd dQ, Eigen::VectorXd reference)
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
    Eigen::VectorXd TauFl(NUMBER_OF_JOINTS);
    
    
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

void controller_kuka::EigToArray(Eigen::VectorXd IN,float *OUT)
{
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        OUT[i] = IN(i);
    }
};

void controller_kuka::ArrayToEig(float *IN, Eigen::VectorXd& OUT)
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

Eigen::VectorXd controller_kuka::PD_controller(Eigen::VectorXd Q, Eigen::VectorXd dQ, Eigen::VectorXd d2Q, Eigen::VectorXd Qd,Eigen::VectorXd dQd,Eigen::VectorXd d2Qd)
{
    Eigen::VectorXd state(NUMBER_OF_JOINTS);
    return state;
};

Eigen::VectorXd controller_kuka::TorqueAdjuster(Eigen::VectorXd torques, Eigen::VectorXd dQ)
{
    double bias = 0.5;
    int i;
    auto torques_biased = Eigen::VectorXd::Constant(torques.rows(),0.0);

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
            torques(i) = 0.0;
        }
    }
    return torques_biased;
};

Eigen::VectorXd controller_kuka::VelocityCalculator(Eigen::VectorXd Q, Eigen::VectorXd Qold)
{
    Eigen::VectorXd velocity = Eigen::VectorXd(NUMBER_OF_JOINTS);
    
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        velocity(i) = (Q(i) - Qold(i)) / DELTAT;
    }
    return velocity;
};

Eigen::VectorXd controller_kuka::AccCalculator(Eigen::VectorXd dQ, Eigen::VectorXd dQold)
{
    Eigen::VectorXd acc = Eigen::VectorXd(NUMBER_OF_JOINTS);
    
    for(int i=0;i<NUMBER_OF_JOINTS;i++)
    {
        acc(i) = (dQ(i) - dQold(i)) / DELTAT;
    }
    
    return acc;    
};

void controller_kuka::SetTorques(Eigen::VectorXd torques)
{
    Eigen::VectorXd torque_biased(NUMBER_OF_JOINTS);
    //torque_biased = TorqueAdjuster(torques, dQ);
    //this->EigToArray(torques_biased, CommandedTorquesInNm);
    this->EigToArray(torques, CommandedTorquesInNm);
    
    this->FRI->SetCommandedJointTorques(CommandedTorquesInNm);
};

void controller_kuka::SetJointsPositions(Eigen::VectorXd positions)
{   
    this->EigToArray(positions,CommandedJointPositions);
    this->FRI->SetCommandedJointPositions(CommandedJointPositions);
};

Eigen::VectorXd controller_kuka::GetState()
{
    auto state = Eigen::VectorXd(NUMBER_OF_JOINTS*2);

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

Eigen::VectorXd controller_kuka::GetGravity()
{
    Eigen::VectorXd Gravity = Eigen::VectorXd(NUMBER_OF_JOINTS);
    this->FRI->GetCurrentGravityVector(GravityVector);
    this->ArrayToEig(GravityVector, Gravity);
    return Gravity;
};

Eigen::MatrixXd controller_kuka::GetMass()
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

Eigen::VectorXd controller_kuka::Filter(std::vector<Eigen::VectorXd> &signal)
{
    int filter_length = 200;
    int signal_length = signal.size();
    
    Eigen::VectorXd output = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

    if(signal_length > filter_length)
    {
            //output = ((25.0 / 12.0) * signal[signal_length-1] - 4.0 * signal[signal_length-2] + 3.0 * signal[signal_length-3] - (4.0/3.0) * signal[signal_length-4] + 0.25 * signal[signal_length-5]) / DELTAT;
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
    Eigen::VectorXd temp(NUMBER_OF_JOINTS);
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