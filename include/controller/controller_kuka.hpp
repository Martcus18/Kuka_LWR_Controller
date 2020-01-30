#ifndef CONTROLLER_KUKA_HPP_
#define CONTROLLER_KUKA_HPP_

#include <controller/controller.hpp>


class controller_kuka : public controller
{
    public:
        //Constructors
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        controller_kuka(){};
        ~controller_kuka(){};
        controller_kuka(std::string MODE)
        {
	    int	            ResultValue=0;
            
            std::string IMP("impedence");
            std::string POS("position");

            dQ = Kuka_Vec::Constant(0.0);
	    dQold = Kuka_Vec::Constant(0.0);
            d2Q = Kuka_Vec::Constant(0.0);
            d2Qold = Kuka_Vec::Constant(0.0);
            
            Kuka_Vec temp_zero = Kuka_Vec::Constant(0.0);

            const float TimeOutValueInSeconds = 120.0;

            //FRI starting
            FRI			=	new FastResearchInterface("/home/kuka_linux/Desktop/Kuka_Controller/external/FRILibrary/etc/980039-FRI-Driver.init");	        
            
            //Choosing the controlling mode
            if(!MODE.compare(IMP))
            {
                    ResultValue = FRI->StartRobot(		FastResearchInterface::JOINT_IMPEDANCE_CONTROL, TimeOutValueInSeconds);
                    this->FRI->SetCommandedJointStiffness(CommandedStiffness);
                    this->FRI->SetCommandedJointDamping(CommandedDamping);
            }
            if(!MODE.compare(POS))
            {
                    ResultValue = FRI->StartRobot(		FastResearchInterface::JOINT_POSITION_CONTROL, TimeOutValueInSeconds);
            }
            else
            {
                std::cout<<"No allowed controlling mode";
            }
            
            if (ResultValue == EOK)
	    {
		        fprintf(stdout, "Robot successfully started.\n");
	    }
	    else
	    {
	               fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	    }

            fprintf(stdout, "Current system state:\n%s\n", FRI->GetCompleteRobotStateAndInformation());

            MeasureJointPositions();

            for(int i=0;i<NUMBER_OF_JOINTS;i++)
            {
                Q(i) = JointValuesInRad[i];
            }
            Qsave.push_back(Q);
            dQsave.push_back(dQ);
            d2Qsave.push_back(d2Q);
            Qsave_filtered.push_back(Q);
            dQsave_filtered.push_back(dQ);
            d2Qsave_filtered.push_back(d2Q);
            state_filtered << Q , dQ;
            old_state_filtered << Q, dQ;

            Regressor = new learning();

        };

        //Set torques in impedence control mode

        void SetTorques(Kuka_Vec torques);

        //Set joints positions in position control

        void SetJointsPositions(Kuka_Vec positions);

        //Get measured joints positions

        void MeasureJointPositions();

        //Get measured joint torques

        void MeasureJointTorques();

        //Calculate complete state [Q,dQ]
        
        Kuka_State GetState();

        //Get gravity vector of KUKA

        Kuka_Vec GetGravity();

        //Get gravity of the FL model

        //Kuka_Vec GetGravityFL(Kuka_Vec Q);

        Kuka_Vec GetFriction(Kuka_Vec Q, Kuka_Vec dQ);

        //Get mass matrix

        //Kuka_Mat GetMass();

        Kuka_Mat GetMass(Kuka_Vec Q);

       //Dataset creation

        void dataset_creation(Eigen::VectorXd State, Eigen::VectorXd OldState, Eigen::VectorXd reference, Eigen::VectorXd prediction);
        
        //Dynamic feedback linearization

        Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference);

        //PD Controller

        Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd);

        //Adding of the torque bias for KUKA LWR-4
        Kuka_Vec TorqueAdjuster(Kuka_Vec torques, Kuka_Vec Q, Kuka_Vec dQ);
        
        //Numerical differentiation for velocity calculation
        Kuka_Vec EulerDifferentiation(Kuka_Vec X, Kuka_Vec Xold);

        //Signal filter
        Kuka_Vec Filter(std::vector<Kuka_Vec> &signal, int filter_length);

        //Gear Differentiation
        Kuka_Vec GearDiff(std::vector<Kuka_Vec> &signal);

        //From Kuka_Vec to std::Vector<Eigen::VectorXd>
        void FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT);
        
        //Safety Mechanisms
        bool JointSafety(Kuka_Vec Q);
        
        bool VelocitySafety(Kuka_Vec dQ);

        bool TorqueSafety(Kuka_Vec Torque);

        //Attributes definition

        FastResearchInterface	*FRI;
        
	CLWR_Dynamic_Model_Lib *dyn;

        learning *Regressor;

        Kuka_State robot_state;
        Kuka_State old_robot_state;
        
        Kuka_State state_filtered;
        Kuka_State old_state_filtered;
        
        Kuka_Vec Q;
	Kuka_Vec Qold;

	Kuka_Vec dQ;
	Kuka_Vec dQold;
        
        Kuka_Vec d2Q;
        Kuka_Vec d2Qold;
        
        Kuka_Vec torque_measured;
        Kuka_Vec torque_assigned;

        float	CommandedTorquesInNm		[NUMBER_OF_JOINTS],
              	CommandedStiffness      	[NUMBER_OF_JOINTS]={0.0},
	        CommandedDamping       		[NUMBER_OF_JOINTS]={0.0},
                CommandedJointPositions  	[NUMBER_OF_JOINTS],
	        MeasuredTorquesInNm		[NUMBER_OF_JOINTS],
	        JointValuesInRad		[NUMBER_OF_JOINTS],
                GravityVector                   [NUMBER_OF_JOINTS];
        float   MassMatrix                    [NUMBER_OF_JOINTS][NUMBER_OF_JOINTS];
        
        std::vector<Kuka_Vec> alpha;
        std::vector<Kuka_Vec> beta;
        std::vector<Kuka_Vec> gamma;
        std::vector<Kuka_Vec> eta;

        std::vector<Kuka_Vec> epsilon;
        std::vector<Kuka_Vec> P;
        std::vector<Kuka_Vec> K;
        Kuka_Vec integralsum = Kuka_Vec::Constant(0.0);

protected:
        //Conversion from Eigen vector to array of float
        void EigToArray(Kuka_Vec IN,float *OUT);

        //Conversion from array of float to Eigen Vector
        void ArrayToEig(float *IN, Kuka_Vec& OUT);
};
#endif /* CONTROLLER_KUKA_HPP_ */
