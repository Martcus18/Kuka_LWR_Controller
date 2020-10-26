#ifndef CONTROLLER_KUKA_HPP_
#define CONTROLLER_KUKA_HPP_

#include <controller/controller.hpp>


class controller_kuka : public controller
{
    public:
        //Constructors
        
        controller_kuka(){};
        controller_kuka(std::string MODE,bool FLAG)
        {
	    int	            ResultValue=0;
            
            std::string IMP("impedence");
            std::string POS("position");
	    
            std::string Xdata = "myGP/samples.dat";
	    std::string Ydata = "myGP/observations.dat";            
            
            std::string Dir = "myGP/";

            dQ = Kuka_Vec::Constant(0.0);
	    dQold = Kuka_Vec::Constant(0.0);
            d2Q = Kuka_Vec::Constant(0.0);
            d2Qold = Kuka_Vec::Constant(0.0);
            
            Kuka_Vec temp_zero = Kuka_Vec::Constant(0.0);
            
            std::cout << "\n Initialization of the GP \n";
            
            Regressor = new learning();
            
            //Regressor->InitializeGp(Xdata,Ydata);

            Regressor->InitializeMultiGP(Dir);

            //dyn = new CLWR_Dynamic_Model_Lib();

            if(FLAG)
            {
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
            }
            else
            {
                Q(0) = 0.0;
		Q(1) = 1.571;
		Q(2) = 0.0;
		Q(3) = 1.571;
		Q(4) = 0.0;
		Q(5) = 1.571;
		Q(6) = 0.0;
            }
            
            Qsave.push_back(Q);
            dQsave.push_back(dQ);
            d2Qsave.push_back(d2Q);
            Qsave_filtered.push_back(Q);
            dQsave_filtered.push_back(dQ);
            d2Qsave_filtered.push_back(d2Q);
            robot_state << Q, dQ;
            old_robot_state << Q, dQ;
            state_filtered << Q , dQ;
            old_state_filtered << Q, dQ;            

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
        
        Kuka_State GetState(bool FLAG);

        //Get gravity vector of KUKA

        Kuka_Vec GetGravity();

        //Get gravity of the nominal model

        Kuka_Vec GetGravityFL(Kuka_Vec Q);

        //Get gravity of the nominal model

        Kuka_Vec GetGravityFLFake(Kuka_Vec Q);

        // Get friction of the nominal model

        Kuka_Vec GetFriction(Kuka_Vec Qnow, Kuka_Vec dQnow);

        // Get Mass Matrix nominal model
        Kuka_Mat GetMass(Kuka_Vec Qnow);

        // Get Mass Matrix fake model

        Kuka_Mat GetMassFake(Kuka_Vec Qnow);
        
        //Dynamic feedback linearization

        Kuka_Vec FeedbackLinearization(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec reference);

        //Dynamic feedback linearization fake model

        Kuka_Vec FeedbackLinearizationFake(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec reference);

        //PD Controller

        Kuka_Vec PDController(Kuka_Vec Qnow, Kuka_Vec dQnow, Kuka_Vec d2Qnow, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd);

        //Adding of the torque bias for KUKA LWR-4
        Kuka_Vec SignalAdjuster(Kuka_Vec signal, double threshold);
        
        //Numerical differentiation for velocity calculation
        Kuka_Vec EulerDifferentiation(Kuka_Vec X, Kuka_Vec Xold);

        //Signal filter
        Kuka_Vec Filter(std::vector<Kuka_Vec> &signal, int filter_length);

        //Gear Differentiation
        Kuka_Vec GearDiff(std::vector<Kuka_Vec> &signal,int filter_length);

        //From Kuka_Vec to std::Vector<Eigen::VectorXd>
        void FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT);
        
        //Safety Mechanisms

        bool JointSafety(Kuka_Vec Qnow);
        
        bool VelocitySafety(Kuka_Vec dQnow);

        bool TorqueSafety(Kuka_Vec Torque);

        //Simulations routine

        Kuka_Vec SimDynamicModel(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque);

        Kuka_Vec SimDynamicModelFake(Kuka_Vec Qnow,Kuka_Vec dQnow,Kuka_Vec Torque);

        Kuka_Vec EulerIntegration(Kuka_Vec dX, Kuka_Vec X);

        ~controller_kuka()
        {
                //delete this->FRI;
                //delete this->dyn;
                //delete this->Regressor;
                
        };

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
        
        Kuka_Vec integralsum = Kuka_Vec::Constant(0.0);

protected:
        //Conversion from Eigen vector to array of float
        void EigToArray(Kuka_Vec IN,float *OUT);

        //Conversion from array of float to Eigen Vector
        void ArrayToEig(float *IN, Kuka_Vec& OUT);
};
#endif /* CONTROLLER_KUKA_HPP_ */
