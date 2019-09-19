#ifndef CONTROLLER_KUKA_HPP_
#define CONTROLLER_KUKA_HPP_
#include<controller/controller.hpp>
#include <FastResearchInterface.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>
#include<TypeIRML.h>
#include<LWR_Dynamic_Model_Lib.h>
#include <TypeIRML.h>


class controller_kuka : public controller
{
    public:
        //Constructors
        
        controller_kuka(){};
        ~controller_kuka(){};
        controller_kuka(bool Flag, std::string MODE):controller(Flag)
        {
	    int	            ResultValue=0;
            
            std::string IMP("impedence");
            std::string POS("position");

            dQ = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);
	    dQold = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);
            d2Q = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);
            d2Qold = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

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
        
        Eigen::VectorXd GetState();

        //Get gravity vector

        Kuka_Vec GetGravity();

        //Get mass matrix

        Kuka_Mat GetMass();

        //Conversion from Eigen vector to array of float

        void EigToArray(Kuka_Vec IN,float *OUT);

        //Conversion from array of float to Eigen Vector

        void ArrayToEig(float *IN, Kuka_Vec& OUT);

       //Dataset creation

        void dataset_creation(Eigen::VectorXd State, Eigen::VectorXd OldState, Eigen::VectorXd reference, Eigen::VectorXd prediction);
        
        //Dynamic feedback linearization

        Kuka_Vec FeedbackLinearization(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec reference);

        //PD controller

        Kuka_Vec PDController(Kuka_Vec Q, Kuka_Vec dQ, Kuka_Vec d2Q, Kuka_Vec Qd, Kuka_Vec dQd, Kuka_Vec d2Qd);
        
        //Filtering of the state

        void StateFiltering();

        //Adding of the torque bias for KUKA LWR-4
        Kuka_Vec TorqueAdjuster(Kuka_Vec torques, Kuka_Vec dQ);
        
        //Numerical differentiation for velocity calculation
        Kuka_Vec VelocityCalculator(Kuka_Vec Q, Kuka_Vec Qold);
        
        //Numerical differentiation for acceleration calculation
        Kuka_Vec AccCalculator(Kuka_Vec dQ, Kuka_Vec dQold);

        //Signal filter
        Kuka_Vec Filter(std::vector<Kuka_Vec> &signal);

        //Gear Differentiation
        Kuka_Vec GearDiff(std::vector<Kuka_Vec> &signal);

        //From Kuka_Vec to std::Vector<Eigen::VectorXd>
        void FromKukaToDyn(std::vector<Eigen::VectorXd>& IN, std::vector<Kuka_Vec>& OUT);
        
        //Attributes definition

        FastResearchInterface	*FRI;
        
	CLWR_Dynamic_Model_Lib *dyn;

        Eigen::VectorXd robot_state = Eigen::VectorXd(NUMBER_OF_JOINTS * 2);
        Eigen::VectorXd old_robot_state = Eigen::VectorXd(NUMBER_OF_JOINTS * 2);
        

        Kuka_Vec Q;
	Kuka_Vec Qold;

	Kuka_Vec dQ;
	Kuka_Vec dQold;
        
        Kuka_Vec d2Q;
        Kuka_Vec d2Qold;
        
        Kuka_Vec torque_measured;


        float	CommandedTorquesInNm		[NUMBER_OF_JOINTS],
              	CommandedStiffness      	[NUMBER_OF_JOINTS]={0.0},
	        CommandedDamping       		[NUMBER_OF_JOINTS]={0.0},
                CommandedJointPositions  	[NUMBER_OF_JOINTS],
	        MeasuredTorquesInNm		[NUMBER_OF_JOINTS],
	        JointValuesInRad		[NUMBER_OF_JOINTS],
                GravityVector                   [NUMBER_OF_JOINTS];
        float   MassMatrix                    [NUMBER_OF_JOINTS][NUMBER_OF_JOINTS];
};
#endif /* CONTROLLER_KUKA_HPP_ */
