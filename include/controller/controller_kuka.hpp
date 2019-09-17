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
	    
            dQ = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);
	    dQold = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);
            d2Q = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);
            d2Qold = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

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
                    std::cout << ResultValue << "\n";
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

        void SetTorques(Eigen::VectorXd torques);

        //Set joints positions in position control

        void SetJointsPositions(Eigen::VectorXd positions);

        //Get measured joints positions

        void MeasureJointPositions();

        //Get measured joint torques

        void MeasureJointTorques();

        //Calculate complete state [Q,dQ]
        
        Eigen::VectorXd GetState();

        //Get gravity vector

        Eigen::VectorXd GetGravity();

        //Get mass matrix

        Eigen::MatrixXd GetMass();

        //Conversion from Eigen vector to array of float

        void EigToArray(Eigen::VectorXd IN,float *OUT);

        //Conversion from array of float to Eigen Vector

        void ArrayToEig(float *IN, Eigen::VectorXd& OUT);

       //Dataset creation

        void dataset_creation(Eigen::VectorXd State, Eigen::VectorXd OldState, Eigen::VectorXd reference, Eigen::VectorXd prediction);
        
        //Dynamic feedback linearization

        Eigen::VectorXd FeedbackLinearization(Eigen::VectorXd Q, Eigen::VectorXd dQ, Eigen::VectorXd reference);

        //PD controller

        Eigen::VectorXd PD_controller(Eigen::VectorXd Q, Eigen::VectorXd dQ, Eigen::VectorXd d2Q, Eigen::VectorXd Qd,Eigen::VectorXd dQd,Eigen::VectorXd d2Qd);
        
        //Filtering of the state

        void state_filtering();

        //Adding of the torque bias for KUKA LWR-4
        Eigen::VectorXd TorqueAdjuster(Eigen::VectorXd torques, Eigen::VectorXd dQ);
        
        //Numerical differentiation for velocity calculation
        Eigen::VectorXd VelocityCalculator(Eigen::VectorXd Q, Eigen::VectorXd Qold);
        
        //Numerical differentiation for acceleration calculation
        Eigen::VectorXd AccCalculator(Eigen::VectorXd dQ, Eigen::VectorXd dQold);

        //Signal filter
        Eigen::VectorXd Filter(std::vector<Eigen::VectorXd> &signal);

        //Attributes definition

        FastResearchInterface	*FRI;


	CLWR_Dynamic_Model_Lib *dyn;

        Eigen::VectorXd robot_state = Eigen::VectorXd(NUMBER_OF_JOINTS * 2);
        Eigen::VectorXd old_robot_state = Eigen::VectorXd(NUMBER_OF_JOINTS * 2);
        
        Eigen::VectorXd Q = Eigen::VectorXd(NUMBER_OF_JOINTS);
	Eigen::VectorXd Qold = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd dQ = Eigen::VectorXd(NUMBER_OF_JOINTS);
	Eigen::VectorXd dQold = Eigen::VectorXd(NUMBER_OF_JOINTS);
        
        Eigen::VectorXd d2Q = Eigen::VectorXd(NUMBER_OF_JOINTS);
        Eigen::VectorXd d2Qold = Eigen::VectorXd(NUMBER_OF_JOINTS);
        
        Eigen::VectorXd torque_measured = Eigen::VectorXd(NUMBER_OF_JOINTS);


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
