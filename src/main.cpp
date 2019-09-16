#include<controller/controller_kuka.hpp>

int main(int argc, char *argv[])
{

	
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;
	
	Eigen::VectorXd Q0 = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd dQ0 = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

	Eigen::VectorXd Q_ref = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd G = Eigen::VectorXd(NUMBER_OF_JOINTS);

	//std::string Mode("impedence");
	
	std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	
	float prova[NUMBER_OF_JOINTS];

	Eigen::VectorXd Torques_ref = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd Acc = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd filtered_signal = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd Torques_measured = Eigen::VectorXd(NUMBER_OF_JOINTS);
	
	Eigen::VectorXd state = Eigen::VectorXd(NUMBER_OF_JOINTS);

	//controller_kuka Controller(NUMBER_OF_JOINTS, DELTAT, int(NUMBER_OF_JOINTS)*3, Mode);

	controller_kuka Controller(true, Mode);

	if (Controller.FRI->IsMachineOK())
	{
		Controller.MeasureJointPositions();

		for(int i=0;i<NUMBER_OF_JOINTS;i++)
		{
			std::cout<<Controller.JointValuesInRad[i] <<"\n";
			Controller.Q(i) = Controller.JointValuesInRad[i];
			Controller.Qold(i) = Controller.Q(i);
			Q0(i) = Controller.Q(i);
		}
	}
	
	std::cout << "RUN TIME" << RUN_TIME_IN_SECONDS<<"\n";

	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{	
		Tic = std::chrono::system_clock::now();
		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;

		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);
		
		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		//Torques_ref = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.3*std::sin(1.0*Time));

		Q_ref = Q0 + Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.1*std::sin(Time));
		
		state = Controller.GetState();
		
		G = Controller.GetGravity();
		
		Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, Q_ref) - G;

		//std::cout<< "Q = "<< Controller.Q << "dQ = " << Controller.dQ << " \n"; 

		Controller.Qsave.push_back(Controller.Q);
		Controller.dQsave.push_back(Controller.dQ);
		
		Controller.d2Qold = Controller.d2Q;
		Controller.d2Q = Controller.AccCalculator(Controller.dQ, Controller.dQold);
		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.state_filtering();

		//Controller.SetTorques(Torques_ref);
		
		Controller.SetJointsPositions(Q_ref);
		
		//Prova_eig = Controller.FeedbackLinearization(Q_old, dQ_old, d2Q_ref);
		
		//Controller.MeasureJointTorques();		
		//Prova_eig_3 = Controller.GetMass();

		CycleCounter++;
		//std::cout<<CycleCounter<<"\n";
		//std::cout<<Time<<"\n";
		Toc = std::chrono::system_clock::now();
		elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(Toc - Tic).count();
		std::cout << elapsed_time << "\n";
	}
	
	fprintf(stdout, "Stopping the robot...\n");

	ResultValue	=	Controller.FRI->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	fprintf(stdout, "Deleting the object...\n");
	
	Controller.writer.write_data(qsave,Controller.Qsave);
	Controller.writer.write_data(dqsave,Controller.dQsave);
	Controller.writer.write_data(d2qsave,Controller.d2Qsave);
	
	delete Controller.FRI;
	
	fprintf(stdout, "Object deleted...\n");
	
	return(EXIT_SUCCESS);
	
return 0;
}
