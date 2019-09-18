#include<controller/controller_kuka.hpp>

int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;
	
	/*
	Eigen::VectorXd Q0 = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd dQ0 = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

	Eigen::VectorXd Q_ref = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd d2Q_ref = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd G = Eigen::VectorXd(NUMBER_OF_JOINTS);

	*/
	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec G;


	//std::string Mode("impedence");
	
	std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	
	float prova[NUMBER_OF_JOINTS];

	/*
	Eigen::VectorXd Torques_ref = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd Acc = Eigen::VectorXd(NUMBER_OF_JOINTS);

	Eigen::VectorXd Torques_measured = Eigen::VectorXd(NUMBER_OF_JOINTS);
	
	Eigen::VectorXd state = Eigen::VectorXd(NUMBER_OF_JOINTS);
	*/

	Kuka_Vec Torques_ref;

	Kuka_Vec Acc;

	Kuka_Vec Torques_measured;
	
	Kuka_Vec state;;
	
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
		Tic = std::chrono::system_clock::now();
	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{	
		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
		Toc = std::chrono::system_clock::now();
		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);
		
		elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(Toc - Tic).count();
		Tic = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		Q_ref = Q0 + Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.1*std::sin(2*Time));
		
		d2Q_ref = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,-0.4*std::sin(Time));

		state = Controller.GetState();
		
		G = Controller.GetGravity();
		
		//Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref);

		Torques_ref = G;

		//Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref);
		//Torques_ref = Eigen::VectorXd::Constant(NUMBER_OF_JOINTS,0.0);

		Controller.Qsave.push_back(Controller.Q);
		
		Controller.dQsave.push_back(Controller.dQ);

		Controller.d2Qold = Controller.d2Q;

		Controller.d2Q = Controller.AccCalculator(Controller.dQ, Controller.dQold);

		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.state_filtering();
		
		Controller.SetTorques(Torques_ref);
		
		//Controller.SetJointsPositions(Q_ref);
		
		Controller.MeasureJointTorques();	
		
		

		Torques_measured(0) = Controller.MeasuredTorquesInNm[0];
		Torques_measured(1) = Controller.MeasuredTorquesInNm[1];
		Torques_measured(2) = Controller.MeasuredTorquesInNm[2];
		Torques_measured(3) = Controller.MeasuredTorquesInNm[3];
		Torques_measured(4) = Controller.MeasuredTorquesInNm[4];
		Torques_measured(5) = Controller.MeasuredTorquesInNm[5];
		Torques_measured(6) = Controller.MeasuredTorquesInNm[6];


		Controller.Tor_meas.push_back(Torques_measured);
		Controller.Tor_th.push_back(Torques_ref);

		CycleCounter++;
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
	
	/*
	Controller.writer.write_data(qsave,Controller.Qsave);
	Controller.writer.write_data(dqsave,Controller.dQsave);
	Controller.writer.write_data(d2qsave,Controller.d2Qsave);
	Controller.writer.write_data(torque_meas,Controller.Tor_meas);
	Controller.writer.write_data(torque_th,Controller.Tor_th);
	*/
	/*
	Controller.writer.write_data(qsave,Controller.Qsave_filtered);
	Controller.writer.write_data(dqsave,Controller.dQsave_filtered);
	Controller.writer.write_data(d2qsave,Controller.d2Qsave_filtered);
	*/
	delete Controller.FRI;
	
	fprintf(stdout, "Object deleted...\n");
	
	return(EXIT_SUCCESS);
	
return 0;
}
