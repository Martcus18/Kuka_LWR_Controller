#include<controller/controller_kuka.hpp>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;
	
	int filter_length = 300;

	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec dQ_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec G;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_measured;

	Kuka_Vec temp_Vec;

	Kuka_Vec zero_vec = Kuka_Vec::Constant(0.0);

	Kuka_Vec Q_filtered;

	Kuka_Vec dQ_filtered;

	Kuka_Vec d2Q_filtered;

	Kuka_Mat Mass;

	Eigen::Vector3d end_effector;

	std::string Mode("impedence");
	
	//std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string qsave_filtered = "Q_filtered.txt";
  	std::string dqsave_filtered = "dQ_filtered.txt";
	std::string d2qsave_filtered = "d2Q_filtered.txt";
	std::string end_effector_pos = "end_eff.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	std::string foo = "foo.txt";
	std::string foo2 = "foo2.txt";
	std::string fooXd = "fooXd.txt";
	std::string Xdata = "X.txt";
	std::string Ydata = "Y.txt";

	
	Kuka_State state;

	std::vector<Eigen::VectorXd> temp;

	controller_kuka Controller(Mode);

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

	d2Q_ref = Kuka_Vec::Constant(0.0);
	Mass = Controller.GetMass(Controller.Q);

	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{	
		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
		
		Toc = std::chrono::system_clock::now();
		
		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);
		
		elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Toc - Tic).count();

		Tic = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}
		
		state = Controller.GetState();

		Q_filtered = Controller.Filter(Controller.Qsave,20);
		dQ_filtered = Controller.Filter(Controller.dQsave,20);
		d2Q_filtered = Controller.Filter(Controller.d2Qsave,20);

		Controller.old_state_filtered = Controller.state_filtered;

		Controller.state_filtered << Q_filtered, dQ_filtered;
		
		//temp_Vec = Controller.Regressor->DatasetCreation(Controller.state_filtered, Controller.old_state_filtered, d2Q_ref, Kuka_Vec::Constant(0.0), Mass);

		//temp_Vec = Controller.Regressor->DataPoint(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Kuka_Vec::Constant(0.0), Mass);
		//Controller.foo.push_back(temp_Vec);
		
		Controller.Regressor->DatasetUpdate(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Kuka_Vec::Constant(0.0), Mass);
		//Controller.fooXd.push_back(Controller.Regressor->DatasetY.back());
		
		


		Q_ref = Q0 + Kuka_Vec::Constant(0.2*std::sin(2*Time));
		dQ_ref = Kuka_Vec::Constant(0.4*std::cos(2*Time));
		d2Q_ref = Kuka_Vec::Constant(-0.8*std::sin(2*Time));
		
		//d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
		
		/*
		Controller.foo2.push_back(Mass.row(0));
		Controller.foo2.push_back(Mass.row(1));
		Controller.foo2.push_back(Mass.row(2));
		Controller.foo2.push_back(Mass.row(3));
		Controller.foo2.push_back(Mass.row(4));
		Controller.foo2.push_back(Mass.row(5));
		Controller.foo2.push_back(Mass.row(6));
		*/

		G = Controller.GetGravity();

		Controller.d2Q = Controller.EulerDifferentiation(Controller.dQ, Controller.dQold);
		Controller.foo2.push_back(Controller.FeedbackLinearization(Controller.Qsave.back(), Controller.dQsave.back(),Controller.d2Q));


		d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
		
		//Controller.Qsave_filtered.push_back(d2Q_ref);

		//FOR TORQUE CONTROL
		Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref) - G;
		
		Controller.Qsave_filtered.push_back(Controller.GetFriction(Controller.Q, Controller.dQ));

		//Torque th for checking feedback linearization
		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_filtered);

		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered , d2Q_ref);
		
		//Torque for PD+Feedforward control
		//Torques_ref = Torques_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);

		//Torques_ref = Controller.TorqueAdjuster(Torques_ref, Controller.dQ);

		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);

		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.d2Qold = Controller.d2Q;	

		//Controller.Qsave_filtered.push_back(Controller.Filter(Controller.Qsave,20));

		Controller.dQsave_filtered.push_back(Controller.Filter(Controller.dQsave,20));
		
		Controller.d2Qsave_filtered.push_back(Controller.Filter(Controller.d2Qsave,20));

		//Controller.SetTorques(Controller.TorqueAdjuster(Torques_ref,Controller.dQ));

		Controller.SetTorques(Torques_ref);

		//Controller.SetJointsPositions(Q_ref);
		
		Mass = Controller.GetMass(Controller.Q);

		end_effector = D_kin(Controller.Q);

		Controller.end_eff_pos.push_back(end_effector);

		Controller.MeasureJointTorques();	
		
		Torques_measured(0) = Controller.MeasuredTorquesInNm[0];
		Torques_measured(1) = Controller.MeasuredTorquesInNm[1];
		Torques_measured(2) = Controller.MeasuredTorquesInNm[2];
		Torques_measured(3) = Controller.MeasuredTorquesInNm[3];
		Torques_measured(4) = Controller.MeasuredTorquesInNm[4];
		Torques_measured(5) = Controller.MeasuredTorquesInNm[5];
		Torques_measured(6) = Controller.MeasuredTorquesInNm[6];


		Controller.Tor_meas.push_back(Torques_measured);
		
		Controller.Tor_meas_filtered.push_back(Controller.Filter(Controller.Tor_meas,20));

		//Controller.Tor_meas_filtered.push_back(Torques_measured);
		
		//Controller.Tor_th.push_back(Torques_ref);
		
		Controller.Tor_th.push_back(Torques_ref + G);
		
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
	
	
	Controller.FromKukaToDyn(temp,Controller.Qsave);
	Controller.writer.write_data(qsave,temp);
	
	Controller.FromKukaToDyn(temp,Controller.dQsave);
	Controller.writer.write_data(dqsave,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave);
	Controller.writer.write_data(d2qsave,temp);
	
	Controller.FromKukaToDyn(temp,Controller.Qsave_filtered);
	Controller.writer.write_data(qsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.dQsave_filtered);
	Controller.writer.write_data(dqsave_filtered,temp);
	
	Controller.FromKukaToDyn(temp,Controller.d2Qsave_filtered);
	Controller.writer.write_data(d2qsave_filtered,temp);
	
	Controller.FromKukaToDyn(temp,Controller.Tor_meas_filtered);
	Controller.writer.write_data(torque_meas,temp);	

	Controller.FromKukaToDyn(temp,Controller.Tor_th);
	Controller.writer.write_data(torque_th,temp);

	Controller.writer.write_data(end_effector_pos,Controller.end_eff_pos);

	Controller.FromKukaToDyn(temp,Controller.foo);
	Controller.writer.write_data(foo,temp);
	
	Controller.FromKukaToDyn(temp,Controller.foo2);
	Controller.writer.write_data(foo2,temp);

	
	Controller.writer.write_data(Xdata,Controller.Regressor->DatasetX);
	Controller.writer.write_data(Ydata,Controller.Regressor->DatasetY);


	delete Controller.FRI;
	delete Controller.dyn;
	delete Controller.Regressor;

	fprintf(stdout, "Object deleted...\n");
	
	return(EXIT_SUCCESS);
	
return 0;
}
