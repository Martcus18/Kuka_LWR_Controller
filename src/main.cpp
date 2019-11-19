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

	Kuka_Vec torques_temp;

	Kuka_Vec temp_Vec;

	Kuka_Vec zero_vec = Kuka_Vec::Constant(0.0);

	Kuka_Vec Q_filtered;

	Kuka_Vec dQ_filtered;

	Kuka_Vec d2Q_filtered;

	Kuka_Vec Prediction = Kuka_Vec::Constant(0.0);
	
	std::vector<Kuka_Vec> Prediction_array;

	std::vector<Kuka_Vec> Ref_Acc;

	std::vector<Kuka_Vec> Time_array;
	
	Kuka_Vec Kuka_temp;

	std::vector<Eigen::VectorXd> temp;

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
	std::string fooXd = "predictions.txt";
	std::string alpha = "alpha.txt";
	std::string epsilon = "epsilon.txt";
	std::string P = "P.txt";
	std::string K = "K.txt";
	std::string ref = "ref_acc.txt";
	std::string Xdata = "X.txt";
	std::string Ydata = "Y.txt";
	
	char net_path1[] = "/home/kuka_linux/Desktop/Kuka_Controller/external/Tensorflow/models/inverse_mapping/net.pb";
	char in_name1[] =  "KerasInput_input_18";
	char out_name1[] = "KerasOutput_18/BiasAdd";

	Kuka_State state;

	Network_Output output;

	Network_Input input;

	tf_network net(net_path1,in_name1,out_name1);

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
	
	std::cout << "RUN TIME" << RUN_TIME_IN_SECONDS << "\n";
	
	//Tic = std::chrono::system_clock::now();

	d2Q_ref = Kuka_Vec::Constant(0.0);
	Mass = Controller.GetMass(Controller.Q);

	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{	
		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
		
		//Toc = std::chrono::system_clock::now();
		
		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);
		
		//elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Toc - Tic).count();

		//Tic = std::chrono::system_clock::now();

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

		//CHECKING USING NOT FILTERED STATE
		//temp_Vec = Controller.Regressor->DataPoint(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Kuka_Vec::Constant(0.0), Mass);
		temp_Vec = Controller.Regressor->DataPoint(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Prediction, Mass);
		

		//CHECKING USING FILTERED STATE
		//temp_Vec = Controller.Regressor->DataPoint(Controller.state_filtered, Controller.old_state_filtered, d2Q_ref, Kuka_Vec::Constant(0.0), Mass);

		Controller.foo.push_back(temp_Vec);
		
		//CHECKING USING NOT FILTERED STATE
		Controller.Regressor->DatasetUpdate(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Prediction, Mass);
		
		//CHECKING USING FILTERED STATE
		//Controller.Regressor->DatasetUpdate(Controller.state_filtered, Controller.old_state_filtered, d2Q_ref, Prediction, Mass);

		Q_ref = Q0 + Kuka_Vec::Constant(0.1*std::sin(2*Time));
		dQ_ref = Kuka_Vec::Constant(0.2*std::cos(2*Time));
		d2Q_ref = Kuka_Vec::Constant(-0.4*std::sin(2*Time));


		Controller.d2Q = Controller.EulerDifferentiation(Controller.dQ, Controller.dQold);

		G = Controller.GetGravity();

		//CHECKING USING NOT FILTERED STATE
		Controller.foo2.push_back(Controller.FeedbackLinearization(Controller.Qsave.back(), Controller.dQsave.back(),d2Q_filtered));
		//Controller.foo2.push_back(Controller.FeedbackLinearization(Controller.Qsave.back(), Controller.dQsave.back(),Controller.d2Q));

		//CHECKING USING FILTERED STATE
		//Controller.foo2.push_back(Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_filtered));

		d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
		
		//d2Q_ref = d2Q_ref + Controller.PDController(Q_filtered, dQ_filtered, d2Q_filtered, Q_ref, dQ_ref , d2Q_filtered);
		//Ref_Acc.push_back(d2Q_ref);
		Ref_Acc.push_back(d2Q_ref);

		//FOR TORQUE CONTROL
		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_ref) - G;
		//Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref) - G;
		

		//Torques_ref = Torques_ref + Prediction;

		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_ref) - G + Prediction;

		Torques_ref = Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);

		//Torque th for checking feedback linearization
		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_filtered);

		//Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered , d2Q_ref);

		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);

		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.d2Qold = Controller.d2Q;	

		//Controller.Qsave_filtered.push_back(Controller.Filter(Controller.Qsave,20));

		Controller.dQsave_filtered.push_back(Controller.Filter(Controller.dQsave,20));
		
		Controller.d2Qsave_filtered.push_back(Controller.Filter(Controller.d2Qsave,20));

		// X = [torque_des + G, Q, dQ]
		// Y = [torque_need + G]
		// torque_need = Y - G
		
		//input.block<7,1>(0,0) = Torques_ref;
		//input.block<7,1>(0,7) = Controller.Q;
		
		input(0) = Torques_ref(0);
		input(1) = Torques_ref(1);
		input(2) = Torques_ref(2);
		input(3) = Torques_ref(3);
		input(4) = Torques_ref(4);
		input(5) = Torques_ref(5);
		input(6) = Torques_ref(6);
		input(7) = Controller.Q(0);
		input(8) = Controller.Q(1);
		input(9) = Controller.Q(2);
		input(10) = Controller.Q(3);
		input(11) = Controller.Q(4);
		input(12) = Controller.Q(5);
		input(13) = Controller.Q(6);

		output = net.predict(input);
		std::cout << output << "\n";

/*		
		torques_temp(0) = output(0) - G(0);
		torques_temp(1) = output(1) - G(1);
		torques_temp(2) = output(2) - G(2);
		torques_temp(3) = output(3) - G(3);
		torques_temp(4) = output(4) - G(4);
		torques_temp(5) = output(5) - G(5);
		torques_temp(6) = output(6) - G(6);
*/
		torques_temp(0) = output(0) + G(0);
		torques_temp(1) = output(1) + G(1);
		torques_temp(2) = output(2) + G(2);
		torques_temp(3) = output(3) + G(3);
		torques_temp(4) = output(4) + G(4);
		torques_temp(5) = output(5) + G(5);
		torques_temp(6) = output(6) + G(6);		

		Controller.SetTorques(Torques_ref);

		//Controller.SetTorques(torques_temp);
		
		//Controller.SetJointsPositions(Q_ref);
		
		Mass = Controller.GetMass(Controller.Q);

		end_effector = D_kin(Controller.Q);

		Controller.end_eff_pos.push_back(end_effector);

		//Controller.torque_assigned = Torques_ref + G;

		Controller.MeasureJointTorques();	
		
		Controller.torque_measured(0) = Controller.MeasuredTorquesInNm[0];
		Controller.torque_measured(1) = Controller.MeasuredTorquesInNm[1];
		Controller.torque_measured(2) = Controller.MeasuredTorquesInNm[2];
		Controller.torque_measured(3) = Controller.MeasuredTorquesInNm[3];
		Controller.torque_measured(4) = Controller.MeasuredTorquesInNm[4];
		Controller.torque_measured(5) = Controller.MeasuredTorquesInNm[5];
		Controller.torque_measured(6) = Controller.MeasuredTorquesInNm[6];

		Controller.Tor_meas.push_back(Controller.torque_measured);
		
		//Controller.Tor_meas_filtered.push_back(Controller.Filter(Controller.Tor_meas,20));

		Controller.Tor_meas_filtered.push_back(Controller.torque_measured);
		
		//Controller.Tor_th.push_back(Torques_ref);
		
		//Controller.Tor_th.push_back(Torques_ref + G);

		//Controller.Tor_th.push_back(Controller.TorqueAdjuster(Torques_ref+G,Controller.dQ));
		
		Controller.Qsave_filtered.push_back(Torques_ref + G);

		//torques_temp = Controller.TorqueAdjuster(Torques_ref+G,Controller.Q, Controller.dQ);

		Controller.Tor_th.push_back(torques_temp);

		//Controller.Tor_th.push_back(Controller.TorqueAdjuster(Torques_ref+G,Controller.Q, Controller.dQ));
		
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
	
	//KINEMATIC VARIABLES PRINTING
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
	
	//TORQUE VARIABLES PRINTING
	Controller.FromKukaToDyn(temp,Controller.Tor_meas_filtered);
	Controller.writer.write_data(torque_meas,temp);	

	Controller.FromKukaToDyn(temp,Controller.Tor_th);
	Controller.writer.write_data(torque_th,temp);

	//END_EFFECTOR PRINTING
	Controller.writer.write_data(end_effector_pos,Controller.end_eff_pos);

	//REFERENCE ACCELERATION PRINTING	
	Controller.FromKukaToDyn(temp,Ref_Acc);
	Controller.writer.write_data(ref,temp);

	//TEMP VARIABLES PRINTING
	Controller.FromKukaToDyn(temp,Controller.foo);
	Controller.writer.write_data(foo,temp);
	
	Controller.FromKukaToDyn(temp,Controller.foo2);
	Controller.writer.write_data(foo2,temp);

	//LEARNING VARIABLES PRINTING
	Controller.FromKukaToDyn(temp,Prediction_array);
	Controller.writer.write_data(fooXd,temp);
	
	Controller.writer.write_data(Xdata,Controller.Regressor->DatasetX);
	Controller.writer.write_data(Ydata,Controller.Regressor->DatasetY);

	//RLS VARIABLES PRINTING
	Controller.FromKukaToDyn(temp,Controller.alpha);
	Controller.writer.write_data(alpha,temp);

	Controller.FromKukaToDyn(temp,Controller.epsilon);
	Controller.writer.write_data(epsilon,temp);	

	Controller.FromKukaToDyn(temp,Controller.K);
	Controller.writer.write_data(K,temp);

	Controller.FromKukaToDyn(temp,Controller.P);
	Controller.writer.write_data(P,temp);
	
	//DELETING POINTERS

	delete Controller.FRI;
	//delete Controller.dyn;
	//delete Controller.Regressor;

	fprintf(stdout, "Objects deleted...\n");
	
	return(EXIT_SUCCESS);
	
return 0;
}
