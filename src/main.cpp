#include<controller/controller_kuka.hpp>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

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

	std::vector<Kuka_Vec> Q_ref_vec;

	std::vector<Kuka_Vec> dQ_ref_vec;

	std::vector<Kuka_Vec> d2Q_ref_vec;

	std::vector<Kuka_Vec> Time_array;

	std::vector<Kuka_Vec> Temp_array;
	
	Kuka_Vec Kuka_temp;

	Kuka_Vec Kuka_temp2;

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
	std::string foo_pred = "predictions.txt";
	std::string foo3 = "foo3.txt";
	std::string friction = "friction.txt";
	std::string Q_ref_file = "Qref.txt";
	std::string dQ_ref_file = "dQref.txt";
	std::string d2Q_ref_file = "d2Qref.txt";
	std::string Xdata = "X.txt";
	std::string Ydata = "Y.txt";
	std::string dqhatsave = "dQ_hat.txt";
	std::string rsave = "res.txt";
	std::string r_obsave = "res_ob.txt";

	Kuka_State state;

	bool FLAG = ROBOT_CONTROL;

	std::array<int,7> flag = {0,0,0,0,0,0,0};

	//CHECKING THE SAMPLE TIME	
	controller_kuka Controller(Mode,FLAG);

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

		bool coll = true;

		while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
		{	
			Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;
			
			//std::cout << "CycleTime = " << Controller.FRI->GetFRICycleTime() << "\n";

			Tic = std::chrono::system_clock::now();

			Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);

			Toc = std::chrono::system_clock::now();

			if (!Controller.FRI->IsMachineOK())
			{
				fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
				break;
			}

			auto Tic2 = std::chrono::system_clock::now();

			state = Controller.GetState(FLAG);
			
			Controller.Qsave.push_back(Controller.Q);

			Controller.dQsave.push_back(Controller.dQ);

			//Q_filtered = Controller.Filter(Controller.Qsave,FILTER_LENGTH);

			Q_filtered = Controller.Q;

			dQ_filtered = Controller.Filter(Controller.dQsave,FILTER_LENGTH);
			
			//Controller.d2Q = Controller.EulerDifferentiation(Controller.dQ, Controller.dQold);

			//Kuka_temp = Controller.EulerDifferentiation(Controller.dQ, Controller.dQold);

			Controller.d2Q = Controller.EulerDifferentiation(dQ_filtered,Controller.dQsave_filtered.back());

			Controller.d2Qold = Controller.d2Q;

			Controller.d2Qsave.push_back(Controller.d2Q);

			d2Q_filtered = Controller.Filter(Controller.d2Qsave,FILTER_LENGTH);

			Controller.Qsave_filtered.push_back(Q_filtered);
			
			Controller.dQsave_filtered.push_back(dQ_filtered);
			
			Controller.d2Qsave_filtered.push_back(d2Q_filtered);

			Controller.old_state_filtered = Controller.state_filtered;

			Controller.state_filtered << Q_filtered, dQ_filtered;

			//end_effector = D_kin(Controller.Q);

			//Controller.end_eff_pos.push_back(end_effector);

			G = Controller.GetGravity();

			//MASS CALCULATION FOR LEARNING
			
			//Mass = Controller.GetMass(Q_filtered);

			Mass = Controller.GetMass(Controller.Q);

			Q_ref = Q0 + Kuka_Vec::Constant(0.1*(1.0 - std::cos(Time)));
			
			dQ_ref = Kuka_Vec::Constant(0.1*std::sin(Time));
			
			d2Q_ref = Kuka_Vec::Constant(0.1*std::cos(Time));

			Q_ref(4) = Q0(4);
			Q_ref(5) = Q0(5);
			Q_ref(6) = Q0(6);

			dQ_ref(4) = 0.0;
			dQ_ref(5) = 0.0;
			dQ_ref(6) = 0.0;

			d2Q_ref(4) = 0.0;
			d2Q_ref(5) = 0.0;
			d2Q_ref(6) = 0.0;


			Q_ref_vec.push_back(Q_ref);

			dQ_ref_vec.push_back(dQ_ref);

			d2Q_ref_vec.push_back(d2Q_ref);
			
			std::cout << "Ts = " << Time << "---\n";
			
			//SAFETY CHECK FOR POSITIONS AND VELOCITIES

			if((!Controller.VelocitySafety(Controller.dQ)) || (!Controller.JointSafety(Controller.Q)))
			{	
				//EXITING THE CONTROL LOOP
				std::cout << "Breaking safety controllers for either Velocities and Joints position \n";
				break;
			}
									
			//d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
			
			// USANDO INFORMAZIONI FILTRATE
			d2Q_ref = d2Q_ref + Controller.PDController(Q_filtered, dQ_filtered,d2Q_filtered, Q_ref, dQ_ref , d2Q_filtered);

			//Kuka_temp2 = d2Q_ref;
			
			//d2Q_ref = Controller.SignalAdjuster(d2Q_ref,6.5);

			//d2Q_ref(2) = Kuka_temp2(2);

			Temp_array.push_back(d2Q_ref);	

			//CHECK IF A COLLISION HAS OCCURRED

			if (Time>=1.0 && coll==true)
			{	
				coll = std::all_of(flag.begin(), flag.end(), [](int i) { return i==0; });	
			}

			if (coll == 0) 
			{
				Q_ref = Controller.Q;
				dQ_ref = Kuka_Vec::Constant(0.0);
			}		

			//Torques_ref = Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
			
			Torques_ref = Controller.PDController(Q_filtered, dQ_filtered, d2Q_filtered, Q_ref, dQ_ref , Controller.d2Q);

			//TORQUE COMMANDING			

			Controller.FRI->SetCommandedJointPositions(Controller.JointValuesInRad);

			Controller.SetTorques(Torques_ref);
			
			//Controller.SetJointsPositions(Q_ref);

			//Controller.torque_assigned = Torques_ref + G;

			//TORQUE MEASUREMENTS

			Controller.MeasureJointTorques();	
			
			Controller.torque_measured(0) = Controller.MeasuredTorquesInNm[0];
			Controller.torque_measured(1) = Controller.MeasuredTorquesInNm[1];
			Controller.torque_measured(2) = Controller.MeasuredTorquesInNm[2];
			Controller.torque_measured(3) = Controller.MeasuredTorquesInNm[3];
			Controller.torque_measured(4) = Controller.MeasuredTorquesInNm[4];
			Controller.torque_measured(5) = Controller.MeasuredTorquesInNm[5];
			Controller.torque_measured(6) = Controller.MeasuredTorquesInNm[6];

			//OBSERVER DYNAMICS

			Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_ref);

			Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

			Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

			//EVALUATION OF THE RESIDUAL

			Controller.r = Controller.Residual(Controller.Q, Controller.dQ, Torques_ref, Controller.r, CycleCounter);

			if (Time >= 1.0)
			{
				Controller.r_ob = Controller.Residual(Controller.Q, Controller.dQ_hat, Torques_ref, Controller.r_ob, CycleCounter);	
			}
			else
			{
				//The joint velocities are obtained through numerical differentiation
				Controller.r_ob = Controller.Residual(Controller.Q, Controller.dQ, Torques_ref, Controller.r_ob, CycleCounter);
			}

			//COMPARISON WITH THE THRESHOLDS

			flag = Controller.collision(Controller.r, Time);

			//Controller.Tor_meas.push_back(Controller.torque_measured);
			
			//Controller.Tor_meas_filtered.push_back(Controller.Filter(Controller.Tor_meas,FILTER_LENGTH));

			Controller.Tor_meas_filtered.push_back(Controller.torque_measured);

			//Controller.Tor_th.push_back(Controller.TorqueAdjuster(Torques_ref+G,Controller.Q,Controller.dQ));
			
			Controller.Tor_th.push_back(Torques_ref + G + Prediction);

			CycleCounter++;
			
			auto Toc2 = std::chrono::system_clock::now();

			//Toc = std::chrono::system_clock::now();

			elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(Toc - Tic).count();
			elapsed_time = elapsed_time / 1e3;

			std::cout << "elapsed = " << elapsed_time << "\n";		

			elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(Toc2 - Tic2).count();
			elapsed_time = elapsed_time / 1e3;

			std::cout << "elapsed2 = " << elapsed_time << "--------------\n";		

			
		}

		fprintf(stdout, "Stopping the robot...\n");

		ResultValue	= Controller.FRI->StopRobot();

		if (ResultValue != EOK)
		{
			fprintf(stderr, "An error occurred during stopping the robot...\n");
		}
		else
		{
			fprintf(stdout, "Robot successfully stopped.\n");
		}

	
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

		//ESTIMATED VELOCITIES
		Controller.FromKukaToDyn(temp,Controller.dQ_hat_save);
		Controller.writer.write_data(dqhatsave,temp);

		//RESIDUALS
		Controller.FromKukaToDyn(temp,Controller.r_save);
		Controller.writer.write_data(rsave,temp);

		Controller.FromKukaToDyn(temp,Controller.r_obs_save);
		Controller.writer.write_data(r_obsave,temp);
		
		//TORQUE VARIABLES PRINTING
		Controller.FromKukaToDyn(temp,Controller.Tor_meas_filtered);
		Controller.writer.write_data(torque_meas,temp);	

		Controller.FromKukaToDyn(temp,Controller.Tor_th);
		Controller.writer.write_data(torque_th,temp);

		//REFERENCE TRAJECTORY PRINTING	

		Controller.FromKukaToDyn(temp,Q_ref_vec);
		Controller.writer.write_data(Q_ref_file,temp);

		//Controller.FromKukaToDyn(temp,dQ_ref_vec);

		//DELETING POINTERS

		delete Controller.FRI;
		//delete Controller.dyn;
		delete Controller.Regressor;

		fprintf(stdout, "Objects deleted...\n");
		
		//return(EXIT_SUCCESS)

return 0;
}
