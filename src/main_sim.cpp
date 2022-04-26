#include<controller/controller_kuka.hpp>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

	std::cout << "Choose what external force you want to simulate: \n"; 
	std::cout << "1 --> fault on 1-st motor \n";
	std::cout << "2 --> fault on 2-nd motor \n";
	std::cout << "3 --> fault on 3-rd motor \n";
	std::cout << "4 --> fault on 4-th motor \n";
	std::cout << "5 --> fault on 5-th motor \n";
	std::cout << "6 --> fault on 6-th motor \n";
	std::cout << "7 --> fault on 7-th motor \n";
	std::cout << "8 --> fault on 4-th and 6-th motors \n";
	std::cout << "9 --> external force acting on the end-effector \n";

	int fault;
	
	scanf("%d", &fault); 

	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec dQ_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec G;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_nom;

	Kuka_Vec Torques_faulty = Kuka_Vec::Constant(0.0);

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

	std::vector<Kuka_Vec> fault_torque_vec;

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
	std::string dqhatsave = "dQ_hat.txt";
	std::string rsave = "res.txt";
	std::string r_obsave = "res_ob.txt";
	std::string torque_save = "torque_ref.txt";
	std::string fault_torque_save = "torque_faulty.txt";

	Eigen::Vector3d F(10,10,0);
	
	//std::string Xdata = "X.txt";
	//std::string Ydata = "Y.txt";

	Kuka_State state;

	bool FLAG = SIM_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	std::cout << "Simulation LOOP" << "\n";

	std::cout << "\n Remember to insert friction compensation for REAL robot experiment \n";

	Controller.Qold = Controller.Q;

	Q0 = Controller.Q;

	//SIMULATION LOOP
	while ((float)CycleCounter * DELTAT < 3)
	{
			Time = DELTAT * (float)CycleCounter;

			Mass = Controller.GetMass(Controller.Q);

			Q_ref = Q0 + Kuka_Vec::Constant(0.1*(1.0 - std::cos(Time)));
			
			dQ_ref = Kuka_Vec::Constant(0.1*std::sin(Time));
			
			d2Q_ref = Kuka_Vec::Constant(0.1*std::cos(Time));

			d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
			
			/*
			if(CycleCounter > (2 * FILTER_LENGTH))
			{
				Prediction =  Controller.Regressor->GpPredict(Controller.Q,Controller.dQ,d2Q_ref);
				Prediction_array.push_back(Prediction);
				
			}
			*/
			
			//MODEL INTEGRATION

			Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref) + Prediction;

			Torques_nom = Torques_ref;

			if (CycleCounter>=100) 
			{
				Torques_faulty = Controller.ExtTorque(Torques_nom, fault, Controller.Q, F);
				Torques_ref += Torques_faulty; 
			}

			//Kuka_temp = Controller.GetFriction(Controller.Q,Controller.dQ) * 0.01;

			//Temp_array.push_back(Kuka_temp);

			//Controller.d2Q = Controller.SimDynamicModelFake(Controller.Q, Controller.dQ, Torques_ref);

			Controller.d2Q = Controller.SimDynamicModel(Controller.Q, Controller.dQ, Torques_ref);

			//Reduced observer

			Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_ref);

			Controller.Qold = Controller.Q;

			Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

			Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

			Controller.r = Controller.Residual(Controller.Q, Controller.dQ, Torques_nom, Controller.r, CycleCounter);

			Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

			Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;

			Controller.r_ob = Controller.Residual_obs(Controller.Q, Controller.dQ_hat, Torques_nom, Controller.r_ob, CycleCounter);

			Controller.GetState(FLAG);

			/*
			if(CycleCounter > FILTER_LENGTH)
			{
				Controller.Regressor->DatasetUpdate(Controller.robot_state, Controller.old_robot_state, d2Q_ref, Prediction, Mass,Controller.d2Qsave,FLAG);
				Controller.Regressor->GpUpdate();
			}
			*/

			//ARRAY SAVING
			
			Controller.Qsave.push_back(Controller.Q);

			Controller.dQsave.push_back(Controller.dQ);

			Controller.d2Qsave.push_back(Controller.d2Q);

			Controller.dQ_hat_save.push_back(Controller.dQ_hat);

			Controller.r_save.push_back(Controller.r);

			Controller.r_obs_save.push_back(Controller.r_ob);

			Q_ref_vec.push_back(Q_ref);

			dQ_ref_vec.push_back(dQ_ref);

			d2Q_ref_vec.push_back(d2Q_ref);

			Controller.Tor_th.push_back(Torques_ref);

			fault_torque_vec.push_back(Torques_faulty);

			std::cout << "Step = " << CycleCounter << "\n";

			CycleCounter++;
	}

	//Writing Simulation Variables

		Controller.FromKukaToDyn(temp,Controller.Qsave);
		Controller.writer.write_data(qsave,temp);
		
		Controller.FromKukaToDyn(temp,Controller.dQsave);
		Controller.writer.write_data(dqsave,temp);

		Controller.FromKukaToDyn(temp,Controller.d2Qsave);
		Controller.writer.write_data(d2qsave,temp);

		Controller.FromKukaToDyn(temp,Controller.dQ_hat_save);
		Controller.writer.write_data(dqhatsave,temp);	

		Controller.FromKukaToDyn(temp,Controller.r_save);
		Controller.writer.write_data(rsave,temp);

		Controller.FromKukaToDyn(temp,Controller.r_obs_save);
		Controller.writer.write_data(r_obsave,temp);	

		Controller.FromKukaToDyn(temp,Controller.Tor_th);
		Controller.writer.write_data(torque_save,temp);	

		Controller.FromKukaToDyn(temp,fault_torque_vec);
		Controller.writer.write_data(fault_torque_save,temp);	

		Controller.FromKukaToDyn(temp,Q_ref_vec);
		Controller.writer.write_data(Q_ref_file,temp);

		Controller.FromKukaToDyn(temp,dQ_ref_vec);
		Controller.writer.write_data(dQ_ref_file,temp);

		Controller.FromKukaToDyn(temp,d2Q_ref_vec);
		Controller.writer.write_data(d2Q_ref_file,temp);
		
		//Controller.FromKukaToDyn(temp,Temp_array);
		//Controller.writer.write_data(friction,temp);

		//Controller.FromKukaToDyn(temp,Controller.Tor_th);
		//Controller.writer.write_data(torque_th,temp);
		
		//Controller.FromKukaToDyn(temp,Prediction_array);
		//Controller.writer.write_data(foo_pred,temp);
		

return 0;
}
