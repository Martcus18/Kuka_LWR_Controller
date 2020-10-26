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

	Kuka_State state;

	bool FLAG = SIM_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	std::cout << "Simulation LOOP" << "\n";

	Controller.Qold = Controller.Q;

	Q0 = Controller.Q;

	//SIMULATION LOOP
	while ((float)CycleCounter * DELTAT < 10)
	{
			Time = DELTAT * (float)CycleCounter;

			Mass = Controller.GetMass(Controller.Q);

			Q_ref = Q0 + Kuka_Vec::Constant(0.1*(1.0 - std::cos(Time)));
			
			dQ_ref = Kuka_Vec::Constant(0.1*std::sin(Time));
			
			d2Q_ref = Kuka_Vec::Constant(0.1*std::cos(Time));

			//d2Q_ref = d2Q_ref + Controller.PDController(Controller.Q, Controller.dQ, Controller.d2Q, Q_ref, dQ_ref , Controller.d2Q);
			
			/*
			if(CycleCounter > (2 * FILTER_LENGTH))
			{
				Prediction =  Controller.Regressor->GpPredict(Controller.Q,Controller.dQ,d2Q_ref);
				Prediction_array.push_back(Prediction);
				
			}
			*/
			//MODEL INTEGRATION

			Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref) + Prediction;

			Kuka_temp = Controller.GetFriction(Controller.Q,Controller.dQ) * 0.01;

			Temp_array.push_back(Kuka_temp);

			//Controller.d2Q = Controller.SimDynamicModelFake(Controller.Q, Controller.dQ, Torques_ref);

			Controller.d2Q = Controller.SimDynamicModel(Controller.Q, Controller.dQ, Torques_ref);

			Controller.Qold = Controller.Q;

			Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

			Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

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

			Q_ref_vec.push_back(Q_ref);

			dQ_ref_vec.push_back(dQ_ref);

			d2Q_ref_vec.push_back(d2Q_ref);

			Controller.Tor_th.push_back(Torques_ref);

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

		Controller.FromKukaToDyn(temp,Q_ref_vec);
		Controller.writer.write_data(Q_ref_file,temp);

		Controller.FromKukaToDyn(temp,dQ_ref_vec);
		Controller.writer.write_data(dQ_ref_file,temp);

		Controller.FromKukaToDyn(temp,d2Q_ref_vec);
		Controller.writer.write_data(d2Q_ref_file,temp);

		//Controller.writer.write_data(Xdata,Controller.Regressor->DatasetX);

		//Controller.writer.write_data(Ydata,Controller.Regressor->DatasetY);

		Controller.FromKukaToDyn(temp,Temp_array);
		Controller.writer.write_data(friction,temp);

		Controller.FromKukaToDyn(temp,Controller.Tor_th);
		Controller.writer.write_data(torque_th,temp);

		/*
		Controller.FromKukaToDyn(temp,Prediction_array);
		Controller.writer.write_data(foo_pred,temp);
		*/

return 0;
}
