#include<controller/controller_kuka.hpp>
#include<random>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;
	double						frequency = 1.0;
	double 						tf = 10.0;
	double						damping = 0.1; // needed for pseudoinverse
	double						e = 1.0; // needed for pseudoinverse

	//random generator for the gaussian noise in the torque
	const double mean = 0.0;
    const double stddev = 0.1;
    std::default_random_engine generator;  //torque noise
	std::default_random_engine generator2; //position noise
    std::normal_distribution<double> dist(mean, stddev);

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

	std::cout << "Choose what external force you want to simulate: \n"; 
	std::cout << "0 --> nominal condition \n";
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

	Kuka_Vec Q_meas;

	Kuka_Vec Q_meas_old;

	Kuka_Vec dQ_meas; //obtained from the measured positions

	Kuka_Vec dQ_meas_old;

	Kuka_Vec d2Q_meas; //obtained from the measured positions

	Kuka_Vec G;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_nom;

	Kuka_Vec Torques_faulty = Kuka_Vec::Constant(0.0);

	Kuka_Vec Torques_measured;

	Kuka_Vec Torques_filtered;

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

	std::vector<Kuka_Vec> noisy_torque_vec;

	std::vector<Kuka_Vec> Time_array;

	std::vector<Kuka_Vec> Temp_array;
	
	Kuka_Vec Kuka_temp;

	Kuka_Vec Kuka_temp2;

	std::vector<Eigen::VectorXd> temp;

	Kuka_Mat Mass;

	Eigen::Vector3d end_effector;

	Eigen::Vector3d p_0 ;

	Eigen::Vector3d dp_0;

	Eigen::Vector3d d2p_0(0,0,0); //initial accelerations are zero 

	Eigen::Vector3d p_ref;

	Eigen::Vector3d dp_ref;

	Eigen::Vector3d d2p_ref;

	Eigen::Vector3d d2p_ff;

	Eigen::Vector3d p;

	Eigen::Vector3d dp;

	Eigen::Vector3d d2p;

	std::vector<Eigen::VectorXd> p_vec;

	Eigen::Vector3d err_p;

	Eigen::Vector3d err_dp;

	Eigen::MatrixXd J;

	Eigen::MatrixXd dJ;

	Eigen::MatrixXd Jpinv;

	std::string Mode("impedence");
	
	//std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string qsave_filtered = "Q_filtered.txt";
  	std::string dqsave_filtered = "dQ_filtered.txt";
	std::string d2qsave_filtered = "d2Q_filtered.txt";
	std::string dqsave_meas = "dQ_meas.txt";
	std::string end_effector_pos = "end_eff.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	std::string friction = "friction.txt";
	std::string Q_ref_file = "Qref.txt";
	std::string dQ_ref_file = "dQref.txt";
	std::string d2Q_ref_file = "d2Qref.txt";
	std::string dqhatsave = "dQ_hat.txt";
	std::string dqnumsave = "dQ_num.txt";
	std::string rsave = "res.txt";
	std::string r_obsave = "res_ob.txt";
	std::string torque_save = "torque_ref.txt";
	std::string torque_noisy_save = "torque_noisy.txt";
	std::string fault_torque_save = "torque_faulty.txt";
	std::string p_save = "p.txt";

	Eigen::Vector3d F(2,0,0);

	Kuka_State state;

	bool FLAG = SIM_CONTROL;

	controller_kuka Controller(Mode,FLAG);

	std::cout << "Simulation LOOP" << "\n";

	std::cout << "\n Remember to insert friction compensation for REAL robot experiment \n";

	Controller.Qold = Controller.Q;

	Q0 = Controller.Q;

	Q_meas = Q0;

	dQ_meas = Kuka_Vec::Constant(0.0);

	d2Q_meas = Kuka_Vec::Constant(0.0);

	Q_filtered = Kuka_Vec::Constant(0.0);

	dQ_filtered = Kuka_Vec::Constant(0.0);

	d2Q_filtered = Kuka_Vec::Constant(0.0);

	p_0 = Controller.DirKin(Controller.Q);
	
	dp_0 = Controller.Jac(Controller.Q)*Controller.dQ; 

	std::array<int,7> logic_flag = {0,0,0,0,0,0,0};

	bool coll = false;

	Kuka_Vec s1 = Kuka_Vec::Constant(0.0);
    Kuka_Vec s2 = Kuka_Vec::Constant(0.0);

	Kuka_Vec s1_ob = Kuka_Vec::Constant(0.0);
    Kuka_Vec s2_ob = Kuka_Vec::Constant(0.0);

	//reduced-order obs
	
	Controller.dQ_hat = Controller.k0 * Controller.Q;
	Controller.dQ_hat_save.push_back(Controller.dQ_hat);
	
	//SIMULATION LOOP
	while (Time < tf)
	{		

		if (Time>=1.5 && !coll)
		{	
			coll = std::any_of(logic_flag.begin(), logic_flag.end(), [](int i) { return i==1; });
			//std::cout << coll << std::endl;
		}
					
		//CIRCULAR TRAJECTORY

		p_ref[0] = p_0[0] - 0.1*(1.0-std::cos(frequency*Time));
		p_ref[1] = p_0[1] - 0.1*(std::sin(frequency*Time));
		p_ref[2] = p_0[2];
		
		dp_ref[0] = dp_0[0] - 0.1*frequency*std::sin(frequency*Time);
		dp_ref[1] = dp_0[1] - 0.1*frequency*std::cos(frequency*Time);
		dp_ref[2] = dp_0[2];

		d2p_ff[0] = d2p_0[0] - 0.1*frequency*frequency*std::cos(frequency*Time);
		d2p_ff[1] = d2p_0[1] + 0.1*frequency*frequency*std::sin(frequency*Time);
		d2p_ff[2] = d2p_0[2];

		//WE ADD THE NOISE DUE TO THE MEASUREMENTS THORUGH ENCODERS

		Q_meas_old = Q_meas;

		Q_meas(0) = Controller.Q(0) * (1.0 + 0.01*dist(generator2));
		Q_meas(1) = Controller.Q(1) * (1.0 + 0.01*dist(generator2));
		Q_meas(2) = Controller.Q(2) * (1.0 + 0.01*dist(generator2));
		Q_meas(3) = Controller.Q(3) * (1.0 + 0.01*dist(generator2));
		Q_meas(4) = Controller.Q(4) * (1.0 + 0.01*dist(generator2));
		Q_meas(5) = Controller.Q(5) * (1.0 + 0.01*dist(generator2));
		Q_meas(6) = Controller.Q(6) * (1.0 + 0.01*dist(generator2));


		dQ_meas_old = dQ_meas;

		dQ_meas = Controller.EulerDifferentiation(Q_meas, Q_meas_old);

		//d2Q_meas = Controller.EulerDifferentiation(dQ_meas, dQ_meas_old);

		//dQ_meas = Controller.dQ;

		d2Q_meas = Controller.d2Q;

		//filtering the varibles

		Controller.Qsave_meas.push_back(Q_meas);

		Controller.dQsave_meas.push_back(dQ_meas);

		Controller.d2Qsave_meas.push_back(d2Q_meas);

		Q_filtered = Controller.Filter(Controller.Qsave_meas,FILTER_LENGTH);

		dQ_filtered = Controller.Filter(Controller.dQsave_meas,FILTER_LENGTH);

		d2Q_filtered = Controller.Filter(Controller.d2Qsave_meas,FILTER_LENGTH); //maybe useless

		Controller.Qsave_filtered.push_back(Q_filtered);

		Controller.dQsave_filtered.push_back(dQ_filtered);

		Controller.d2Qsave_filtered.push_back(d2Q_filtered);

		//std::cout << ' Q = ' << Q_meas <<', dQ = ' << dQ_meas << ', d2Q = '<< d2Q_meas << std::endl;

		/*
		p = Controller.DirKin(Controller.Q);
		J = Controller.Jac(Controller.Q);
		dJ = Controller.diff_Jac(Controller.Q, Controller.dQ);

		dp = J * Controller.dQ;
		d2p = dJ * Controller.dQ + J * Controller.d2Q;

		err_p = p_ref -p;
		err_dp = dp_ref - dp;

		d2p_ref = Controller.Kp_cart * err_p + Controller.Kd_cart * err_dp + d2p_ff;

		// damped least square pseudo inverse

		Controller.dls_pinv(J, damping, e, Jpinv);

		d2Q_ref = Jpinv*(d2p_ref-dJ*Controller.dQ);
		*/

		p = Controller.DirKin(Q_filtered);
		J = Controller.Jac(Q_filtered);
		dJ = Controller.diff_Jac(Q_filtered, Controller.dQ);

		dp = J * Controller.dQ;
		d2p = dJ * Controller.dQ + J * d2Q_meas;

		err_p = p_ref -p;
		err_dp = dp_ref - dp;

		d2p_ref = Controller.Kp_cart * err_p + Controller.Kd_cart * err_dp + d2p_ff;

		// damped least square pseudo inverse

		Controller.dls_pinv(J, damping, e, Jpinv);

		d2Q_ref = Jpinv*(d2p_ref-dJ*Controller.dQ);
		
		//MODEL INTEGRATION

		//Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref);
		//Torques_ref = Controller.FeedbackLinearization(Q_meas, dQ_meas, d2Q_ref);
		Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_ref);

		Torques_nom = Torques_ref;

		if (Time>=5.0)
		{
			Torques_faulty = Controller.ExtTorque(Torques_nom, fault, Controller.Q, F);
			Torques_ref += Torques_faulty;
		}

		//Here we apply the reaction to a collision: we make stop the robot in a damped way
		
		/*if (coll)
		{
			std::cout << "Collision has occurred at time: " << Time << "\n";
			d2Q_ref = (-1.0)*Controller.K_damp*Controller.dQ;
			//d2Q_ref = (-1.0)*Controller.Kp*Controller.dQ;
			Torques_ref = Controller.FeedbackLinearization(Q_filtered, dQ_filtered, d2Q_ref);
			Torques_nom = Torques_ref;
			Torques_faulty = Controller.ExtTorque(Torques_nom, fault, Q_filtered, F);
			Torques_ref += Torques_faulty;
			Controller.d2Q = d2Q_ref;
		}
		
		/*
		if (coll==0)
		{
			std::cout << "Collision has occurred at time: " << Time << "\n";
			//d2Q_ref = (-1.0)*Controller.K_damp*Controller.dQ;
			d2Q_ref = (-1.0)*Controller.Kp*Controller.dQ;
			Torques_ref = Controller.FeedbackLinearization(Controller.Q, Controller.dQ, d2Q_ref);
			Torques_nom = Torques_ref;
			Torques_faulty = Controller.ExtTorque(Torques_nom, fault, Controller.Q, F);
			Torques_ref += Torques_faulty;
			Controller.d2Q = d2Q_ref;
		}
		*/
		
		//Addition of a gaussian noise in order to simulate better the noisy measurements of the 
		//torques through the robot sensors

		Torques_measured(0) = Torques_ref(0) * (1.0 + 0.005*dist(generator));
		Torques_measured(1) = Torques_ref(1) * (1.0 + 0.005*dist(generator));
		Torques_measured(2) = Torques_ref(2) * (1.0 + 0.005*dist(generator));
		Torques_measured(3) = Torques_ref(3) * (1.0 + 0.005*dist(generator));
		Torques_measured(4) = Torques_ref(4) * (1.0 + 0.005*dist(generator));
		Torques_measured(5) = Torques_ref(5) * (1.0 + 0.005*dist(generator));
		Torques_measured(6) = Torques_ref(6) * (1.0 + 0.005*dist(generator));
		

		Controller.Tor_meas.push_back(Torques_measured);

		Torques_filtered = Controller.Filter(Controller.Tor_meas,FILTER_LENGTH);

		Controller.Tor_meas_filtered.push_back(Torques_filtered);

		//Kuka_temp = Controller.GetFriction(Controller.Q,Controller.dQ) * 0.01;

		Controller.d2Q = Controller.SimDynamicModel(Controller.Q, Controller.dQ, Torques_ref);

		//std::cout << "d2Q: " << Controller.d2Q << "\n";

		//Reduced observer

		//Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_ref);
		//Controller.dz = Controller.SimReducedObserver(Controller.Q, Controller.dQ_hat, Torques_filtered);
		//Controller.dz = Controller.SimReducedObserver(Q_meas, Controller.dQ_hat, Torques_filtered);
		Controller.dz = Controller.SimReducedObserver(Q_filtered, Controller.dQ_hat, Torques_filtered);

		//Generalized coordinates

		Controller.Qold = Controller.Q;

		Controller.dQold = Controller.dQ;

		Controller.Q = Controller.EulerIntegration(Controller.dQ,Controller.Q);

		Controller.dQ = Controller.EulerIntegration(Controller.d2Q,Controller.dQ);

		Controller.dQ_num = Controller.EulerDifferentiation(Controller.Q,Controller.Qold);

		//Residual

		//Controller.r = Controller.Residual(Controller.Q, Controller.dQ, Torques_filtered, Controller.r, CycleCounter);
		//Controller.r = Controller.Residual(Controller.Q, Controller.dQ, Torques_measured, Controller.r, CycleCounter);
		//Controller.r = Controller.Residual(Q_meas, dQ_meas, Torques_filtered, Controller.r, CycleCounter);
		Controller.r = Controller.Residual(Q_filtered, dQ_filtered, Torques_nom, Controller.r, CycleCounter, s1, s2, Controller.p0);

		Controller.z = Controller.EulerIntegration(Controller.dz, Controller.z);

		//Controller.dQ_hat = Controller.z + Controller.k0*Controller.Q;
		//Controller.dQ_hat = Controller.z + Controller.k0*Q_meas;
		Controller.dQ_hat = Controller.z + Controller.k0*Q_filtered;

		//For the first second we use as velocities the ones obtained through numeric derivation and then we use the ones obtained thourgh the observer so to avoid the inial peak
		//in the residual that influences the check if a collision has occurred or not by compring the residual with a constant threshold

		if (Time >= 1.0)
		{
			//Controller.r_ob = Controller.Residual(Controller.Q, Controller.dQ_hat, Torques_filtered, Controller.r_ob, CycleCounter);
			//Controller.r_ob = Controller.Residual(Q_meas, Controller.dQ_hat, Torques_filtered, Controller.r_ob, CycleCounter);
			Controller.r_ob = Controller.Residual(Q_filtered, Controller.dQ_hat, Torques_nom, Controller.r_ob, CycleCounter, s1_ob, s2_ob, Controller.p0);	
		}
		else
		{
			//Controller.r_ob = Controller.Residual(Controller.Q, Controller.dQ_num, Torques_filtered, Controller.r_ob, CycleCounter);
			//Controller.r_ob = Controller.Residual(Q_meas, dQ_meas, Torques_filtered, Controller.r_ob, CycleCounter);
			Controller.r_ob = Controller.Residual(Q_filtered, dQ_filtered, Torques_nom, Controller.r_ob, CycleCounter, s1_ob, s2_ob, Controller.p0); 
		}

		Controller.GetState(FLAG); 

		//CHECKING IF A FAULT/COLLISION HAS OCCURED (COMPARISON WITH THE THRESHOLDS)

		logic_flag = Controller.collision(Controller.r_ob, Time);

		std::cout << logic_flag[0] << logic_flag[1] << logic_flag[2] << logic_flag[3] << logic_flag[4] << logic_flag[5] << logic_flag[6] << std::endl;

		//ARRAY SAVING
		
		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);

		Controller.d2Qsave.push_back(Controller.d2Q);

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.dQ_num_save.push_back(Controller.dQ_num);

		Controller.r_save.push_back(Controller.r);

		Controller.r_obs_save.push_back(Controller.r_ob);

		Q_ref_vec.push_back(Q_ref);

		dQ_ref_vec.push_back(dQ_ref);

		d2Q_ref_vec.push_back(d2Q_ref);

		noisy_torque_vec.push_back(Torques_measured);

		Controller.Tor_th.push_back(Torques_ref);

		fault_torque_vec.push_back(Torques_faulty);

		p_vec.push_back(p);

		std::cout << "-------Step = " << CycleCounter << "\n";

		CycleCounter++;

		Time += DELTAT;
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

		Controller.FromKukaToDyn(temp,Controller.dQ_num_save);
		Controller.writer.write_data(dqnumsave,temp);	

		Controller.FromKukaToDyn(temp,Controller.r_save);
		Controller.writer.write_data(rsave,temp);

		Controller.FromKukaToDyn(temp,Controller.r_obs_save);
		Controller.writer.write_data(r_obsave,temp);	

		Controller.FromKukaToDyn(temp,Controller.Tor_th);
		Controller.writer.write_data(torque_save,temp);

		Controller.FromKukaToDyn(temp,noisy_torque_vec);
		Controller.writer.write_data(torque_noisy_save,temp);	

		Controller.FromKukaToDyn(temp,fault_torque_vec);
		Controller.writer.write_data(fault_torque_save,temp);	

		Controller.FromKukaToDyn(temp,Q_ref_vec);
		Controller.writer.write_data(Q_ref_file,temp);

		Controller.FromKukaToDyn(temp,dQ_ref_vec);
		Controller.writer.write_data(dQ_ref_file,temp);

		Controller.FromKukaToDyn(temp,d2Q_ref_vec);
		Controller.writer.write_data(d2Q_ref_file,temp);

		Controller.writer.write_data(p_save,p_vec);

		//"Measured" velocities

		Controller.FromKukaToDyn(temp,Controller.dQsave_meas);
		Controller.writer.write_data(dqsave_meas,temp);

		//Filtered variables

		Controller.FromKukaToDyn(temp,Controller.Qsave_filtered);
		Controller.writer.write_data(qsave_filtered,temp);

		Controller.FromKukaToDyn(temp,Controller.dQsave_filtered);
		Controller.writer.write_data(dqsave_filtered,temp);

		Controller.FromKukaToDyn(temp,Controller.d2Qsave_filtered);
		Controller.writer.write_data(d2qsave_filtered,temp);
		

return 0;
}
