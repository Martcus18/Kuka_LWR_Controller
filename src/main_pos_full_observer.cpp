#include<controller/controller_kuka.hpp>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	unsigned int 				CollCounter     =   0;
	int							ResultValue		=	0;
	double 						Time = 0.0;
	double						frequency = 0.3;	

	std::chrono::time_point<std::chrono::system_clock> Tic, Toc;

	double elapsed_time;

	unsigned int TimeoutValueInMicroSeconds = 0;

	Kuka_Vec Q0;

	Kuka_Vec dQ0 = Kuka_Vec::Constant(NUMBER_OF_JOINTS,0.0);

	Kuka_Vec Q_ref;

	Kuka_Vec dQ_ref;

	Kuka_Vec d2Q_ref;

	Kuka_Vec Torques_ref;

	Kuka_Vec Torques_measured;

	Kuka_Vec torques_temp;

	Kuka_Vec temp_Vec;

	Kuka_Vec zero_vec = Kuka_Vec::Constant(0.0);

	Kuka_Vec Q_filtered;

	Kuka_Vec Q_stop;

	Kuka_Vec dQ_filtered;

	Kuka_Vec d2Q_filtered;

	std::vector<Kuka_Vec> Q_ref_vec;

	std::vector<Kuka_Vec> dQ_ref_vec;

	std::vector<Kuka_Vec> d2Q_ref_vec;

	std::vector<Kuka_Vec> Time_array;

	std::vector<Kuka_Vec> Temp_array;
	
	Kuka_Vec Kuka_temp;

	Kuka_Vec Kuka_temp2;

	std::vector<Eigen::VectorXd> temp;

	Kuka_Mat Mass;

	//std::string Mode("impedence");
	
	std::string Mode("position");
	
	std::string qsave = "Q.txt";
  	std::string dqsave = "dQ.txt";
	std::string d2qsave = "d2Q.txt";
	std::string qsave_filtered = "Q_filtered.txt";
  	std::string dqsave_filtered = "dQ_filtered.txt";
	std::string d2qsave_filtered = "d2Q_filtered.txt";
	std::string torque_meas = "tor_meas.txt";
	std::string torque_th = "tor_th.txt";
	std::string Q_ref_file = "Qref.txt";
	std::string dQ_ref_file = "dQref.txt";
	std::string d2Q_ref_file = "d2Qref.txt";
	std::string qhatsave = "Q_hat.txt";
	std::string dqhatsave = "dQ_hat.txt";
	std::string rsave = "res.txt";
	std::string r_obsave = "res_ob.txt";

	Kuka_State state;

	bool FLAG = ROBOT_CONTROL;

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

	//Collision detection variables

	bool coll = false;

	Q_stop = Q0;

	double time_res = 0.3;

	std::array<int,7> logic_flag;

	//Residual terms

	Kuka_Vec s1 = Kuka_Vec::Constant(0.0);
    Kuka_Vec s2 = Kuka_Vec::Constant(0.0);

	Kuka_Vec s1_ob = Kuka_Vec::Constant(0.0);
    Kuka_Vec s2_ob = Kuka_Vec::Constant(0.0);

	//Full-order observer variables

	Kuka_Vec y_tilda = Kuka_Vec::Constant(0.0);
	Kuka_Vec dx1_hat = Kuka_Vec::Constant(0.0);
	Kuka_Vec d2Q_hat = Kuka_Vec::Constant(0.0);

	//Initializing estimated varibales

	Controller.Q_hat = Controller.Q;
	Controller.dQ_hat = Kuka_Vec::Constant(0.0);
	Controller.dQ_hat_save.push_back(Controller.dQ_hat);
	Controller.Q_hat_save.push_back(Controller.Q_hat);

	//Initial generalized momentum

	Mass = Controller.GetMass(Controller.Q);
	Controller.p0 = Mass*Controller.dQ;
	Controller.p0_hat = Mass*Controller.dQ_hat;

	while ((float)CycleCounter * Controller.FRI->GetFRICycleTime() < RUN_TIME_IN_SECONDS)
	{	
		Time = Controller.FRI->GetFRICycleTime() * (float)CycleCounter;

		Q_ref = Q0 + Kuka_Vec::Constant(0.2*(1.0 - std::cos(Time)));
		
		dQ_ref = Kuka_Vec::Constant(0.2*std::sin(Time));
		
		d2Q_ref = Kuka_Vec::Constant(0.2*std::cos(Time));		
		
		Q_ref(4) = Q0(4);
		Q_ref(5) = Q0(5);
		Q_ref(6) = Q0(6);
		
		dQ_ref(4) = 0.0;
		dQ_ref(5) = 0.0;
		dQ_ref(6) = 0.0;

		d2Q_ref(4) = 0.0;
		d2Q_ref(5) = 0.0;
		d2Q_ref(6) = 0.0;

		//CHECK IF A COLLISION HAS OCCURRED

		//COLL = TRUE <-----> COLLISION	

		//Look if any of the components of the residual has exceeded the threshold, is so then coll = TRUE
		
		if (Time >= time_res && !coll)
		{	
			coll = std::any_of(logic_flag.begin(), logic_flag.end(), [](int i) { return i==1; });
			Q_stop = Controller.Q;
		}

		//If a collision has occurred and it has passed 1 second then coll = FALSE and the robot can return 
		//tracking the original sinusoidal reference in the joint positions
		
		if( (CollCounter >= 200) && coll)
		{
			coll = false;
			CollCounter = 0;
			time_res = Time + 0.9;
		}

		//If a collision has occurred and it has NOT passed 1 second then we keep as reference for the robot the 
		//value saved in Q_stop
		
		if( (CollCounter < 200) && coll)
		{
			CollCounter++;
			CycleCounter--;
		}

		Tic = std::chrono::system_clock::now();

		Controller.FRI->WaitForKRCTick(TimeoutValueInMicroSeconds);

		Toc = std::chrono::system_clock::now();

		if (!Controller.FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		//GetState takes the value of joint position from the encoders and then obtains joint velocities through Euler differentiation

		state = Controller.GetState(FLAG);

		//Saving robot state
		
		Controller.Qsave.push_back(Controller.Q);

		Controller.dQsave.push_back(Controller.dQ);

		//Filtering the variables and saving them

		Q_filtered = Controller.Filter(Controller.Qsave,FILTER_LENGTH);

		dQ_filtered = Controller.Filter(Controller.dQsave,FILTER_LENGTH);

		Controller.d2Q = Controller.EulerDifferentiation(dQ_filtered,Controller.dQsave_filtered.back());

		Controller.d2Qsave.push_back(Controller.d2Q);

		d2Q_filtered = Controller.Filter(Controller.d2Qsave,FILTER_LENGTH);

		Controller.Qsave_filtered.push_back(Q_filtered);
		
		Controller.dQsave_filtered.push_back(dQ_filtered);
		
		Controller.d2Qsave_filtered.push_back(d2Q_filtered);

		Controller.old_state_filtered = Controller.state_filtered;

		Controller.state_filtered << Q_filtered, dQ_filtered;

		//REACTION STRATEGY: if a collision has occurred then stop the robot at the position in which it is, i.e. the value saved in Q_stop
		
		if (coll)
		{
			std::cout << "Collision has occurred at time: " << Time << "\n";
			Q_ref = Q_stop;			
		}
		
		//Saving state reference values

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

		Temp_array.push_back(d2Q_ref);		
		
		//POSITION COMMANDING			

		for(int y=0;y<NUMBER_OF_JOINTS;y++)
		{
			Controller.JointValuesInRad[y] = Q_ref(y);
		}
		
		Controller.FRI->SetCommandedJointPositions(Controller.JointValuesInRad);

		//TORQUE MEASUREMENTS

		Controller.MeasureJointTorques();	
		
		Controller.torque_measured(0) = Controller.MeasuredTorquesInNm[0];
		Controller.torque_measured(1) = Controller.MeasuredTorquesInNm[1];
		Controller.torque_measured(2) = Controller.MeasuredTorquesInNm[2];
		Controller.torque_measured(3) = Controller.MeasuredTorquesInNm[3];
		Controller.torque_measured(4) = Controller.MeasuredTorquesInNm[4];
		Controller.torque_measured(5) = Controller.MeasuredTorquesInNm[5];
		Controller.torque_measured(6) = Controller.MeasuredTorquesInNm[6];

		Controller.Tor_meas.push_back(Controller.torque_measured);

		torques_temp = Controller.Filter(Controller.Tor_meas,FILTER_LENGTH);
		
		Controller.Tor_meas_filtered.push_back(torques_temp);

		//FULL-ORDER OBSERVER DYNAMICS
		
		y_tilda = Controller.Q - Controller.Q_hat;

		dx1_hat = Controller.dQ_hat + Controller.kd*y_tilda;

		d2Q_hat = Controller.SimObserver(Controller.Q, y_tilda, Controller.dQ_hat, torques_temp);

		Controller.Q_hat = Controller.EulerIntegration(dx1_hat, Controller.Q_hat);

		Controller.dQ_hat = Controller.EulerIntegration(d2Q_hat, Controller.dQ_hat);
		
		//EVALUATION OF THE RESIDUAL

		Controller.r = Controller.Residual(Controller.Q, Controller.dQ, torques_temp, Controller.r, CycleCounter, s1, s2, Controller.p0);

		Controller.r_ob = Controller.Residual(Controller.Q, Controller.dQ_hat, torques_temp, Controller.r_ob, CycleCounter, s1_ob, s2_ob, Controller.p0_hat);

		logic_flag = {0,0,0,0,0,0,0};

		logic_flag = Controller.collision(Controller.r_ob, Time);	

		std::cout << logic_flag[0] << logic_flag[1] << logic_flag[2] << logic_flag[3] << logic_flag[4] << logic_flag[5] << logic_flag[6] << std::endl;	

		//Controller.Tor_meas_filtered.push_back(Controller.torque_measured);

		Controller.r_save.push_back(Controller.r);

		Controller.r_obs_save.push_back(Controller.r_ob);

		Controller.dQ_hat_save.push_back(Controller.dQ_hat);

		Controller.Q_hat_save.push_back(Controller.Q_hat);

		CycleCounter++;

			
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

	//ROBOT STATE
	Controller.FromKukaToDyn(temp,Controller.Qsave);
	Controller.writer.write_data(qsave,temp);
	
	Controller.FromKukaToDyn(temp,Controller.dQsave);
	Controller.writer.write_data(dqsave,temp);

	Controller.FromKukaToDyn(temp,Controller.d2Qsave);
	Controller.writer.write_data(d2qsave,temp);
	
	//FILTERED ROBOT STATE
	Controller.FromKukaToDyn(temp,Controller.Qsave_filtered);
	Controller.writer.write_data(qsave_filtered,temp);

	Controller.FromKukaToDyn(temp,Controller.dQsave_filtered);
	Controller.writer.write_data(dqsave_filtered,temp);
	
	Controller.FromKukaToDyn(temp,Controller.d2Qsave_filtered);
	Controller.writer.write_data(d2qsave_filtered,temp);

	//ESTIMATED VELOCITIES
	Controller.FromKukaToDyn(temp,Controller.dQ_hat_save);
	Controller.writer.write_data(dqhatsave,temp);

	//ESTIMATED POSITIONS
	Controller.FromKukaToDyn(temp,Controller.Q_hat_save);
	Controller.writer.write_data(qhatsave,temp);

	//RESIDUALS
	Controller.FromKukaToDyn(temp,Controller.r_save);
	Controller.writer.write_data(rsave,temp);

	Controller.FromKukaToDyn(temp,Controller.r_obs_save);
	Controller.writer.write_data(r_obsave,temp);
	
	//TORQUE VARIABLES PRINTING
	Controller.FromKukaToDyn(temp,Controller.Tor_meas_filtered);
	Controller.writer.write_data(torque_meas,temp);	

	//REFERENCE TRAJECTORY PRINTING
	Controller.FromKukaToDyn(temp,Q_ref_vec);
	Controller.writer.write_data(Q_ref_file,temp);

	//DELETING POINTERS
	delete Controller.FRI;

	fprintf(stdout, "Objects deleted...\n");
	
	//return(EXIT_SUCCESS)

return 0;
}
