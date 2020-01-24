#include<controller/controller_kuka.hpp>


int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;
	int							ResultValue		=	0;
	double 						Time = 0.0;
	    
	data_manager dataX;
    data_manager dataY;
	
	std::string Xtest = "Xtest3.txt";
    std::string Ytest = "Ytest3.txt";
	std::string Prediction = "Prediction.txt";
	
	std::vector<Eigen::VectorXd> X;
    std::vector<Eigen::VectorXd> Y;
	std::vector<Eigen::VectorXd> Out;
	
	dataX.read_data(Xtest,X,3);

    dataY.read_data(Ytest,Y,1);

	char net_path1[] = "/home/kuka_linux/Desktop/Kuka_Controller/external/Tensorflow/models/inverse_mapping/net_3.pb";
    char in_name1[] =  "KerasInput_input_2";
    char out_name1[] = "KerasOutput_2/BiasAdd";

    Network_Output output;
    Network_Input input;
	tf_network net(net_path1,in_name1,out_name1);

	for(std::vector<Eigen::VectorXd>::iterator it = X.begin() ; it != X.end(); ++it)
	{
		//std::cout << "---" << *it << "---\n";
		auto temp = *it;
		input(0) = temp(0);
		input(1) = temp(1);
		input(2) = temp(2);
		std::cout << "input_torque = " << temp(0) << "\n";
		output = net.predict(input);
		std::cout << "output2 = " << output << "\n";
		Out.push_back(output);
	}
	dataY.write_data(Prediction,Out);
	//std::chrono::time_point<std::chrono::system_clock> Tic, Toc;
return 0;
}
