#include<tensorflow/c/tf_network.hpp>


//FIND HOW TO PASS POINT OF A METHOD FOR FREE BUFFER E DEALLOCATOR
void free_buffer(void* data, size_t length) 
{
	free(data);
};

static void Deallocator(void* data, size_t length, void* arg) {};

TF_Buffer* tf_network::read_file(const char* file) 
{
	FILE *f = fopen(file, "rb");
	fseek(f, 0, SEEK_END);
	long fsize = ftell(f);
	fseek(f, 0, SEEK_SET);  //same as rewind(f);

	void* data = malloc(fsize);
	fread(data, fsize, 1, f);
	fclose(f);

	TF_Buffer* buf = TF_NewBuffer();
	buf->data = data;
	buf->length = fsize;
	buf->data_deallocator = free_buffer;
	return buf;
};

void tf_network::predict(float in[], float out[])
{
	std::vector<TF_Output> inputs;
	std::vector<TF_Tensor*> input_values;
	std::vector<TF_Output> outputs;
	std::vector<TF_Tensor*> output_values;

	//inputs.push_back({ TF_GraphOperationByName(graph, "KerasInput_input_25"), 0 });
	//outputs.push_back({ TF_GraphOperationByName(graph, "KerasOutput_24/BiasAdd"), 0 });
	
	//inputs.push_back({ TF_GraphOperationByName(graph, "KerasInput_input_26"), 0 });
	//outputs.push_back({ TF_GraphOperationByName(graph, "KerasOutput_25/BiasAdd"), 0 });	
	inputs.push_back({ TF_GraphOperationByName(graph, input_name), 0 });
	outputs.push_back({ TF_GraphOperationByName(graph, output_name), 0 });
	input_values.push_back(TF_NewTensor(TF_FLOAT, in_dims, 2, in, num_bytes_in, &Deallocator, 0));
	output_values.push_back(TF_AllocateTensor(TF_FLOAT, out_dims, 2, num_bytes_out));
	TF_SessionRun(session, nullptr,	&inputs[0], &input_values[0], 1, &outputs[0], &output_values[0], 1, nullptr, 0, nullptr, status);
	const auto data = static_cast<float*>(TF_TensorData(input_values.at(0)));
	const auto tau_data = static_cast<float*>(TF_TensorData(output_values.at(0)));
	std::memcpy(out, tau_data, sizeof(tau_data));
};