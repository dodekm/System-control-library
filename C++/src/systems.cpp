#include "systems.h"

using namespace SystemControl;

void System::assign_port_pointers(real_t* input_port_ptr, real_t* output_port_ptr) {

	signal_realtime_T signal_struct = { 0 };
	signal_struct.owner = this;
	if (input_port_ptr != NULL) {
		input_port.assert();
		for (uint i = 0; i < input_port.get_length(); i++) {
			signal_struct.ptr = input_port_ptr + i;
			input_port.at(i) = signal_struct;
		}
	}
	if (output_port_ptr != NULL) {
		output_port.assert();
		for (uint i = 0; i < output_port.get_length(); i++) {
			signal_struct.ptr = output_port_ptr + i;
			output_port.at(i) = signal_struct;
		}
	}

}

System::~System() {

}

System::System(system_type system_type, system_time_domain time_domain, size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr) :
		input_port(input_port_width), output_port(output_port_width) {
	this->type = system_type;
	this->time_domain = time_domain;
	assign_port_pointers(input_port_ptr, output_port_ptr);
}

SubSystem SubSystem::connect_serial(Vector<System*>& systems_list, Vector<uint>& input_port_numbers, Vector<uint>& output_port_numbers) {

	systems_list.assert();
	input_port_numbers.assert();
	output_port_numbers.assert();

#ifdef ASSERT_DIMENSIONS
	if (input_port_numbers.get_length() != systems_list.get_length() || output_port_numbers.get_length() != systems_list.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	size_t n_systems = systems_list.get_length();
	SubSystem subsystem(system_type_subsystem, system_time_domain_uninitialized, 1, NULL, 1, NULL);

	for (uint i = 0; i < n_systems - 1; i++) {
#ifdef ASSERT_DIMENSIONS
		if (systems_list[i + 1]->input_port_size() <= input_port_numbers[i + 1])
			throw exception_WRONG_DIMENSIONS;
		if (systems_list[i + 1]->output_port_size() <= output_port_numbers[i + 1])
			throw exception_WRONG_DIMENSIONS;
#endif

		systems_list[i + 1]->input_port_struct(input_port_numbers[i + 1]).owner->input_port_struct(input_port_numbers[i + 1]) = systems_list[i]->output_port_struct(output_port_numbers[i]);
		systems_list[i + 1]->input_port_struct(input_port_numbers[i + 1]) = systems_list[i]->output_port_struct(output_port_numbers[i]);
		subsystem.time_domain = static_cast<System::system_time_domain>(subsystem.time_domain | systems_list[i]->time_domain);
		subsystem.execution_list.push_back(systems_list[i]);
	}
	subsystem.execution_list.push_back(systems_list[systems_list.get_length() - 1]);
	subsystem.input_port_struct(0) = systems_list[0]->input_port_struct(input_port_numbers[0]);
	subsystem.output_port_struct(0) = systems_list[n_systems - 1]->output_port_struct(output_port_numbers[n_systems - 1]);
	return subsystem;
}

SubSystem SubSystem::connect_paralel(Vector<System*>&systems_list, System& system_to) {

	systems_list.assert();
	size_t n_systems = systems_list.get_length();

#ifdef ASSERT_DIMENSIONS
	if (system_to.input_port_size() != n_systems || system_to.output_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	bool common_input = true;
	signal_realtime_T common_input_signal = systems_list[0]->input_port_struct(0);
	for (uint i = 0; i < n_systems; i++) {
		if (systems_list[i]->input_port_struct(0).owner != common_input_signal.owner || systems_list[i]->input_port_struct(0).ptr != common_input_signal.ptr) {
			common_input = false;
			break;
		}
	}
	SubSystem subsystem(system_type_subsystem, system_to.time_domain, common_input ? 1 : n_systems, NULL, 1, NULL);

	for (uint i = 0; i < n_systems; i++) {
#ifdef ASSERT_DIMENSIONS
		if (systems_list[i]->input_port_size() != 1 || systems_list[i]->output_port_size() != 1)
			throw exception_WRONG_DIMENSIONS;
#endif

		system_to.input_port_struct(i).owner->input_port_struct(i) = systems_list[i]->output_port_struct(0);
		system_to.input_port_struct(i) = systems_list[i]->output_port_struct(0);
		subsystem.time_domain = static_cast<System::system_time_domain>(subsystem.time_domain | systems_list[i]->time_domain);
		subsystem.execution_list.push_back(systems_list[i]);

		if (common_input == false)
			subsystem.input_port_struct(i) = systems_list[i]->input_port_struct(0);
	}
	subsystem.execution_list.push_back(&system_to);

	if (common_input == true)
		subsystem.input_port_struct(0) = common_input_signal;

	subsystem.output_port_struct(0) = system_to.output_port_struct(0);
	return subsystem;
}

SubSystem SubSystem::connect_feedback(System& operator_system, System& feedDirect_system, System& feedBack_system) {

#ifdef ASSERT_DIMENSIONS
	if (operator_system.input_port_size() != 2)
		throw exception_WRONG_DIMENSIONS;
	if (operator_system.output_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
	if (feedDirect_system.input_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
	if (feedDirect_system.output_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
	if (feedBack_system.input_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
	if (feedBack_system.input_port_size() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	feedDirect_system.input_port_struct(0).owner->input_port_struct(0) = operator_system.output_port_struct(0);
	feedDirect_system.input_port_struct(0) = operator_system.output_port_struct(0);

	operator_system.input_port_struct(1).owner->input_port_struct(1) = feedBack_system.output_port_struct(0);
	operator_system.input_port_struct(1) = feedBack_system.output_port_struct(0);

	feedBack_system.input_port_struct(0).owner->input_port_struct(0) = feedDirect_system.output_port_struct(0);
	feedBack_system.input_port_struct(0) = feedDirect_system.output_port_struct(0);

	SubSystem subsystem(system_type_subsystem, static_cast<System::system_time_domain>(feedDirect_system.time_domain | feedBack_system.time_domain | operator_system.time_domain), 1, NULL, 1, NULL);

	subsystem.input_port_struct(0) = operator_system.input_port_struct(0);
	subsystem.output_port_struct(0) = feedDirect_system.output_port_struct(0);

	subsystem.execution_list.push_back(&feedBack_system);
	subsystem.execution_list.push_back(&operator_system);
	subsystem.execution_list.push_back(&feedDirect_system);

	return subsystem;
}

void SubSystem::update(real_t time,step_type step_type) {
	auto lambda = [&](System* system_generic,uint i)->auto {
		if (system_generic == NULL)
		throw exception_NULLPTR;
		system_generic->update(time,step_type);
		return true;
	};
	execution_list.for_each(lambda);
}

void StateSpace::assert() const {
	A.assert();
	B.assert();
	C.assert();
#ifdef ASSERT_DIMENSIONS
	if (!A.is_square())
		throw exception_WRONG_DIMENSIONS;
	if (B.get_length() != A.get_n_rows() || C.get_length() != A.get_n_rows())
		throw exception_WRONG_DIMENSIONS;
#endif

}

void StateSpace::coeffs_set(const real_t* A_data_ptr, const real_t* B_data_ptr, const real_t* C_data_ptr) {

	if (A_data_ptr != NULL)
		A.load_data(A_data_ptr);
	if (B_data_ptr != NULL)
		B.load_data(B_data_ptr);
	if (C_data_ptr != NULL)
		C.load_data(C_data_ptr);
}

void TransferFunction::assert() const {
	numerator_coeffs.assert();
	denominator_coeffs.assert();
}

void TransferFunction::coeffs_set(const real_t* numerator_coeffs_ptr, const real_t* denominator_coeffs_ptr) {

	if (numerator_coeffs_ptr != NULL)
		numerator_coeffs.load_data(numerator_coeffs_ptr);

	if (denominator_coeffs_ptr != NULL)
		denominator_coeffs.load_data(denominator_coeffs_ptr);

}

void ZeroPoleGain::assert() const {
	zeros.assert();
	poles.assert();
}

