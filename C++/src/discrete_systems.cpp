#include "discrete_systems.h"
#include "wrappers.h"

using namespace SystemControl;
using namespace Convert;

DiscreteSystem::DiscreteSystem(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr) :
		System(system_type_dynamic_system, system_time_domain_discrete, input_port_width, input_port_ptr, output_port_width, output_port_ptr) {
}

real_t DiscreteSystem::step(real_t u) {

	real_t y = 0;
	signal_realtime_T input_signal, output_signal;
	input_signal.owner = NULL;
	input_signal.ptr = &u;
	output_signal.owner = NULL;
	output_signal.ptr = &y;

	const Vector<signal_realtime_T> input_signals(1, &input_signal);
	Vector<signal_realtime_T> output_signals(1, &output_signal);
	step_function(input_signals, output_signals);
	return y;
}

DiscreteSystemSISO::DiscreteSystemSISO() :
		DiscreteSystem(1, &input, 1, &output) {
	input = 0;
	output = 0;
}

DiscreteSystemMIMO::DiscreteSystemMIMO(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr) :
		DiscreteSystem(input_port_width, input_port_ptr, output_port_width, output_port_ptr) {
	if (input_port_ptr != NULL)
		memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	if (output_port_ptr != NULL)
		memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);
}

DiscreteTransferFunction::DiscreteTransferFunction(size_t numerator_order, size_t denominator_order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* input_states_ptr, real_t* output_states_ptr) :
		TransferFunction(numerator_order, denominator_order, System::system_time_domain_discrete, numerator_coeffs_ptr, denominator_coeffs_ptr), input_states(numerator_order + 1, input_states_ptr), output_states(denominator_order + 1, output_states_ptr) {
	states_set(0.0, 0.0);
}
void DiscreteTransferFunction::states_set(const real_t* input_states_ptr, const real_t* output_states_ptr) {

	if (input_states_ptr != NULL) {
		input_states.load_data(input_states_ptr);
		input_states.reset();
	}
	if (output_states_ptr != NULL) {
		output_states.load_data(output_states_ptr);
		output_states.reset();
	}
}
void DiscreteTransferFunction::states_set(real_t input_states_value, real_t output_states_value) {

	input_states.set_all(input_states_value);
	input_states.reset();
	output_states.set_all(output_states_value);
	output_states.reset();
}

void DiscreteTransferFunction::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;

	real_t y = 0;
	input_states.sample(u);

	for (uint i = 0; i < numerator_coeffs.get_length(); i++) {
		y += numerator_coeffs[i] * input_states[i];
	}
	input_states.shift();

	size_t order =get_order();
	for (uint i = 0; i < order; i++) {
		y += -denominator_coeffs[i] * output_states[i];
	}
	y /= denominator_coeffs[order];

	y = saturate(y);
	output_states.sample(y);
	output_states.shift();
	*y_ptr = y;

}

DiscreteStateSpace::DiscreteStateSpace(size_t order, real_t* A_data_ptr, real_t* B_data_ptr, real_t* C_data_ptr, real_t* Xk_0_data_ptr, real_t* Xk_1_data_ptr) :
		StateSpace(order, System::system_time_domain_discrete, A_data_ptr, B_data_ptr, C_data_ptr), Xk_0(order, Xk_0_data_ptr), Xk_1(order, Xk_1_data_ptr) {
	states_set(0.0);
}
void DiscreteStateSpace::states_set(const real_t* X_ptr) {
	Xk_1.load_data(X_ptr);
}
void DiscreteStateSpace::states_set(real_t X_values) {
	Xk_1.set_all(X_values);
}

void DiscreteStateSpace::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;
	real_t y = 0;

	Xk_0 = Xk_1;

#ifdef USE_GSL

	const gsl_matrix A_gsl = A.to_gsl_matrix();
	const gsl_vector B_gsl = B.to_gsl_vector();
	const gsl_vector C_gsl = C.to_gsl_vector();

	const gsl_vector Xk_0_gsl = Xk_0.to_gsl_vector();
	gsl_vector Xk_1_gsl = Xk_1.to_gsl_vector();

	exception_code returnval = exception_OK;
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1.0, &A_gsl, &Xk_0_gsl, 0.0, &Xk_1_gsl));
	if (returnval != exception_OK)
		throw returnval;
	returnval = gsl_error_code_to_return_code(gsl_blas_daxpy(u, &B_gsl, &Xk_1_gsl));
	if (returnval != exception_OK)
		throw returnval;
	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&C_gsl, &Xk_0_gsl, &y));
	if (returnval != exception_OK)
		throw returnval;

#else
	VectorReal2colMatrix(Xk_1).multiply(A, VectorReal2colMatrix(Xk_0));
	VectorReal2colMatrix(Xk_1).multiply_by_scalar_and_accumulate(VectorReal2colMatrix(B), u);
	y=C.dot_product(Xk_0);

#endif
	y = saturate(y);
	*y_ptr = y;

}

Discrete_RST_Regulator::Discrete_RST_Regulator(size_t R_order, size_t S_order, size_t T_order, real_t* R_coeffs_ptr, real_t* S_coeffs_ptr, real_t* T_coeffs_ptr, real_t* u_states_ptr, real_t* y_states_ptr, real_t* w_states_ptr) :
		DiscreteSystemMIMO(2, input_signals_data, 1, output_signals_data), R_coeffs(R_order + 1, R_coeffs_ptr), S_coeffs(S_order + 1, S_coeffs_ptr), T_coeffs(T_order + 1, T_coeffs_ptr), u_states(R_order + 1, u_states_ptr), y_states(S_order + 1, y_states_ptr), w_states(T_order + 1, w_states_ptr) {
	states_set(0.0, 0.0, 0.0);
}

void Discrete_RST_Regulator::states_set(const real_t* u_states_ptr, const real_t* y_states_ptr, const real_t* w_states_ptr) {

	if (u_states_ptr != NULL) {
		u_states.load_data(u_states_ptr);
		u_states.reset();
	}
	if (y_states_ptr != NULL) {
		y_states.load_data(y_states_ptr);
		y_states.reset();
	}
	if (w_states_ptr != NULL) {
		w_states.load_data(w_states_ptr);
		w_states.reset();
	}

}
void Discrete_RST_Regulator::states_set(real_t u_states_value, real_t y_states_value, real_t w_states_value) {

	u_states.set_all(u_states_value);
	u_states.reset();
	y_states.set_all(y_states_value);
	y_states.reset();
	w_states.set_all(w_states_value);
	w_states.reset();
}

void Discrete_RST_Regulator::coeffs_set(const real_t* R_coeffs_ptr, const real_t* S_coeffs_ptr, const real_t* T_coeffs_ptr) {

	if (R_coeffs_ptr != NULL) {
		R_coeffs.load_data(R_coeffs_ptr);
	}
	if (S_coeffs_ptr != NULL) {
		S_coeffs.load_data(S_coeffs_ptr);
	}
	if (T_coeffs_ptr != NULL) {
		T_coeffs.load_data(T_coeffs_ptr);
	}
}

void Discrete_RST_Regulator::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 2 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t w = *input_signals.at(0).ptr;
	const real_t y = *input_signals.at(1).ptr;
	real_t* u_ptr = output_signals.at(0).ptr;

	w_states.sample(w);
	real_t u = 0;
	for (uint i = 0; i < T_coeffs.get_length(); i++) {
		u += T_coeffs[i] * w_states[i];
	}
	w_states.shift();

	y_states.sample(y);
	for (uint i = 0; i < S_coeffs.get_length(); i++) {
		u += -S_coeffs[i] * y_states[i];
	}
	y_states.shift();

	for (uint i = 0; i < R_coeffs.get_length() - 1; i++) {
		u += -R_coeffs[i] * u_states[i];
	}
	u /= R_coeffs[R_coeffs.get_length() - 1];

	u = saturate(u);
	u_states.sample(u);
	u_states.shift();
	*u_ptr = u;
}

SubSystem Discrete_RST_Regulator::create_control_loop(DiscreteSystemSISO& system) {

	{

		uint input_port_numbers[] = { 0, 1 };
		Vector<uint> vec_input_port_numbers(2, input_port_numbers);
		uint output_port_numbers[] = { 0, 0 };
		Vector<uint> vec_output_port_numbers(2, output_port_numbers);
		System* systems_list[] = { &system, this };
		Vector<System*> vec_systems_list(2, systems_list);
		SubSystem::connect_serial(vec_systems_list, vec_input_port_numbers, vec_output_port_numbers);
	}

	uint input_port_numbers[] = { 0, 0 };
	Vector<uint> vec_input_port_numbers(2, input_port_numbers);
	uint output_port_numbers[] = { 0, 0 };
	Vector<uint> vec_output_port_numbers(2, output_port_numbers);
	System* systems_list[] = { this, &system };
	Vector<System*> vec_systems_list(2, systems_list);
	return SubSystem::connect_serial(vec_systems_list, vec_input_port_numbers, vec_output_port_numbers);

}

DiscreteIIRfilterDFII::DiscreteIIRfilterDFII(size_t order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* states_ptr) :
		TransferFunction(order, order, System::system_time_domain_discrete, numerator_coeffs_ptr, denominator_coeffs_ptr), states(order + 1, states_ptr) {
	states_set(0.0);
}
void DiscreteIIRfilterDFII::states_set(const real_t* states_ptr) {
	states.load_data(states_ptr);
	states.reset();
}
void DiscreteIIRfilterDFII::states_set(real_t states_value) {
	states.set_all(states_value);
	states.reset();
}

void DiscreteIIRfilterDFII::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;
	real_t v = u;

	size_t order = get_order();
	for (uint i = 0; i < order; i++) {
		v += -denominator_coeffs[i] * states[i];
	}
	v /= denominator_coeffs[order];
	states.sample(v);

	real_t y = 0;
	for (uint i = 0; i < order + 1; i++) {
		y += numerator_coeffs[i] * states[i];
	}
	states.shift();
	y = saturate(y);
	*y_ptr = y;

}

DiscreteFIRfilter::DiscreteFIRfilter(size_t order, real_t* coeffs_ptr, real_t* states_ptr) :
		coeffs(order + 1, coeffs_ptr), states(order + 1, states_ptr) {
	states_set(0.0);
}
void DiscreteFIRfilter::states_set(const real_t* states_ptr) {

	states.load_data(states_ptr);
	states.reset();

}
void DiscreteFIRfilter::states_set(real_t states_value) {

	states.set_all(states_value);
	states.reset();
}

void DiscreteFIRfilter::coeffs_set(const real_t* coeffs_ptr) {
	coeffs.load_data(coeffs_ptr);

}

void DiscreteFIRfilter::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;

	states.sample(u);
	real_t y = 0;

	for (uint i = 0; i < coeffs.get_length(); i++) {
		y += coeffs[i] * states[i];
	}
	states.shift();
	y = saturate(y);
	*y_ptr = y;

}

DiscreteDelay::DiscreteDelay(size_t order, real_t* states_ptr) :
		states(order + 1, states_ptr) {
	states.set_all(0);
}

void DiscreteDelay::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;
	states.sample(u);
	real_t y = states[0];
	states.shift();
	*y_ptr = y;

}

static real_t discrete_biquad_section_DF_I_step(discrete_biquad_section_states_DF_I_T* filter_states, discrete_biquad_section_coeffs_T* filter_coeffs, real_t u) {

	real_t y = u * filter_coeffs->b_coeffs[0] + filter_states->input_states[0] * filter_coeffs->b_coeffs[1] + filter_states->input_states[1] * filter_coeffs->b_coeffs[2];
	y -= filter_states->output_states[0] * filter_coeffs->a_coeffs[1] + filter_states->output_states[1] * filter_coeffs->a_coeffs[2];
	y /= filter_coeffs->a_coeffs[0];

	filter_states->input_states[1] = filter_states->input_states[0];
	filter_states->input_states[0] = u;
	filter_states->output_states[1] = filter_states->output_states[0];
	filter_states->output_states[0] = y;
	return y;
}

DiscreteBiquadSOSfilterDFI::DiscreteBiquadSOSfilterDFI(size_t n_stages, discrete_biquad_section_coeffs_T* coeffs_ptr, discrete_biquad_section_states_DF_I_T* states_ptr) :
		coeffs(n_stages, coeffs_ptr), states(n_stages, states_ptr) {
	discrete_biquad_section_states_DF_I states_value = { 0 };
	states.set_all(states_value);
}

void DiscreteBiquadSOSfilterDFI::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;
	size_t n_stages = states.get_length();

	real_t y = 0;
	for (uint i = 0; i < n_stages; i++) {
		y = discrete_biquad_section_DF_I_step(states.get_data_ptr() + i, coeffs.get_data_ptr() + i, u);
		u = y;
	}

	*y_ptr = y;
}

static real_t discrete_biquad_section_DF_II_step(discrete_biquad_section_states_DF_II_T* filter_states, discrete_biquad_section_coeffs_T* filter_coeffs, real_t u) {

	real_t v = u - filter_states->v_states[0] * filter_coeffs->a_coeffs[1] - filter_states->v_states[1] * filter_coeffs->a_coeffs[2];
	v /= filter_coeffs->a_coeffs[0];
	real_t y = v * filter_coeffs->b_coeffs[0] + filter_states->v_states[0] * filter_coeffs->b_coeffs[1] + filter_states->v_states[1] * filter_coeffs->b_coeffs[2];

	filter_states->v_states[1] = filter_states->v_states[0];
	filter_states->v_states[0] = v;
	return y;
}

DiscreteBiquadSOSfilterDFII::DiscreteBiquadSOSfilterDFII(size_t n_stages, discrete_biquad_section_coeffs_T* coeffs_ptr, discrete_biquad_section_states_DF_II_T* states_ptr) :
		coeffs(n_stages, coeffs_ptr), states(n_stages, states_ptr) {
	discrete_biquad_section_states_DF_II states_value = { 0 };
	states.set_all(states_value);
}

void DiscreteBiquadSOSfilterDFII::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	real_t u = *input_signals.at(0).ptr;
	real_t* y_ptr = output_signals.at(0).ptr;
	size_t n_stages = states.get_length();

	real_t y = 0;
	for (uint i = 0; i < n_stages; i++) {
		y = discrete_biquad_section_DF_II_step(states.get_data_ptr() + i, coeffs.get_data_ptr() + i, u);
		u = y;
	}

	*y_ptr = y;

}

DiscreteSumator::DiscreteSumator(real_t initial_condition) :
		DiscreteTransferFunctionFirstOrder(0, 1, numerator_coeffs_data, denominator_coeffs_data, input_states_data, output_states_data) {
	numerator_coeffs_data[0] = 1.0;
	denominator_coeffs_data[1] = 1.0;
	denominator_coeffs_data[0] = -1.0;

	real_t output_initial_states[] = { initial_condition, initial_condition };
	states_set(NULL, output_initial_states);
}

DiscreteDiference::DiscreteDiference(real_t initial_condition) :
		DiscreteTransferFunctionFirstOrder(1, 0, numerator_coeffs_data, denominator_coeffs_data, input_states_data, output_states_data) {

	numerator_coeffs_data[1] = 1.0;
	numerator_coeffs_data[0] = -1.0;
	denominator_coeffs_data[0] = 1.0;

	real_t input_initial_states[] = { initial_condition, initial_condition };
	states_set(input_initial_states, NULL);
}

DiscretePSDregulator::DiscretePSDregulator(real_t P_gain, real_t S_gain, real_t D_gain) :
		sumator(0), diference(0) {
	this->P_gain = P_gain;
	this->S_gain = S_gain;
	this->D_gain = D_gain;

}

void DiscretePSDregulator::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t e = *input_signals.at(0).ptr;
	real_t* u_ptr = output_signals.at(0).ptr;

	real_t u = 0;
	real_t u_P = 0;
	real_t u_S = 0;
	real_t u_D = 0;

	if (S_gain != (real_t)0) {
		sumator.input_port_real_value(0) = e + get_AW_correction();
		sumator.step();
		u_S = S_gain * sumator.output_port_real_value(0);
	}
	if (D_gain != (real_t)0) {
		diference.input_port_real_value(0) = e;
		diference.step();
		u_D = D_gain * diference.output_port_real_value(0);
	}
	if (P_gain != (real_t)0) {
		u_P = P_gain * e;
	}
	u = u_P + u_S + u_D;
	u = apply_AW_saturation(u);
	*u_ptr = u;

}

DiscreteIntegrator::DiscreteIntegrator(approximation_method method, real_t sample_time, real_t initial_condition) :
		DiscreteTransferFunctionFirstOrder(1, 1, numerator_coeffs_data, denominator_coeffs_data, input_states_data, output_states_data) {

	denominator_coeffs_data[1] = 1.0;
	denominator_coeffs_data[0] = -1.0;

	if (method == approximation_method_backward_euler) {
		numerator_coeffs_data[1] = sample_time;
		numerator_coeffs_data[0] = 0;
	} else if (method == approximation_method_forward_euler) {
		numerator_coeffs_data[1] = 0;
		numerator_coeffs_data[0] = sample_time;
	} else if (method == approximation_method_trapezoidal_rule) {
		numerator_coeffs_data[1] = sample_time / 2.0;
		numerator_coeffs_data[0] = sample_time / 2.0;
	} else {
		throw exception_ERROR;
	}
	real_t output_initial_states[] = { initial_condition, initial_condition };
	states_set(NULL, output_initial_states);

}

DiscreteDerivative::DiscreteDerivative(approximation_method method, real_t sample_time, real_t N_gain, real_t initial_condition) :
		DiscreteTransferFunctionFirstOrder(1, 1, numerator_coeffs_data, denominator_coeffs_data, input_states_data, output_states_data) {

	if (method == approximation_method_backward_euler) {
		numerator_coeffs_data[1] = N_gain;
		numerator_coeffs_data[0] = -N_gain;
		denominator_coeffs_data[1] = N_gain * sample_time + 1.0;
		denominator_coeffs_data[0] = -1.0;
	} else if (method == approximation_method_forward_euler) {
		numerator_coeffs_data[1] = N_gain;
		numerator_coeffs_data[0] = -N_gain;
		denominator_coeffs_data[1] = 1.0;
		denominator_coeffs_data[0] = N_gain * sample_time - 1.0;
	} else if (method == approximation_method_trapezoidal_rule) {
		numerator_coeffs_data[1] = N_gain;
		numerator_coeffs_data[0] = -N_gain;
		denominator_coeffs_data[1] = N_gain * sample_time / 2.0 + 1.0;
		denominator_coeffs_data[0] = N_gain * sample_time / 2.0 - 1.0;
	} else {
		throw exception_ERROR;
	}

	real_t input_initial_states[] = { initial_condition, initial_condition };
	states_set(input_initial_states, NULL);

}

DiscretePIDregulator::DiscretePIDregulator(real_t sample_time, real_t P_gain, real_t I_gain, real_t D_gain, real_t N_gain, approximation_method method) :
		integrator(method, sample_time), derivator(method, sample_time, N_gain) {
	this->P_gain = P_gain;
	this->I_gain = I_gain;
	this->D_gain = D_gain;

}

void DiscretePIDregulator::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

	const real_t e = *input_signals.at(0).ptr;
	real_t* u_ptr = output_signals.at(0).ptr;

	real_t u = 0;
	real_t u_P = 0;
	real_t u_I = 0;
	real_t u_D = 0;

	if (I_gain != (real_t)0) {
		integrator.input_port_real_value(0) = e + get_AW_correction();
		integrator.step();
		u_I = I_gain * integrator.output_port_real_value(0);
	}
	if (D_gain != (real_t)0) {
		derivator.input_port_real_value(0) = e;
		derivator.step();
		u_D = D_gain * derivator.output_port_real_value(0);
	}
	if (P_gain != (real_t)0) {
		u_P = P_gain * e;
	}
	u = u_P + u_I + u_D;
	u = apply_AW_saturation(u);
	*u_ptr = u;
}

DiscreteIPregulator::DiscreteIPregulator(real_t sample_time, real_t P_gain, real_t I_gain, approximation_method method) :
		DiscreteSystemMIMO(2, input_signals_data, 1, output_signals_data), integrator(method, sample_time) {
	this->P_gain = P_gain;
	this->I_gain = I_gain;
}

void DiscreteIPregulator::step_function(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 2 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t e = *input_signals.at(0).ptr;
	const real_t y = *input_signals.at(0).ptr;
	real_t* u_ptr = output_signals.at(0).ptr;
	real_t u, u_P, u_I;

	integrator.input_port_real_value(0) = e + get_AW_correction();
	integrator.step();
	u_I = I_gain * integrator.output_port_real_value(0);
	u_P = P_gain * (-y);
	u = u_P + u_I;
	u = apply_AW_saturation(u);
	*u_ptr = u;
}

