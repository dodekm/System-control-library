#include "discrete_systems.h"
#include "gsl_wrapper.h"
#include "signal_process.h"

#define state_buffer_value_at(buffer,n) vector_type_at((vector_generic_T*)&buffer,real_t,(n+buffer.shift_idx+1)%vector_length((vector_generic_T*)&buffer)) ///< access value in state circular buffer n samples past k-n
#define state_buffer_value_now(buffer)  vector_type_at((vector_generic_T*)&buffer,real_t,buffer.shift_idx) ///< write value to state circular buffer at current sample k
#define state_buffer_shift(buffer) {buffer.shift_idx++; buffer.shift_idx%=vector_length((vector_generic_T*)&buffer);} ///< shifts state circular buffer (sample the buffer) k++
#define state_buffer_reset(buffer) {buffer.shift_idx=0;} ///< resets state circular buffer shift index

static return_code discrete_SISO_system_generic_deinit(discrete_SISO_system_generic_T*);
static return_code discrete_SISO_system_generic_init(discrete_SISO_system_generic_T*, discrete_system_step_fcn_T, system_deinit_fcn);
static return_code discrete_MIMO_system_generic_deinit(discrete_MIMO_system_generic_T*);
static return_code discrete_MIMO_system_generic_init(discrete_MIMO_system_generic_T*, discrete_system_step_fcn_T, size_t, real_t*, size_t, real_t*, system_deinit_fcn);

/**
 * Generic discrete system step function with scalar arguments.
 * Updates system states and output.
 * Suitable for direct SISO system evaluation.
 * @param system discrete system structure pointer
 * @param u input scalar value
 * @param y_ptr output scalar value pointer
 * @return system control library error code
 */
return_code discrete_system_generic_step_direct_scalar(discrete_system_generic_T* system, real_t u, real_t* y_ptr) {
	vector_generic_T u_vector = { 0 }, y_vector = { 0 };
	signal_realtime_T signal_u, signal_y;
	signal_u.owner = NULL;
	signal_u.ptr = &u;
	signal_y.owner = NULL;
	signal_y.ptr = y_ptr;
	vector_type_init(&u_vector, signal_realtime_T, 1, &signal_u);
	vector_type_init(&y_vector, signal_realtime_T, 1, &signal_y);
	return system->step_fcn(system, &u_vector, &y_vector);
}

/**
 * Generic discrete SISO system interface deinitialization.
 * Function for internal (static) use.
 * Resets input/output ports.
 * Resets step function pointers.
 *
 * @param system discrete system structure pointer
 * @return system control library error code
 */
static return_code discrete_SISO_system_generic_deinit(discrete_SISO_system_generic_T* system) {

	system->step_fcn = NULL;
	return return_OK;
}

/**
 * Discrete SISO system initialization.
 * - initializes system interface (input and output ports)
 * - registers: step function and deinit function
 * Function for internal (static) use
 * @see system_interface_atomic_init
 * @param system discrete SISO system structure pointer
 * @param step_fcn system step function pointer
 * @param deinit_fcn system deinitialization function pointer
 * @return system control library error code
 */
static return_code discrete_SISO_system_generic_init(discrete_SISO_system_generic_T* system, discrete_system_step_fcn_T step_fcn, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;

	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) discrete_SISO_system_generic_deinit;
	returnval = system_interface_init(&system->interface, system_type_dynamic_system, system_time_domain_discrete, 1, &system->input, 1, &system->output, deinit_fcn);
	if (returnval != return_OK)
		return returnval;

	system->step_fcn = step_fcn;
	system->input = 0;
	system->output = 0;
	return return_OK;
}

/**
 * Generic discrete MIMO system interface deinitialization.
 * Function for internal (static) use.
 * Resets step function pointer.
 *
 * @param system discrete MIMO system structure pointer
 * @return system control library error code
 */

static return_code discrete_MIMO_system_generic_deinit(discrete_MIMO_system_generic_T* system) {

	system->step_fcn = NULL;
	return return_OK;
}

/**
 * Discrete MIMO system initialization.
 * - initializes system interface (input and output ports)
 * - registers: step function and deinit function
 * Function for internal (static) use.
 * @see system_interface_atomic_init
 * @param system discrete MIMO system structure pointer
 * @param step_fcn system step function pointer
 * @param input_port_width number of input signals
 * @param input_port_ptr  input signals pointer
 * @param output_port_width number of output signals
 * @param output_port_ptr  output signals pointer
 * @param deinit_fcn system deinitialization function pointer
 * @return system control library error code
 */
static return_code discrete_MIMO_system_generic_init(discrete_MIMO_system_generic_T* system, discrete_system_step_fcn_T step_fcn, size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;

	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) discrete_MIMO_system_generic_deinit;

	returnval = system_interface_init(&system->interface, system_type_dynamic_system, system_time_domain_discrete, input_port_width, input_port_ptr, output_port_width, output_port_ptr, deinit_fcn);
	if (returnval != return_OK)
		return returnval;

	system->step_fcn = step_fcn;
	memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);
	return return_OK;
}

static return_code discrete_transfer_function_step(discrete_transfer_function_T* transfer_function, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	real_t u = *u_ptr;
	real_t y = 0;
	state_buffer_value_now(transfer_function->input_states) = u;

	real_t* coeffs_ptr = vector_type_data_ptr(&transfer_function->numerator_coeffs, real_t);
	size_t num_length = vector_length(&transfer_function->numerator_coeffs);
	for (uint i = 0; i < num_length; i++) {
		y += coeffs_ptr[i] * state_buffer_value_at(transfer_function->input_states, i);
	}
	state_buffer_shift(transfer_function->input_states);
	coeffs_ptr = vector_type_data_ptr(&transfer_function->denominator_coeffs, real_t);

	size_t order = vector_length(&transfer_function->denominator_coeffs) - 1;
	for (uint i = 0; i < order; i++) {
		y += -coeffs_ptr[i] * state_buffer_value_at(transfer_function->output_states, i);
	}
	y /= coeffs_ptr[order];

#ifdef  USE_CLIP
	y = CLIP_TOP(y, transfer_function->saturation_high);
	y = CLIP_BOTTOM(y, transfer_function->saturation_low);
#endif

	state_buffer_value_now(transfer_function->output_states) = y;
	state_buffer_shift(transfer_function->output_states);

	*y_ptr = y;

	return return_OK;
}

static return_code discrete_transfer_function_deinit(discrete_transfer_function_T*);

/**
 * Discrete transfer function initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific step function and deinit function
 * - initializes states vectors -sets all states to zero
 * - initializes coefficients vectors
 * - sets saturation to -inf/inf
 *
 * @see discrete_SISO_system_generic_init
 * @see vector_generic_init
 *
 * @param transfer_function transfer function to initialize
 * @param numerator_order order of numerator
 * @param denominator_order order of denominator
 * @param numerator_coeffs_ptr pointer to numerator coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param denominator_coeffs_ptr pointer to denominator coefficients data (ascending power of z) , if NULL -function performs dynamic allocation
 * @param input_states_ptr pointer to input signal history states data , if NULL -function performs dynamic allocation
 * @param output_states_ptr pointer to output signal history states data , if NULL -function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_transfer_function_init(discrete_transfer_function_T* transfer_function, size_t numerator_order, size_t denominator_order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* input_states_ptr, real_t* output_states_ptr) {

	return_code returnval = return_OK;

	returnval = transfer_function_init((transfer_function_T*) &transfer_function->numerator_coeffs, numerator_order, denominator_order, numerator_coeffs_ptr, denominator_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_init((vector_generic_T* ) &transfer_function->input_states, real_t, numerator_order + 1, input_states_ptr);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_init((vector_generic_T* ) &transfer_function->output_states, real_t, denominator_order + 1, output_states_ptr);
	if (returnval != return_OK)
		return returnval;

	state_buffer_reset(transfer_function->input_states);
	state_buffer_reset(transfer_function->output_states);
	discrete_transfer_function_states_set_all(transfer_function, 0, 0);
#ifdef USE_CLIP
	transfer_function->saturation_high = INFINITY;
	transfer_function->saturation_low = -INFINITY;
#endif
	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) transfer_function, (discrete_system_step_fcn_T) discrete_transfer_function_step, (system_deinit_fcn) discrete_transfer_function_deinit);

}
static return_code discrete_transfer_function_deinit(discrete_transfer_function_T* transfer_function) {

	return_code returnval = return_OK;

	returnval = transfer_function_deinit((transfer_function_T*) &transfer_function->numerator_coeffs);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit((vector_generic_T*) &transfer_function->input_states);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit((vector_generic_T*) &transfer_function->output_states);
	if (returnval != return_OK)
		return returnval;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) transfer_function);

}

/**
 * Sets discrete transfer function input and output signals history states values.
 * Loads data to input/output states vectors.
 * @see vector_generic_load
 * @param transfer_function discrete transfer function structure pointer
 * @param input_states input signal states data pointer, if NULL - does not load
 * @param output_states input signal states data pointer, if NULL - does not load
 * @return system control library error code
 */
return_code discrete_transfer_function_states_set(discrete_transfer_function_T* transfer_function, const real_t* input_states, const real_t* output_states) {

	return_code returnval = return_OK;
	if (input_states != NULL) {
		returnval = vector_type_load((vector_generic_T* )&transfer_function->input_states, input_states);
		if (returnval != return_OK)
			return returnval;
		state_buffer_reset(transfer_function->input_states);
	}
	if (output_states != NULL) {
		returnval = vector_type_load((vector_generic_T* )&transfer_function->output_states, output_states);
		if (returnval != return_OK)
			return returnval;
		state_buffer_reset(transfer_function->output_states);
	}
	return return_OK;
}
/**
 * Sets discrete transfer function input/output signals history states to scalar value.
 * @see vector_type_set_all
 * @param transfer_function discrete transfer function structure pointer
 * @param input_state input signal states scalar value
 * @param output_state output signal states scalar value
 * @return system control library error code
 */
return_code discrete_transfer_function_states_set_all(discrete_transfer_function_T* transfer_function, real_t input_state, real_t output_state) {
	return_code returnval = return_OK;

	returnval = vector_type_set_all((vector_generic_T* ) &transfer_function->input_states, real_t, input_state)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(transfer_function->input_states);
	returnval = vector_type_set_all((vector_generic_T* ) &transfer_function->output_states, real_t, output_state)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(transfer_function->output_states);
	return return_OK;
}

/**
 * Sets transfer function output saturation.
 * @param transfer_function transfer function structure
 * @param saturation_high saturation upper bound
 * @param saturation_low saturation lower bound
 * @return system control library error code
 */
#ifdef USE_CLIP
return_code discrete_transfer_function_set_saturation(discrete_transfer_function_T* transfer_function, real_t saturation_high, real_t saturation_low) {
	if (saturation_high < saturation_low)
		return return_ERROR;
	transfer_function->saturation_high = saturation_high;
	transfer_function->saturation_low = saturation_low;
	return return_OK;

}
#endif

static return_code discrete_RST_regulator_deinit(discrete_RST_regulator_T*);
static return_code discrete_RST_regulator_step(discrete_RST_regulator_T*, const vector_generic_T*, vector_generic_T*);

/**
 * Discrete RST polynomial regulator initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific step function and deinit function
 * - initializes states vectors - sets all states to zero
 * - initializes coefficients vectors
 * - sets saturation to -inf/inf
 * @see discrete_MIMO_system_generic_init
 * @see vector_generic_init
 *
 * @param regulator RST regulator structure pointer
 * @param R_order order of autoregressive polynomial
 * @param S_order order of feedback polynomial
 * @param T_order order of feedforwad polynomial
 * @param R_coeffs_ptr  pointer to R coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param S_coeffs_ptr  pointer to S coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param T_coeffs_ptr  pointer to T coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param u_states_ptr  pointer to output signal history states data , if NULL -function performs dynamic allocation
 * @param y_states_ptr  pointer to feedback signal history states data , if NULL -function performs dynamic allocation
 * @param w_states_ptr  pointer to setpoint signal history states data , if NULL -function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_RST_regulator_init(discrete_RST_regulator_T* regulator, size_t R_order, size_t S_order, size_t T_order, real_t* R_coeffs_ptr, real_t* S_coeffs_ptr, real_t* T_coeffs_ptr, real_t* u_states_ptr, real_t* y_states_ptr, real_t* w_states_ptr) {

	return_code returnval = return_OK;

	returnval = vector_type_init(&regulator->R_coeffs, real_t, R_order + 1, R_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&regulator->S_coeffs, real_t, S_order + 1, S_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&regulator->T_coeffs, real_t, T_order + 1, T_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_init((vector_generic_T* ) &regulator->u_states, real_t, R_order + 1, u_states_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init((vector_generic_T* ) &regulator->y_states, real_t, S_order + 1, y_states_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init((vector_generic_T* ) &regulator->w_states, real_t, T_order + 1, w_states_ptr);
	if (returnval != return_OK)
		return returnval;

	state_buffer_reset(regulator->u_states);
	state_buffer_reset(regulator->y_states);
	state_buffer_reset(regulator->w_states);
	discrete_RST_regulator_states_set_all(regulator, 0, 0, 0);
#ifdef USE_CLIP
	regulator->saturation_high = INFINITY;
	regulator->saturation_low = -INFINITY;
#endif
	return discrete_MIMO_system_generic_init((discrete_MIMO_system_generic_T*) regulator, (discrete_system_step_fcn_T) discrete_RST_regulator_step, 2, regulator->input, 1, regulator->output, (system_deinit_fcn) discrete_RST_regulator_deinit);
}

static return_code discrete_RST_regulator_step(discrete_RST_regulator_T* regulator, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 2 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t w = *input_signals_ptr[0].ptr;
	real_t y = *input_signals_ptr[1].ptr;

	state_buffer_value_now(regulator->w_states) = w;

	real_t* T_coeffs_ptr = vector_type_data_ptr(&regulator->T_coeffs, real_t);
	size_t T_coeffs_length = vector_length(&regulator->T_coeffs);

	real_t u = 0;
	for (uint i = 0; i < T_coeffs_length; i++) {
		u += T_coeffs_ptr[i] * state_buffer_value_at(regulator->w_states, i);
	}
	state_buffer_shift(regulator->w_states);

	state_buffer_value_now(regulator->y_states) = y;
	real_t* S_coeffs_ptr = vector_type_data_ptr(&regulator->S_coeffs, real_t);
	size_t S_coeffs_length = vector_length(&regulator->S_coeffs);

	for (uint i = 0; i < S_coeffs_length; i++) {
		u += -S_coeffs_ptr[i] * state_buffer_value_at(regulator->y_states, i);
	}
	state_buffer_shift(regulator->y_states);

	real_t* R_coeffs_ptr = vector_type_data_ptr(&regulator->R_coeffs, real_t);
	size_t R_coeffs_length = vector_length(&regulator->R_coeffs);
	for (uint i = 0; i < R_coeffs_length - 1; i++) {
		u += -R_coeffs_ptr[i] * state_buffer_value_at(regulator->u_states, i);
	}
	u /= R_coeffs_ptr[R_coeffs_length - 1];

#ifdef  USE_CLIP
	u = CLIP_TOP(u, regulator->saturation_high);
	u = CLIP_BOTTOM(u, regulator->saturation_low);
#endif

	state_buffer_value_now(regulator->u_states) = u;
	state_buffer_shift(regulator->u_states);

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = u;

	return return_OK;
}

/**
 * Sets discrete RST regulator R,S,T polynomials coefficients.
 * Loads data to R,S,T coefficients vectors.
 * @see vector_generic_load
 * @param regulator RST regulator structure pointer
 * @param R_coeffs_ptr R polynomial coefficients source pointer (ascending power of z), if NULL - does not load
 * @param S_coeffs_ptr S polynomial coefficients source pointer (ascending power of z), if NULL - does not load
 * @param T_coeffs_ptr S polynomial coefficients source pointer (ascending power of z), if NULL - does not load
 * @return system control library error code
 */
return_code discrete_RST_regulator_coeffs_set(discrete_RST_regulator_T* regulator, const real_t* R_coeffs_ptr, const real_t* S_coeffs_ptr, const real_t* T_coeffs_ptr) {

	return_code returnval = return_OK;
	if (R_coeffs_ptr != NULL) {
		returnval = vector_type_load(&regulator->R_coeffs, R_coeffs_ptr);
		if (returnval != return_OK)
			return returnval;
	}
	if (S_coeffs_ptr != NULL) {
		returnval = vector_type_load(&regulator->S_coeffs, S_coeffs_ptr);
		if (returnval != return_OK)
			return returnval;
	}
	if (T_coeffs_ptr != NULL) {
		returnval = vector_type_load(&regulator->T_coeffs, T_coeffs_ptr);
		if (returnval != return_OK)
			return returnval;
	}
	return return_OK;
}

/**
 * Sets RST regulator output,feedback and setpoint signals history states values.
 * Loads data to u,y,w states vectors.
 * @see vector_generic_load
 * @param regulator RST regulator structure pointer
 * @param u_states_ptr output signal states data pointer, if NULL - does not load,
 * @param y_states_ptr feedback signal states data pointer, if NULL - does not load
 * @param w_states_ptr setpoint signal states data pointer, if NULL - does not load
 * @return system control library error code
 */
return_code discrete_RST_regulator_states_set(discrete_RST_regulator_T* regulator, const real_t* u_states_ptr, const real_t* y_states_ptr, const real_t* w_states_ptr) {
	return_code returnval = return_OK;
	if (u_states_ptr != NULL) {
		returnval = vector_type_load((vector_generic_T* ) &regulator->u_states, u_states_ptr);
		if (returnval != return_OK)
			return returnval;
		state_buffer_reset(regulator->u_states);
	}
	if (y_states_ptr != NULL) {
		returnval = vector_type_load((vector_generic_T* ) &regulator->y_states, y_states_ptr);
		if (returnval != return_OK)
			return returnval;
		state_buffer_reset(regulator->y_states);
	}
	if (w_states_ptr != NULL) {
		returnval = vector_type_load((vector_generic_T* ) &regulator->w_states, w_states_ptr);
		if (returnval != return_OK)
			return returnval;
		state_buffer_reset(regulator->w_states);
	}
	return return_OK;
}
/**
 * Sets RST regulator output,feedback and setpoint signals history states to scalar value.
 * @see vector_type_set_all
 * @param regulator RST regulator structure pointer
 * @param u_states_value output signal states scalar value
 * @param y_states_value feedback signal states scalar value
 * @param w_states_value setpoint signal states scalar value
 * @return system control library error code
 */
return_code discrete_RST_regulator_states_set_all(discrete_RST_regulator_T* regulator, real_t u_states_value, real_t y_states_value, real_t w_states_value) {
	return_code returnval = return_OK;
	returnval = vector_type_set_all((vector_generic_T* ) &regulator->u_states, real_t, u_states_value)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(regulator->u_states);
	returnval = vector_type_set_all((vector_generic_T* ) &regulator->y_states, real_t, y_states_value)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(regulator->y_states);
	returnval = vector_type_set_all((vector_generic_T* ) &regulator->w_states, real_t, w_states_value)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(regulator->w_states);
	return return_OK;
}

static return_code discrete_RST_regulator_deinit(discrete_RST_regulator_T* regulator) {

	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&regulator->R_coeffs);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_generic_deinit(&regulator->S_coeffs);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&regulator->T_coeffs);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_generic_deinit((vector_generic_T*) &regulator->u_states);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit((vector_generic_T*) &regulator->y_states);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit((vector_generic_T*) &regulator->w_states);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(regulator->u_states);
	state_buffer_reset(regulator->y_states);
	state_buffer_reset(regulator->w_states);

	return discrete_MIMO_system_generic_deinit((discrete_MIMO_system_generic_T*) regulator);
}

/**
 * Creates control loop subsystem using specific RST regulator.
 * Connects input/output signals of regulator and controlled system appropriately, fills system execution list.
 * @param regulator RST regulator structure pointer
 * @param system controlled discrete system
 * @param control_loop_subsystem destination control loop subsystem
 * @return system control library error code
 */
return_code discrete_RST_regulator_create_control_loop(discrete_RST_regulator_T* regulator, discrete_SISO_system_generic_T* system, system_generic_T* control_loop_subsystem) {
	return_code returnval = return_OK;
	{
		system_generic_T feedback_system = { 0 };
		uint input_port_numbers[] = { 0, 1 };
		uint output_port_numbers[] = { 0, 0 };
		system_generic_T* systems_list[] = { (system_generic_T*) system, (system_generic_T*) regulator };
		returnval = systems_connect_serial(systems_list, 2, input_port_numbers, output_port_numbers, &feedback_system);
		if (returnval != return_OK)
			return returnval;
		system_interface_deinit(&feedback_system);
	}
	{
		uint input_port_numbers[] = { 0, 0 };
		uint output_port_numbers[] = { 0, 0 };
		system_generic_T* systems_list[] = { (system_generic_T*) regulator, (system_generic_T*) system };
		returnval = systems_connect_serial(systems_list, 2, input_port_numbers, output_port_numbers, control_loop_subsystem);
		if (returnval != return_OK)
			return returnval;
	}
	return return_OK;
}

/**
 * Sets RST regulator output saturation.
 * @param regulator RST regulator structure pointer
 * @param saturation_high saturation upper bound
 * @param saturation_low saturation lower bound
 * @return system control library error code
 */
#ifdef USE_CLIP
return_code discrete_RST_regulator_set_saturation(discrete_RST_regulator_T* regulator, real_t saturation_high, real_t saturation_low) {
	if (saturation_high < saturation_low)
		return return_ERROR;
	regulator->saturation_high = saturation_high;
	regulator->saturation_low = saturation_low;
	return return_OK;
}
#endif

static return_code discrete_delay_step(discrete_delay_T* delay, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;
	real_t u = *u_ptr;
	state_buffer_value_now(delay->states) = u;
	real_t y = state_buffer_value_at(delay->states, 0);
	state_buffer_shift(delay->states);
	*y_ptr = y;
	return return_OK;
}

static return_code discrete_delay_deinit(discrete_delay_T*);

/**
 * Discrete delay initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific step function and deinit function
 * - initializes states vector - sets all states to zero
 * @see discrete_SISO_system_generic_init
 * @see vector_generic_init
 * @param delay discrete delay structure pointer
 * @param order order of  delay  (n samples)
 * @param states_ptr pointer to input signal history state buffer, if NULL -function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_delay_init(discrete_delay_T* delay, uint order, real_t* states_ptr) {
	return_code returnval = return_OK;

	returnval = vector_type_init((vector_generic_T* ) &delay->states, real_t, order + 1, states_ptr);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(delay->states);
	returnval = vector_type_set_all((vector_generic_T* ) &delay->states, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;

	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) delay, (discrete_system_step_fcn_T) discrete_delay_step, (system_deinit_fcn) discrete_delay_deinit);

}
static return_code discrete_delay_deinit(discrete_delay_T* delay) {

	return_code returnval = return_OK;
	returnval = vector_generic_deinit((vector_generic_T*) &delay->states);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(delay->states);

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) delay);

}

/**
 * Discrete sumator initialization.
 * @param sumator discrete sumator structure pointer
 * @param initial_condition sumator initial condition value
 * @return system control library error code
 */
return_code discrete_sumator_init(discrete_sumator_T* sumator, real_t initial_condition) {
	return_code returnval = return_OK;
	returnval = discrete_transfer_function_init((discrete_transfer_function_T*) sumator, 0, 1, sumator->numerator_coeffs_data, sumator->denominator_coeffs_data, sumator->input_states_data, sumator->output_states_data);
	if (returnval != return_OK)
		return returnval;

	sumator->numerator_coeffs_data[0] = 1.0;
	sumator->denominator_coeffs_data[1] = 1.0;
	sumator->denominator_coeffs_data[0] = -1.0;

	real_t output_initial_states[] = { initial_condition, initial_condition };
	returnval = discrete_transfer_function_states_set((discrete_transfer_function_T*) sumator, NULL, output_initial_states);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}
/**
 * Discrete difference initialization.
 * @param difference discrete difference structure pointer
 * @param initial_condition difference initial condition value
 * @return system control library error code
 */
return_code discrete_difference_init(discrete_difference_T* difference, real_t initial_condition) {
	return_code returnval = return_OK;
	returnval = discrete_transfer_function_init((discrete_transfer_function_T*) difference, 1, 0, difference->numerator_coeffs_data, difference->denominator_coeffs_data, difference->input_states_data, difference->output_states_data);
	if (returnval != return_OK)
		return returnval;

	difference->numerator_coeffs_data[1] = 1.0;
	difference->numerator_coeffs_data[0] = -1.0;
	difference->denominator_coeffs_data[0] = 1.0;

	real_t input_initial_states[] = { initial_condition, initial_condition };
	returnval = discrete_transfer_function_states_set((discrete_transfer_function_T*) difference, input_initial_states, NULL);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}
/**
 * Discrete integrator initialization.
 * Initializes discrete time integrator using numerical approximations
 * @param integrator discrete integrator structure pointer
 * @param sample_time sample time
 * @param initial_condition integrator initial condition
 * @param method integration approximation method
 * @return system control library error code
 */
return_code discrete_integrator_init(discrete_integrator_T* integrator, real_t sample_time, real_t initial_condition, approximation_method method) {

	return_code returnval = return_OK;
	returnval = discrete_transfer_function_init((discrete_transfer_function_T*) integrator, 1, 1, integrator->numerator_coeffs_data, integrator->denominator_coeffs_data, integrator->input_states_data, integrator->output_states_data);
	if (returnval != return_OK)
		return returnval;

	integrator->denominator_coeffs_data[1] = 1.0;
	integrator->denominator_coeffs_data[0] = -1.0;

	if (method == approximation_method_backward_euler) {
		integrator->numerator_coeffs_data[1] = sample_time;
		integrator->numerator_coeffs_data[0] = 0;
	} else if (method == approximation_method_forward_euler) {
		integrator->numerator_coeffs_data[1] = 0;
		integrator->numerator_coeffs_data[0] = sample_time;
	} else if (method == approximation_method_trapezoidal_rule) {
		integrator->numerator_coeffs_data[1] = sample_time / 2;
		integrator->numerator_coeffs_data[0] = sample_time / 2;
	} else {
		return return_ERROR;
	}
	real_t output_initial_states[] = { initial_condition, initial_condition };
	returnval = discrete_transfer_function_states_set((discrete_transfer_function_T*) integrator, NULL, output_initial_states);
	if (returnval != return_OK)
		return returnval;

	return return_OK;
}
/**
 * Discrete derivative initialization.
 * Initializes discrete time derivative using numerical approximations
 * @param derivative discrete derivative structure pointer
 * @param N_gain derivative filtering coefficient
 * @param sample_time sample time
 * @param initial_condition derivator initial condition
 * @param method derivative approximation method
 * @return system control library error code
 */
return_code discrete_derivative_init(discrete_derivative_T* derivative, real_t N_gain, real_t sample_time, real_t initial_condition, approximation_method method) {

	return_code returnval = return_OK;

	returnval = discrete_transfer_function_init((discrete_transfer_function_T*) derivative, 1, 1, derivative->numerator_coeffs_data, derivative->denominator_coeffs_data, derivative->input_states_data, derivative->output_states_data);
	if (method == approximation_method_backward_euler) {
		derivative->numerator_coeffs_data[1] = N_gain;
		derivative->numerator_coeffs_data[0] = -N_gain;
		derivative->denominator_coeffs_data[1] = N_gain * sample_time + 1;
		derivative->denominator_coeffs_data[0] = -1;
	} else if (method == approximation_method_forward_euler) {
		derivative->numerator_coeffs_data[1] = N_gain;
		derivative->numerator_coeffs_data[0] = -N_gain;
		derivative->denominator_coeffs_data[1] = 1;
		derivative->denominator_coeffs_data[0] = N_gain * sample_time - 1;
	} else if (method == approximation_method_trapezoidal_rule) {
		derivative->numerator_coeffs_data[1] = N_gain;
		derivative->numerator_coeffs_data[0] = -N_gain;
		derivative->denominator_coeffs_data[1] = N_gain * sample_time / 2 + 1;
		derivative->denominator_coeffs_data[0] = N_gain * sample_time / 2 - 1;
	} else {
		return return_ERROR;
	}
	if (returnval != return_OK)
		return returnval;
	real_t input_initial_states[] = { initial_condition, initial_condition };
	returnval = discrete_transfer_function_states_set((discrete_transfer_function_T*) derivative, input_initial_states, NULL);
	if (returnval != return_OK)
		return returnval;

	return return_OK;

}

static return_code discrete_PSD_regulator_step(discrete_PSD_regulator_T* PSD_regulator, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* e_ptr = input_signals_ptr[0].ptr;
	real_t* u_ptr = output_signals_ptr[0].ptr;
	real_t e = *e_ptr;
	real_t u = 0;
	real_t u_P = 0;
	real_t u_S = 0;
	real_t u_D = 0;
	return_code returnval = return_OK;

	if (PSD_regulator->S_gain != 0) {
		system_input_port_real_value((system_generic_T*)&PSD_regulator->sumator,0) = e;
#ifdef  USE_CLIP
		system_input_port_real_value((system_generic_T*)&PSD_regulator->sumator,0) += PSD_regulator->u_cut * PSD_regulator->AW_gain;
#endif
		returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) &PSD_regulator->sumator);
		if (returnval != return_OK)
			return returnval;
		u_S = PSD_regulator->S_gain * system_output_port_real_value((system_generic_T* )&PSD_regulator->sumator, 0);
	}

	if (PSD_regulator->D_gain != 0) {
		system_input_port_real_value((system_generic_T*)&PSD_regulator->difference,0) = e;
		returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) &PSD_regulator->difference);
		if (returnval != return_OK)
			return returnval;
		u_D = PSD_regulator->D_gain * system_output_port_real_value((system_generic_T* )&PSD_regulator->difference, 0);
	}
	if (PSD_regulator->P_gain != 0) {
		u_P = PSD_regulator->P_gain * e;
	}
	u = u_P + u_S + u_D;

#ifdef  USE_CLIP
	real_t u_saturated = u;
	u_saturated = CLIP_TOP(u_saturated, PSD_regulator->saturation_high);
	u_saturated = CLIP_BOTTOM(u_saturated, PSD_regulator->saturation_low);
	PSD_regulator->u_cut = u_saturated - u;
	u = u_saturated;
#endif
	*u_ptr = u;
	return return_OK;
}
static return_code discrete_PSD_regulator_deinit(discrete_PSD_regulator_T*);

/**
 * Discrete PSD regulator initialization.
 * Initializes sumator and difference structures.
 * Sets saturation to -inf/inf.
 * @see discrete_SISO_system_generic_init
 * @param PSD_regulator PSD regulator structure pointer
 * @param P_gain proportional gain
 * @param S_gain sumator gain
 * @param D_gain difference gain
 * @return system control library error code
 */
return_code discrete_PSD_regulator_init(discrete_PSD_regulator_T* PSD_regulator, real_t P_gain, real_t S_gain, real_t D_gain) {

	return_code returnval = return_OK;

	returnval = discrete_sumator_init(&PSD_regulator->sumator, 0);
	if (returnval != return_OK)
		return returnval;
	returnval = discrete_difference_init(&PSD_regulator->difference, 0);
	if (returnval != return_OK)
		return returnval;

	PSD_regulator->P_gain = P_gain;
	PSD_regulator->S_gain = S_gain;
	PSD_regulator->D_gain = D_gain;

#ifdef USE_CLIP
	PSD_regulator->saturation_high = INFINITY;
	PSD_regulator->saturation_low = -INFINITY;
	PSD_regulator->AW_gain = 0;
	PSD_regulator->u_cut = 0;
#endif

	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) PSD_regulator, (discrete_system_step_fcn_T) discrete_PSD_regulator_step, (system_deinit_fcn) discrete_PSD_regulator_deinit);
}
#ifdef USE_CLIP
return_code discrete_PSD_regulator_set_saturation(discrete_PSD_regulator_T* PSD_regulator, real_t saturation_high, real_t saturation_low, real_t AW_gain) {
	if (saturation_high < saturation_low)
		return return_ERROR;
	PSD_regulator->saturation_high = saturation_high;
	PSD_regulator->saturation_low = saturation_low;
	PSD_regulator->AW_gain = AW_gain;
	return return_OK;
}
#endif

static return_code discrete_PSD_regulator_deinit(discrete_PSD_regulator_T* PSD_regulator) {

	return_code returnval = return_OK;
	returnval = system_deinit((system_generic_T*) &PSD_regulator->sumator);
	if (returnval != return_OK)
		return returnval;
	returnval = system_deinit((system_generic_T*) &PSD_regulator->difference);
	if (returnval != return_OK)
		return returnval;

	PSD_regulator->P_gain = 0;
	PSD_regulator->S_gain = 0;
	PSD_regulator->D_gain = 0;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) PSD_regulator);
}

static return_code discrete_PID_regulator_step(discrete_PID_regulator_T* PID_regulator, const vector_generic_T* input_signals, vector_generic_T* output_signals) {
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* e_ptr = input_signals_ptr[0].ptr;
	real_t* u_ptr = output_signals_ptr[0].ptr;
	real_t e = *e_ptr;
	real_t u = 0;
	real_t u_P = 0;
	real_t u_I = 0;
	real_t u_D = 0;
	return_code returnval = return_OK;

	if (PID_regulator->I_gain != 0) {
		system_input_port_real_value((system_generic_T*)&PID_regulator->integrator,0) = e;
#ifdef  USE_CLIP
		system_input_port_real_value((system_generic_T*)&PID_regulator->integrator,0) += PID_regulator->u_cut * PID_regulator->AW_gain;
#endif
		returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) &PID_regulator->integrator);
		if (returnval != return_OK)
			return returnval;
		u_I = PID_regulator->I_gain * system_output_port_real_value((system_generic_T* )&PID_regulator->integrator, 0);
	}
	if (PID_regulator->D_gain != 0) {
		system_input_port_real_value((system_generic_T*)&PID_regulator->derivator,0) = e;
		returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) &PID_regulator->derivator);
		if (returnval != return_OK)
			return returnval;
		u_D = PID_regulator->D_gain * system_output_port_real_value((system_generic_T* )&PID_regulator->derivator, 0);
	}
	if (PID_regulator->P_gain != 0) {
		u_P = PID_regulator->P_gain * e;
	}

	u = u_P + u_I + u_D;
#ifdef  USE_CLIP
	real_t u_saturated = u;
	u_saturated = CLIP_TOP(u_saturated, PID_regulator->saturation_high);
	u_saturated = CLIP_BOTTOM(u_saturated, PID_regulator->saturation_low);
	PID_regulator->u_cut = u_saturated - u;
	u = u_saturated;
#endif
	*u_ptr = u;
	return return_OK;
}

static return_code discrete_PID_regulator_deinit(discrete_PID_regulator_T*);

/**
 * Discrete PID regulator initialization.
 * Initializes integrator and derivator structures using numerical approximations.
 * Sets saturation to -inf/inf.
 * @see discrete_SISO_system_generic_init
 * @param PID_regulator discrete PID regulator structure pointer
 * @param sample_time sample time
 * @param P_gain proportional gain
 * @param I_gain integrator gain
 * @param D_gain derivative gain
 * @param N_gain derivative filtering coefficient
 * @param method approximation method for derivator and integrator
 * @return system control library error code
 */
return_code discrete_PID_regulator_init(discrete_PID_regulator_T* PID_regulator, real_t sample_time, real_t P_gain, real_t I_gain, real_t D_gain, real_t N_gain, approximation_method method) {

	return_code returnval = return_OK;

	returnval = discrete_integrator_init(&PID_regulator->integrator, sample_time, 0, method);
	if (returnval != return_OK)
		return returnval;
	returnval = discrete_derivative_init(&PID_regulator->derivator, N_gain, sample_time, 0, method);
	if (returnval != return_OK)
		return returnval;

	PID_regulator->P_gain = P_gain;
	PID_regulator->I_gain = I_gain;
	PID_regulator->D_gain = D_gain;
#ifdef USE_CLIP
	PID_regulator->saturation_high = INFINITY;
	PID_regulator->saturation_low = -INFINITY;
	PID_regulator->AW_gain = 0;
	PID_regulator->u_cut = 0;
#endif

	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) PID_regulator, (discrete_system_step_fcn_T) discrete_PID_regulator_step, (system_deinit_fcn) discrete_PID_regulator_deinit);

}
#ifdef USE_CLIP
return_code discrete_PID_regulator_set_saturation(discrete_PID_regulator_T* PID_regulator, real_t saturation_high, real_t saturation_low, real_t AW_gain) {
	if (saturation_high < saturation_low)
		return return_ERROR;
	PID_regulator->saturation_high = saturation_high;
	PID_regulator->saturation_low = saturation_low;
	PID_regulator->AW_gain = AW_gain;
	return return_OK;
}
#endif

static return_code discrete_PID_regulator_deinit(discrete_PID_regulator_T* PID_regulator) {

	return_code returnval = return_OK;
	returnval = system_deinit((system_generic_T*) &PID_regulator->integrator);
	if (returnval != return_OK)
		return returnval;
	returnval = system_deinit((system_generic_T*) &PID_regulator->derivator);
	if (returnval != return_OK)
		return returnval;

	PID_regulator->P_gain = 0;
	PID_regulator->I_gain = 0;
	PID_regulator->D_gain = 0;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) PID_regulator);

}

static return_code discrete_IP_regulator_step(discrete_IP_regulator_T* IP_regulator, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 2 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* e_ptr = input_signals_ptr[0].ptr;
	const real_t* y_ptr = input_signals_ptr[1].ptr;
	real_t* u_ptr = output_signals_ptr[0].ptr;
	real_t e = *e_ptr;
	real_t y = *y_ptr;
	real_t u, u_P, u_I;
	return_code returnval = return_OK;

	system_input_port_real_value((system_generic_T*)&IP_regulator->integrator,0) = e;
#ifdef  USE_CLIP
	system_input_port_real_value((system_generic_T*)&IP_regulator->integrator,0) += IP_regulator->u_cut * IP_regulator->AW_gain;

#endif
	returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) &IP_regulator->integrator);
	if (returnval != return_OK)
		return returnval;
	u_I = IP_regulator->I_gain * system_output_port_real_value((system_generic_T* )&IP_regulator->integrator, 0);
	u_P = IP_regulator->P_gain * (-y);
	u = u_P + u_I;
#ifdef  USE_CLIP
	real_t u_saturated = u;
	u_saturated = CLIP_TOP(u_saturated, IP_regulator->saturation_high);
	u_saturated = CLIP_BOTTOM(u_saturated, IP_regulator->saturation_low);
	IP_regulator->u_cut = u_saturated - u;
	u = u_saturated;
#endif
	*u_ptr = u;
	return return_OK;
}

static return_code discrete_IP_regulator_deinit(discrete_IP_regulator_T*);

/**
 * Discrete IP regulator initialization.
 * Initializes discrete integrator structure using numerical approximations.
 * Sets saturation to -inf/inf.
 * @see discrete_MIMO_system_generic_init
 * @param IP_regulator IP regulator structure pointer
 * @param sample_time sample time
 * @param P_gain proportional gain
 * @param I_gain integrator gain
 * @param method approximation method
 * @return system control library error code
 */
return_code discrete_IP_regulator_init(discrete_IP_regulator_T* IP_regulator, real_t sample_time, real_t P_gain, real_t I_gain, approximation_method method) {
	return_code returnval = return_OK;
	returnval = discrete_integrator_init(&IP_regulator->integrator, sample_time, 0, method);
	if (returnval != return_OK)
		return returnval;
	IP_regulator->P_gain = P_gain;
	IP_regulator->I_gain = I_gain;

#ifdef USE_CLIP
	IP_regulator->saturation_high = INFINITY;
	IP_regulator->saturation_low = -INFINITY;
	IP_regulator->AW_gain = 0;
	IP_regulator->u_cut = 0;
#endif

	return discrete_MIMO_system_generic_init((discrete_MIMO_system_generic_T*) IP_regulator, (discrete_system_step_fcn_T) discrete_IP_regulator_step, 2, IP_regulator->input, 1, IP_regulator->output, (system_deinit_fcn) discrete_IP_regulator_deinit);
}
#ifdef USE_CLIP
return_code discrete_IP_regulator_set_saturation(discrete_IP_regulator_T* IP_regulator, real_t saturation_high, real_t saturation_low, real_t AW_gain) {
	if (saturation_high < saturation_low)
		return return_ERROR;
	IP_regulator->saturation_high = saturation_high;
	IP_regulator->saturation_low = saturation_low;
	IP_regulator->AW_gain = AW_gain;
	return return_OK;
}
#endif

static return_code discrete_IP_regulator_deinit(discrete_IP_regulator_T* IP_regulator) {

	return_code returnval = return_OK;
	returnval = system_deinit((system_generic_T*) &IP_regulator->integrator);
	if (returnval != return_OK)
		return returnval;

	IP_regulator->P_gain = 0;
	IP_regulator->I_gain = 0;

	return discrete_MIMO_system_generic_deinit((discrete_MIMO_system_generic_T*) IP_regulator);
}

static return_code discrete_IIR_filter_DF_II_step(discrete_IIR_filter_DF_II_T* filter, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	real_t v = *u_ptr;
	real_t* coeffs_ptr = vector_type_data_ptr(&filter->denominator_coeffs, real_t);

	size_t order = vector_length(&filter->denominator_coeffs) - 1;
	for (uint i = 0; i < order; i++) {
		v += -coeffs_ptr[i] * state_buffer_value_at(filter->states, i);
	}
	v /= coeffs_ptr[order];
	state_buffer_value_now(filter->states) = v;

	coeffs_ptr = vector_type_data_ptr(&filter->numerator_coeffs, real_t);
	real_t y = 0;
	for (uint i = 0; i < order + 1; i++) {
		y += coeffs_ptr[i] * state_buffer_value_at(filter->states, i);
	}
	state_buffer_shift(filter->states);
	*y_ptr = y;

	return return_OK;
}

static return_code discrete_IIR_filter_DF_II_deinit(discrete_IIR_filter_DF_II_T*);

/**
 * Discrete infinite impulse response filter (direct form II) initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific step function and deinit function
 * - initializes states vectors -sets all states to zero
 * - initializes coefficients vectors
 *
 * @see discrete_SISO_system_generic_init
 * @see vector_generic_init
 * @param filter filter structure pointer
 * @param order order of filter
 * @param numerator_coeffs_ptr pointer to numerator coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param denominator_coeffs_ptr pointer to denominator coefficients data (ascending power of z) , if NULL -function performs dynamic allocation
 * @param states_ptr pointer to internal signal history states data , if NULL -function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_IIR_filter_DF_II_init(discrete_IIR_filter_DF_II_T* filter, size_t order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* states_ptr) {

	return_code returnval = return_OK;
	returnval = transfer_function_init((transfer_function_T*) &filter->numerator_coeffs, order, order, numerator_coeffs_ptr, denominator_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init((vector_generic_T* ) &filter->states, real_t, order + 1, states_ptr);
	if (returnval != return_OK)
		return returnval;

	state_buffer_reset(filter->states);
	vector_type_set_all((vector_generic_T* ) &filter->states, real_t, 0);

	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) filter, (discrete_system_step_fcn_T) discrete_IIR_filter_DF_II_step, (system_deinit_fcn) discrete_IIR_filter_DF_II_deinit);
}

/**
 * Sets discrete IIR filter numerator and denominator coefficients.
 * Loads data to filters numerator/denominator coefficients vectors
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param numerator_coeffs_ptr  numerator coefficients source pointer (ascending power of z), if NULL - does not load
 * @param denominator_coeffs_ptr  denominator coefficients source pointer (ascending power of z), if NULL - does not load
 * @return system control library error code
 */
return_code discrete_IIR_filter_DF_II_coeffs_set(discrete_IIR_filter_DF_II_T* filter, const real_t* numerator_coeffs_ptr, const real_t* denominator_coeffs_ptr) {
	return transfer_function_coeffs_set((transfer_function_T*) &filter->numerator_coeffs, numerator_coeffs_ptr, denominator_coeffs_ptr);
}

/**
 * Sets discrete IIR filter internal signal history states buffer values.
 * Loads data to filter states vectors
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param states_ptr internal signal states data pointer
 * @return system control library error code
 */
return_code discrete_IIR_filter_DF_II_states_set(discrete_IIR_filter_DF_II_T* filter, const real_t* states_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_load((vector_generic_T* )&filter->states, states_ptr);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);
	return return_OK;

}
/**
 * Sets discrete IIR filter internal signal history states buffer to scalar value.
 * @see vector_type_set_all
 * @param filter filter structure pointer
 * @param states_value internal signal states scalar value
 * @return system control library error code
 */
return_code discrete_IIR_filter_DF_II_states_set_all(discrete_IIR_filter_DF_II_T* filter, real_t states_value) {
	return_code returnval = return_OK;
	returnval = vector_type_set_all((vector_generic_T* ) &filter->states, real_t, states_value)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);
	return return_OK;
}

static return_code discrete_IIR_filter_DF_II_deinit(discrete_IIR_filter_DF_II_T* filter) {

	return_code returnval = return_OK;

	returnval = transfer_function_deinit((transfer_function_T*) &filter->numerator_coeffs);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_generic_deinit((vector_generic_T*) &filter->states);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) filter);

}

static return_code discrete_FIR_filter_deinit(discrete_FIR_filter_T*);
static return_code discrete_FIR_filter_step(discrete_FIR_filter_T*, const vector_generic_T*, vector_generic_T*);

/**
 * Discrete finite impulse response filter initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific  step function and deinit function
 * - initializes states vectors -sets all states to zero
 * - initializes coefficients vectors
 *
 * @see discrete_SISO_system_generic_init
 * @see vector_generic_init
 *
 * @param filter filter structure pointer
 * @param order order of filter
 * @param coeffs_ptr pointer to filter coefficients data (ascending power of z) , if NULL - function performs dynamic allocation
 * @param states_ptr  pointer to input signal history states data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_FIR_filter_init(discrete_FIR_filter_T* filter, size_t order, real_t* coeffs_ptr, real_t* states_ptr) {
	return_code returnval = return_OK;

	returnval = vector_type_init((vector_generic_T* ) &filter->states, real_t, order + 1, states_ptr);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);
	returnval = vector_type_set_all((vector_generic_T* ) &filter->states, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&filter->coeffs, real_t, order + 1, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) filter, (discrete_system_step_fcn_T) discrete_FIR_filter_step, (system_deinit_fcn) discrete_FIR_filter_deinit);

}

static return_code discrete_FIR_filter_step(discrete_FIR_filter_T* filter, const vector_generic_T* input_signals, vector_generic_T* output_signals) {
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	real_t u = *u_ptr;
	state_buffer_value_now(filter->states) = u;
	real_t y = 0;
	real_t* coeffs_ptr = vector_type_data_ptr(&filter->coeffs, real_t);
	for (uint i = 0; i < filter->coeffs.length; i++) {
		y += coeffs_ptr[i] * state_buffer_value_at(filter->states, i);
	}
	state_buffer_shift(filter->states);
	*y_ptr = y;
	return return_OK;
}

/**
 * Sets discrete FIR filter coefficients.
 * Loads data to filter coefficients vector
 * @see vector_generic_load
 *
 * @param filter filter structure pointer
 * @param coeffs_ptr filter coefficients data pointer (ascending power of z)
 * @return system control library error code
 */
return_code discrete_FIR_filter_coeffs_set(discrete_FIR_filter_T* filter, const real_t* coeffs_ptr) {
	return_code returnval = return_OK;
	returnval = vector_type_load((vector_generic_T* )&filter->coeffs, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Sets discrete FIR filter input signal history states buffer values.
 * Loads data to states vector
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param states_ptr input signal states data pointer
 * @return system control library error code
 */
return_code discrete_FIR_filter_states_set(discrete_FIR_filter_T* filter, const real_t* states_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_load((vector_generic_T* )&filter->states, states_ptr);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);
	return return_OK;
}

/**
 * Sets discrete FIR filter input signal history states buffer to scalar value.
 * @see vector_type_set_all
 * @param filter filter structure pointer
 * @param states_value input signal states scalar value
 * @return system control library error code
 */
return_code discrete_FIR_filter_states_set_all(discrete_FIR_filter_T* filter, real_t states_value) {
	return_code returnval = return_OK;
	returnval = vector_type_set_all((vector_generic_T* ) &filter->states, real_t, states_value)
	;
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);
	return return_OK;
}

static return_code discrete_FIR_filter_deinit(discrete_FIR_filter_T* filter) {
	return_code returnval = return_OK;
	returnval = vector_generic_deinit((vector_generic_T*) &filter->states);
	if (returnval != return_OK)
		return returnval;
	state_buffer_reset(filter->states);

	returnval = vector_generic_deinit(&filter->coeffs);
	if (returnval != return_OK)
		return returnval;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) filter);

}

/**
 * Initializes discrete FIR filter from generic discrete system.
 * Uses discrete impulse function of generic system to obtain FIR filter coefficients
 * @param filter filter structure pointer
 * @param discrete_system generic discrete system pointer
 * @return system control library error code
 */
return_code discrete_FIR_filter_coeffs_set_from_impulse_response(discrete_FIR_filter_T* filter, discrete_system_generic_T* discrete_system) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (discrete_system == NULL)
		return return_NULLPTR;
#endif
	size_t order = vector_length(&filter->coeffs) - 1;
	real_t u = 0;
	real_t y = 0;
	for (uint i = 0; i < order + 1; i++) {
		u = (real_t) (i == 0);
		returnval = discrete_system_generic_step_direct_scalar(discrete_system, u, &y);
		if (returnval != return_OK)
			return returnval;
		vector_type_at(&filter->coeffs,real_t,order-i) = y;
	}
	return return_OK;
}

static real_t discrete_biquad_section_DF_I_step(discrete_biquad_section_states_DF_I_T*, const discrete_biquad_section_coeffs_T*, real_t);
static return_code discrete_biquad_SOS_filter_DF_I_step(discrete_biquad_SOS_filter_DF_I_T*, const vector_generic_T*, vector_generic_T*);
static return_code discrete_biquad_SOS_filter_DF_I_deinit(discrete_biquad_SOS_filter_DF_I_T*);

/**
 * Discrete biquadratic (second order series) filter initialization (direct form I).
 * - initializes system interface -input/output signal ports
 * - registers specific  step function and deinit function
 * - initializes sections states vectors
 * - initializes sections coefficients vectors
 *
 * @see vector_generic_init
 * @see discrete_SISO_system_generic_init
 * @param filter discrete SOS filter structure pointer
 * @param n_stages number of filter stages (sections)
 * @param coeffs_ptr pointer to filter coefficients data, if NULL - function performs dynamic allocation
 * @param states_ptr pointer to filter states data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_biquad_SOS_filter_DF_I_init(discrete_biquad_SOS_filter_DF_I_T* filter, size_t n_stages, discrete_biquad_section_coeffs_T* coeffs_ptr, discrete_biquad_section_states_DF_I_T* states_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_init(&filter->coeffs_vector, discrete_biquad_section_coeffs_T, n_stages, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&filter->states_vector, discrete_biquad_section_states_DF_I_T, n_stages, states_ptr);
	if (returnval != return_OK)
		return returnval;
	discrete_biquad_section_states_DF_I_T initial_states = { 0 };
	vector_generic_set_all(&filter->states_vector, &initial_states);

	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) filter, (discrete_system_step_fcn_T) discrete_biquad_SOS_filter_DF_I_step, (system_deinit_fcn) discrete_biquad_SOS_filter_DF_I_deinit);
}

static real_t discrete_biquad_section_DF_I_step(discrete_biquad_section_states_DF_I_T* filter_states, const discrete_biquad_section_coeffs_T* filter_coeffs, real_t u) {

	real_t y = u * filter_coeffs->b_coeffs[0] + filter_states->input_states[0] * filter_coeffs->b_coeffs[1] + filter_states->input_states[1] * filter_coeffs->b_coeffs[2];
	y -= filter_states->output_states[0] * filter_coeffs->a_coeffs[1] + filter_states->output_states[1] * filter_coeffs->a_coeffs[2];
	y /= filter_coeffs->a_coeffs[0];

	filter_states->input_states[1] = filter_states->input_states[0];
	filter_states->input_states[0] = u;
	filter_states->output_states[1] = filter_states->output_states[0];
	filter_states->output_states[0] = y;
	return y;
}

static return_code discrete_biquad_SOS_filter_DF_I_step(discrete_biquad_SOS_filter_DF_I_T* filter, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	size_t n_stages = vector_length(&filter->states_vector);
	real_t u = *u_ptr;
	real_t y = 0;
	for (uint i = 0; i < n_stages; i++) {
		discrete_biquad_section_states_DF_I_T* filter_states = vector_type_data_ptr(&filter->states_vector,discrete_biquad_section_states_DF_I_T) + i;
		discrete_biquad_section_coeffs_T* filter_coeffs = vector_type_data_ptr(&filter->coeffs_vector,discrete_biquad_section_coeffs_T) + i;
		y = discrete_biquad_section_DF_I_step(filter_states, filter_coeffs, u);
		u = y;
	}
	*y_ptr = y;
	return return_OK;
}

static return_code discrete_biquad_SOS_filter_DF_I_deinit(discrete_biquad_SOS_filter_DF_I_T* filter) {

	return_code returnval = return_OK;

	returnval = vector_generic_deinit(&filter->coeffs_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&filter->states_vector);
	if (returnval != return_OK)
		return returnval;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) filter);
}

/**
 * Sets discrete biquadratic filter coefficients (direct form I).
 * Loads data to stages coefficients vector
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param coeffs_ptr filter coefficients data pointer
 * @return system control library error code
 */
return_code discrete_biquad_SOS_filter_DF_I_coeffs_set(discrete_biquad_SOS_filter_DF_I_T* filter, const discrete_biquad_section_coeffs_T* coeffs_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_load(&filter->coeffs_vector, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}
/**
 * Sets discrete biquadratic filter states (direct form I).
 * Loads data to stages states vector
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param states_ptr filter states data pointer
 * @return control library error code
 */
return_code discrete_biquad_SOS_filter_DF_I_states_set(discrete_biquad_SOS_filter_DF_I_T* filter, const discrete_biquad_section_states_DF_I_T* states_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_load(&filter->states_vector, states_ptr);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Sets all filter states vector elements.
 * @see vector_generic_set_all
 * @param filter filter structure pointer
 * @param states states data structure
 * @return control library error code
 */
return_code discrete_biquad_SOS_filter_DF_I_states_set_all(discrete_biquad_SOS_filter_DF_I_T* filter, discrete_biquad_section_states_DF_I_T states) {

	return_code returnval = return_OK;
	returnval = vector_generic_set_all(&filter->states_vector, &states);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}

static return_code discrete_biquad_SOS_filter_DF_II_step(discrete_biquad_SOS_filter_DF_II_T*, const vector_generic_T*, vector_generic_T*);
static return_code discrete_biquad_SOS_filter_DF_II_deinit(discrete_biquad_SOS_filter_DF_II_T*);

/**
 *  Discrete biquadratic (second order series) filter initialization (direct form II).
 * - initializes system interface -input/output signal ports
 * - registers specific  step function and deinit function
 * - initializes sections states vectors
 * - initializes sections coefficients vectors
 *
 * @see vector_generic_init
 * @see discrete_SISO_system_generic_init
 * @param filter discrete SOS filter structure pointer
 * @param n_stages number of filter stages (sections)
 * @param coeffs_ptr pointer to filter coefficients data, if NULL - function performs dynamic allocation
 * @param states_ptr pointer to filter states data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_biquad_SOS_filter_DF_II_init(discrete_biquad_SOS_filter_DF_II_T* filter, size_t n_stages, discrete_biquad_section_coeffs_T* coeffs_ptr, discrete_biquad_section_states_DF_II_T* states_ptr) {

	return_code returnval = return_OK;

	returnval = vector_type_init(&filter->coeffs_vector, discrete_biquad_section_coeffs_T, n_stages, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_init(&filter->states_vector, discrete_biquad_section_states_DF_II_T, n_stages, states_ptr);
	if (returnval != return_OK)
		return returnval;

	discrete_biquad_section_states_DF_II_T initial_states = { 0 };
	vector_generic_set_all(&filter->states_vector, &initial_states);
	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) filter, (discrete_system_step_fcn_T) discrete_biquad_SOS_filter_DF_II_step, (system_deinit_fcn) discrete_biquad_SOS_filter_DF_II_deinit);
}

static real_t discrete_biquad_section_DF_II_step(discrete_biquad_section_states_DF_II_T* filter_states, const discrete_biquad_section_coeffs_T* filter_coeffs, real_t u) {

	real_t v = u - filter_states->v_states[0] * filter_coeffs->a_coeffs[1] - filter_states->v_states[1] * filter_coeffs->a_coeffs[2];
	v /= filter_coeffs->a_coeffs[0];
	real_t y = v * filter_coeffs->b_coeffs[0] + filter_states->v_states[0] * filter_coeffs->b_coeffs[1] + filter_states->v_states[1] * filter_coeffs->b_coeffs[2];

	filter_states->v_states[1] = filter_states->v_states[0];
	filter_states->v_states[0] = v;
	return y;
}

static return_code discrete_biquad_SOS_filter_DF_II_step(discrete_biquad_SOS_filter_DF_II_T* filter, const vector_generic_T* input_signals, vector_generic_T* output_signals) {
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	size_t n_stages = vector_length(&filter->states_vector);
	real_t u = *u_ptr;
	real_t y = 0;
	for (uint i = 0; i < n_stages; i++) {
		discrete_biquad_section_states_DF_II_T* filter_states = vector_type_data_ptr(&filter->states_vector,discrete_biquad_section_states_DF_II_T) + i;
		discrete_biquad_section_coeffs_T* filter_coeffs = vector_type_data_ptr(&filter->coeffs_vector,discrete_biquad_section_coeffs_T) + i;
		y = discrete_biquad_section_DF_II_step(filter_states, filter_coeffs, u);
		u = y;
	}
	*y_ptr = y;
	return return_OK;
}

static return_code discrete_biquad_SOS_filter_DF_II_deinit(discrete_biquad_SOS_filter_DF_II_T* filter) {

	return_code returnval = return_OK;

	returnval = vector_generic_deinit(&filter->coeffs_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&filter->states_vector);
	if (returnval != return_OK)
		return returnval;

	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) filter);
}

/**
 * Sets discrete biquadratic filter coefficients (direct form II).
 * Loads data to stages coefficients vector
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param coeffs_ptr filter coefficients data pointer
 * @return system control library error code
 */

return_code discrete_biquad_SOS_filter_DF_II_coeffs_set(discrete_biquad_SOS_filter_DF_II_T* filter, const discrete_biquad_section_coeffs_T* coeffs_ptr) {
	return_code returnval = return_OK;
	returnval = vector_type_load(&filter->coeffs_vector, coeffs_ptr);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Sets discrete biquadratic filter states (direct form II).
 * Loads data to stages states vector
 * @see vector_generic_load
 * @param filter filter structure pointer
 * @param states_ptr filter states data pointer
 * @return control library error code
 */
return_code discrete_biquad_SOS_filter_DF_II_states_set(discrete_biquad_SOS_filter_DF_II_T* filter, const discrete_biquad_section_states_DF_II_T* states_ptr) {
	return_code returnval = return_OK;
	returnval = vector_type_load(&filter->states_vector, states_ptr);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}

/**
 * Sets all filter states vector elements.
 * @see vector_generic_set_all
 * @param filter filter structure pointer
 * @param states filter states data structure
 * @return control library error code
 */
return_code discrete_biquad_SOS_filter_DF_II_states_set_all(discrete_biquad_SOS_filter_DF_II_T* filter, discrete_biquad_section_states_DF_II_T states) {

	return_code returnval = return_OK;
	returnval = vector_generic_set_all(&filter->states_vector, &states);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}

static return_code discrete_state_space_step(discrete_state_space_T*, const vector_generic_T*, vector_generic_T*);
static return_code discrete_state_space_deinit(discrete_state_space_T*);

/**
 * Discrete state space initialization.
 * - initializes system interface -input/output signal ports
 * - registers specific : step function and deinit function
 * - initializes A matrix and B,C vectors according to system order
 * - initializes states x(k+1) and x(k) vectors according to system order
 * @see vector_generic_init
 * @see discrete_SISO_system_generic_init
 * @param state_space state space structure pointer
 * @param order order system
 * @param A pointer to A matrix data, if NULL - function performs dynamic allocation
 * @param B pointer to B vector data, if NULL - function performs dynamic allocation
 * @param C pointer to C vector data, if NULL - function performs dynamic allocation
 * @param Xk_0 pointer to x(k) vector data, if NULL - function performs dynamic allocation
 * @param Xk_1 pointer to x(k+1) vector data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */
return_code discrete_state_space_init(discrete_state_space_T* state_space, size_t order, real_t* A, real_t* B, real_t* C, real_t* Xk_0, real_t* Xk_1) {

	return_code returnval = return_OK;

	returnval = state_space_init((state_space_T*) &state_space->A, order, A, B, C);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&state_space->Xk_0, real_t, order, Xk_0);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&state_space->Xk_1, real_t, order, Xk_1);
	if (returnval != return_OK)
		return returnval;
	vector_type_set_all(&state_space->Xk_1, real_t, 0);
	return discrete_SISO_system_generic_init((discrete_SISO_system_generic_T*) state_space, (discrete_system_step_fcn_T) discrete_state_space_step, (system_deinit_fcn) discrete_state_space_deinit);

}
static return_code discrete_state_space_step(discrete_state_space_T* state_space, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	signal_realtime_T* output_signals_ptr = vector_type_data_ptr(output_signals, signal_realtime_T);
	const real_t* u_ptr = input_signals_ptr[0].ptr;
	real_t* y_ptr = output_signals_ptr[0].ptr;

	return_code returnval = return_OK;
	real_t u = *u_ptr;

	returnval = vector_generic_copy(&state_space->Xk_0, &state_space->Xk_1);
	if (returnval != return_OK)
		return returnval;

#ifdef USE_GSL
	const gsl_matrix A_gsl = matrix_to_gsl_matrix(&state_space->A);
	const gsl_vector B_gsl = vector_real_to_gsl_vector(&state_space->B);
	const gsl_vector C_gsl = vector_real_to_gsl_vector(&state_space->C);
	gsl_vector Xk_0_gsl = vector_real_to_gsl_vector(&state_space->Xk_0);
	gsl_vector Xk_1_gsl = vector_real_to_gsl_vector(&state_space->Xk_1);

	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1.0, &A_gsl, &Xk_0_gsl, 0.0, &Xk_1_gsl));
	if (returnval != return_OK)
		return returnval;
	returnval = gsl_error_code_to_return_code(gsl_blas_daxpy(u, &B_gsl, &Xk_1_gsl));
	if (returnval != return_OK)
		return returnval;
	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&C_gsl, &Xk_0_gsl, y_ptr));
	if (returnval != return_OK)
		return returnval;
#else
	matrix_T B_mat=vector_real_to_col_matrix(&state_space->B);
	matrix_T Xk_0_mat=vector_real_to_col_matrix(&state_space->Xk_0);
	matrix_T Xk_1_mat=vector_real_to_col_matrix(&state_space->Xk_1);

	returnval = matrix_multiply(&state_space->A, &Xk_0_mat,&Xk_1_mat);
	if (returnval != return_OK)
	return returnval;
	returnval = matrix_element_wise_operation_and_scalar_operation(&B_mat, &Xk_1_mat, &Xk_1_mat, u, real_sum, real_mul);
	if (returnval != return_OK)
	return returnval;
	returnval =vector_real_dot_product(&state_space->C, &state_space->Xk_0,y_ptr);
	if (returnval != return_OK)
		return returnval;
#endif

	return return_OK;
}
static return_code discrete_state_space_deinit(discrete_state_space_T* state_space) {

	return_code returnval = return_OK;

	returnval = state_space_deinit((state_space_T*) &state_space->A);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_generic_deinit(&state_space->Xk_0);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_generic_deinit(&state_space->Xk_1);
	if (returnval != return_OK)
		return returnval;
	return discrete_SISO_system_generic_deinit((discrete_SISO_system_generic_T*) state_space);

}


/**
 * Loads data to state space states vector x(k+1).
 * @see vector_generic_load
 *
 * @param state_space state space structure pointer
 * @param X_ptr pointer to x(k+1) states values
 * @return system control library error code
 */
return_code discrete_state_space_states_set(discrete_state_space_T* state_space, const real_t* X_ptr) {
	return vector_generic_load(&state_space->Xk_1, X_ptr);
}

/**
 * Sets all state space states to scalar value.
 * @see matrix_set_all_elements
 *
 * @param state_space state space structure pointer
 * @param X_value states scalar value
 * @return system control library error code
 */
return_code discrete_state_space_states_set_all(discrete_state_space_T* state_space, real_t X_value) {
	return vector_type_set_all(&state_space->Xk_1,real_t, X_value);
}
