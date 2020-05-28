#include "static_systems.h"


static return_code static_MIMO_system_generic_deinit(static_MIMO_system_generic_T* system) {

	system->eval_fcn = NULL;
	return return_OK;
}

static return_code static_SISO_system_generic_deinit(static_SISO_system_generic_T* system) {
	system->input = 0;
	system->output = 0;
	system->eval_fcn = NULL;
	return return_OK;
}

static return_code static_SISO_system_generic_init(static_SISO_system_generic_T* system, static_system_eval_fcn_T eval_fcn, system_deinit_fcn deinit_fcn) {

	return_code returnval = return_OK;
	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) static_SISO_system_generic_deinit;
	returnval = system_interface_init(&system->interface, system_type_static_system, system_time_domain_discrete | system_time_domain_continuous, 1, &system->input, 1, &system->output, deinit_fcn);
	if (returnval != return_OK)
		return returnval;
	system->input = 0;
	system->output = 0;
	system->eval_fcn = eval_fcn;
	return return_OK;
}

static return_code static_MIMO_system_generic_init(static_MIMO_system_generic_T* system, static_system_eval_fcn_T eval_fcn, size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, system_deinit_fcn deinit_fcn) {

	return_code returnval = return_OK;
	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) static_MIMO_system_generic_deinit;

	returnval = system_interface_init(&system->interface, system_type_static_system, system_time_domain_discrete | system_time_domain_continuous, input_port_width, input_port_ptr, output_port_width, output_port_ptr, deinit_fcn);
	if (returnval != return_OK)
		return returnval;
	memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);
	system->eval_fcn = eval_fcn;
	return return_OK;

}

/**
 * Evaluates static system with direct scalar input and output signals.
 * Explicit signals arguments (non interconnected systems)
 *
 * @param system static system structure pointer
 * @param u input signal value
 * @param y_ptr output signal pointer
 * @return system control library error code
 */
return_code static_system_eval_direct_scalar(static_system_generic_T* system, real_t u, real_t* y_ptr) {

	vector_generic_T u_vector = { 0 }, y_vector = { 0 };
	signal_realtime_T signal_u, signal_y;
	signal_u.owner = NULL;
	signal_u.ptr = &u;
	signal_y.owner = NULL;
	signal_y.ptr = y_ptr;
	vector_type_init(&u_vector, signal_realtime_T, 1, &signal_u);
	vector_type_init(&y_vector, signal_realtime_T, 1, &signal_y);
	return static_system_eval_direct(system, &u_vector, &y_vector);

}

static return_code math_scalar_function(static_system_generic_scalar_function_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t u = *input_signals_ptr[0].ptr;
	real_t y = system->scalar_function(u);
	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;

}

/**
 * Initializes static system scalar math function structure.
 * @param system system structure pointer
 * @param function scalar math function pointer
 * @return system control library error code
 */
return_code static_system_math_scalar_function_init(static_system_generic_scalar_function_T* system, real_unary_operator_T function) {
#ifdef ASSERT_NULLPTR
	if (system == NULL || function == NULL)
		return return_NULLPTR;
#endif
	system->scalar_function = function;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) math_scalar_function, NULL);

}

static return_code saturation_fcn(static_system_saturation_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	real_t u = *input_signals_ptr[0].ptr;

	real_t y = u;

	y = CLIP_TOP(y, system->upper_limit);
	y = CLIP_BOTTOM(y, system->lower_limit);

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;

	return return_OK;
}

/**
 * Initializes saturation static system structure.
 * @param system system structure pointer
 * @param lower_limit output high bound
 * @param upper_limit output low bound
 * @return system control library error code
 */
return_code static_system_saturation_init(static_system_saturation_T* system, real_t lower_limit, real_t upper_limit) {

	system->lower_limit = lower_limit;
	system->upper_limit = upper_limit;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) saturation_fcn, NULL);
}

static return_code dead_zone_fcn(static_system_dead_zone_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t u = *input_signals_ptr[0].ptr;
	real_t y = 0;
	if (u > system->threshold_high)
		y = u - system->threshold_high;
	if (u < system->threshold_low)
		y = u - system->threshold_low;
	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;

	return return_OK;

}

/**
 * Initializes deadzone static system structure.
 * @param system system structure pointer
 * @param threshold_low input un-sensitivity low threshold
 * @param threshold_high input un-sensitivity high threshold
 * @return system control library error code
 */
return_code static_system_dead_zone_init(static_system_dead_zone_T* system, real_t threshold_low, real_t threshold_high) {
	system->threshold_low = threshold_low;
	system->threshold_high = threshold_high;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) dead_zone_fcn, NULL);
}

static return_code dead_zone_cancelation_fcn(static_system_dead_zone_cancelation_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t u = *input_signals_ptr[0].ptr;
	real_t y = u;

	if (fabs(u) < system->treshold)
		y = 0;
	else
		y = u + SIGN(u) * system->deadzone;

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;

	return return_OK;

}
/**
 *
 *
 * @param system system structure pointer
 * @param deadzone
 * @param treshold
 * @return system control library error code
 */
return_code static_system_dead_zone_cancelation_init(static_system_dead_zone_cancelation_T* system, real_t deadzone, real_t treshold) {
	system->deadzone = deadzone;
	system->treshold = treshold;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) dead_zone_cancelation_fcn, NULL);
}

static return_code relay_fcn(static_system_relay_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	real_t u = *input_signals_ptr[0].ptr;
	real_t y = system->y_last;

	if (system->y_last == system->value_on) {
		if (u < system->treshold_switch_off)
			y = system->value_off;
	} else {
		if (u > system->treshold_switch_on)
			y = system->value_on;
	}
	system->y_last = y;

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;

}

/**
 * Initializes deadzone static system structure.
 * @param system system structure pointer
 * @param treshold_switch_on hysteresis on state threshold
 * @param treshold_switch_off hysteresis off state threshold
 * @param value_on on state output value
 * @param value_off off state output value
 * @return system control library error code
 */
return_code static_system_relay_init(static_system_relay_T* system, real_t treshold_switch_on, real_t treshold_switch_off, real_t value_on, real_t value_off) {

	system->treshold_switch_on = treshold_switch_on;
	system->treshold_switch_off = treshold_switch_off;
	system->value_on = value_on;
	system->value_off = value_off;
	system->y_last = 0;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) relay_fcn, NULL);

}

static return_code gain_fcn(static_system_gain_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	real_t u = *input_signals_ptr[0].ptr;
	real_t y = u * system->gain;

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;

	return return_OK;
}

/**
 * Initializes gain static system structure.
 * @param system system structure pointer
 * @param gain gain value
 * @return system control library error code
 */
return_code static_system_gain_init(static_system_gain_T* system, real_t gain) {
	system->gain = gain;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) gain_fcn, NULL);
}
static return_code quantizer_fcn(static_system_quantizer_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 1 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	real_t u = *input_signals_ptr[0].ptr;
	real_t y = quantize(u, system->step_size);
	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;
}
/**
 * Initializes quantizer static system structure.
 * @param system system structure pointer
 * @param step_size
 * @return system control library error code
 */
return_code static_system_quantizer_init(static_system_quantizer_T* system, real_t step_size) {

	system->step_size = step_size;
	return static_SISO_system_generic_init((static_SISO_system_generic_T*) system, (static_system_eval_fcn_T) quantizer_fcn, NULL);
}

static return_code product_fcn(static_system_product_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 2 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t u_1 = *input_signals_ptr[0].ptr;
	real_t u_2 = *input_signals_ptr[1].ptr;

	real_t y = u_1 * u_2;

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;
}

/**
 * Initializes product static system structure.
 * @param system system structure pointer
 * @return system control library error code
 */
return_code static_system_product_init(static_system_product_T* system) {

	return static_MIMO_system_generic_init((static_MIMO_system_generic_T*) system, (static_system_eval_fcn_T) product_fcn, 2, system->input, 1, system->output, NULL);
}

static return_code sum_fcn(static_system_sum_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 2 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t u_1 = *input_signals_ptr[0].ptr;
	real_t u_2 = *input_signals_ptr[1].ptr;
	real_t y = u_1 + u_2;
	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;

	return return_OK;

}
/**
 * Initializes sum static system structure.
 * @param system system structure pointer
 * @return system control library error code
 */
return_code static_system_sum_init(static_system_sum_T* system) {

	return static_MIMO_system_generic_init((static_MIMO_system_generic_T*) system, (static_system_eval_fcn_T) sum_fcn, 2, system->input, 1, system->output, NULL);
}

static return_code sub_fcn(static_system_sub_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != 2 || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);
	real_t u_1 = *input_signals_ptr[0].ptr;
	real_t u_2 = *input_signals_ptr[1].ptr;
	real_t y = u_1 - u_2;

	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;

}

/**
 * Initializes sub static system structure.
 * @param system system structure pointer
 * @return system control library error code
 */
return_code static_system_sub_init(static_system_sub_T* system) {

	return static_MIMO_system_generic_init((static_MIMO_system_generic_T*) system, (static_system_eval_fcn_T) sub_fcn, 2, system->input, 1, system->output, NULL);
}

static return_code port_operator_fcn(static_system_port_operator_T* system, const vector_generic_T* input_signals, vector_generic_T* output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (vector_length(input_signals) != vector_length(&system->operators) || vector_length(output_signals) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	const signal_realtime_T* input_signals_ptr = vector_type_data_ptr(input_signals, signal_realtime_T);

	real_t y = 0;
	for (uint i = 0; i < vector_length(&system->operators); i++) {
		real_binary_operator_T operator = vector_type_at(&system->operators, real_binary_operator_T, i);
		real_t u = *(input_signals_ptr[i].ptr);
		y = operator(y, u);
	}
	*vector_type_at(output_signals,signal_realtime_T, 0).ptr = y;
	return return_OK;

}

static return_code port_operator_deinit(static_system_port_operator_T* system) {

	vector_generic_deinit(&system->operators);
	vector_generic_deinit(&system->input_signals);
	return static_MIMO_system_generic_deinit((static_MIMO_system_generic_T*) system);
}

/**
 * Initializes port operator static system structure.
 * @param system system structure pointer
 * @param input_port_size number of input signals - input port length
 * @param operators array of operators - function pointers
 * @return system control library error code
 */
return_code static_system_port_operator_init(static_system_port_operator_T* system, size_t input_port_size, real_binary_operator_T operators[]) {

	return_code returnval = return_OK;
	returnval = vector_type_init(&system->operators, real_binary_operator_T, input_port_size, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_load(&system->operators, operators);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&system->input_signals, real_t, input_port_size, NULL);
	if (returnval != return_OK)
		return returnval;

	return static_MIMO_system_generic_init((static_MIMO_system_generic_T*) system, (static_system_eval_fcn_T) port_operator_fcn, input_port_size, vector_generic_get_data_ptr(&system->input_signals), 1, system->output, (system_deinit_fcn) port_operator_deinit);
}

