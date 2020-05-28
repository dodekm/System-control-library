
#include "signal_sources.h"

/**
 *
 * @param system signal source system structure pointer
 * @return system control library error code
 */
return_code signal_source_generic_deinit(signal_source_generic_T* system) {

	system->output = 0;
	system->function = NULL;
	return return_OK;

}

/**
 *
 * @param system signal source system structure pointer
 * @param function
 * @param deinit_fcn
 * @return system control library error code
 */
return_code signal_source_generic_init(signal_source_generic_T* system, signal_source_fcn function, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;
	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) signal_source_generic_deinit;
	returnval = system_interface_init(&system->interface, system_type_signal_source, system_time_domain_continuous | system_time_domain_discrete, 0, NULL, 1, &system->output, deinit_fcn);
	if (returnval != return_OK)
		return returnval;
	system->output = 0;
	system->function = function;
	return return_OK;
}

static return_code signal_source_timeseries_fcn(signal_source_timeseries_T* signal_source, real_t time, vector_generic_T* y_vector) {

	return signal_timeseries_read(signal_source->timeseries, time,vector_type_at(y_vector,signal_realtime_T,0).ptr);
}

/**
 *
 * @param signal_source signal source system structure pointer
 * @param timeseries
 * @return system control library error code
 */
return_code signal_source_timeseries_init(signal_source_timeseries_T* signal_source, signal_timeseries_T* timeseries) {

#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	if (signal_source == NULL || timeseries == NULL)
		return return_NULLPTR;
	returnval = vector_assert(&timeseries->time_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&timeseries->values_vector);
	if (returnval != return_OK)
		return returnval;
#endif
	signal_source->timeseries = timeseries;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_timeseries_fcn, NULL);

}

static return_code signal_source_sampled_fcn(signal_source_sampled_T* signal_source, real_t time, vector_generic_T* y_vector) {

	return signal_sampled_read_value(signal_source->sampled_data, time, vector_type_at(y_vector,signal_realtime_T,0).ptr);

}

/**
 *
 * @param signal_source signal source system structure pointer
 * @param sampled_data
 * @return system control library error code
 */
return_code signal_source_sampled_init(signal_source_sampled_T* signal_source, signal_sampled_T* sampled_data) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	if (signal_source == NULL || sampled_data == NULL)
		return return_NULLPTR;
	returnval = vector_assert((vector_generic_T*) (sampled_data));
	if (returnval != return_OK)
		return returnval;
#endif
	signal_source->sampled_data = sampled_data;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_sampled_fcn, NULL);
}

static return_code signal_source_constant_fcn(signal_source_constant_T* signal_source, real_t time, vector_generic_T* y_vector) {

	(void) time;
	*vector_type_at(y_vector,signal_realtime_T,0).ptr = signal_source->constant;
	return return_OK;
}

/**
 *
 * @param signal_source signal source system structure pointer
 * @param constant
 * @return system control library error code
 */
return_code signal_source_constant_init(signal_source_constant_T* signal_source, real_t constant) {
#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->constant = constant;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_constant_fcn, NULL);
}

static return_code signal_source_sine_fcn(signal_source_sine_T* signal_source, real_t time, vector_generic_T* y_vector) {

	*vector_type_at(y_vector,signal_realtime_T,0).ptr = signal_source->amplitude * (sin(2 * M_PI * time / signal_source->period)) + signal_source->offset;
	return return_OK;
}

/**
 *
 * @param signal_source signal source system structure pointer
 * @param period
 * @param amplitude
 * @param offset
 * @return system control library error code
 */

return_code signal_source_sine_init(signal_source_sine_T* signal_source, real_t period, real_t amplitude, real_t offset) {
#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->period = period;
	signal_source->amplitude = amplitude;
	signal_source->offset = offset;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_sine_fcn, NULL);

}

static return_code signal_source_square_fcn(signal_source_square_T* signal_source, real_t time,vector_generic_T* y_vector) {

	*vector_type_at(y_vector,signal_realtime_T,0).ptr = signal_source->amplitude * ((real_mod(time, signal_source->period) / signal_source->period) > 0.5 ? 1 : -1) + signal_source->offset;
	return return_OK;
}


/**
 *
 * @param signal_source signal source system structure pointer
 * @param period
 * @param amplitude
 * @param offset
 * @return system control library error code
 */
return_code signal_source_square_init(signal_source_square_T* signal_source, real_t period, real_t amplitude, real_t offset) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->period = period;
	signal_source->amplitude = amplitude;
	signal_source->offset = offset;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_square_fcn, NULL);

}

static return_code signal_source_saw_fcn(signal_source_saw_T* signal_source, real_t time, vector_generic_T* y_vector) {
	*vector_type_at(y_vector,signal_realtime_T,0).ptr = signal_source->amplitude * ((fmod(time, signal_source->period) / signal_source->period) * 2.0 - 1.0) + signal_source->offset;
	return return_OK;
}

return_code signal_source_saw_init(signal_source_saw_T* signal_source, real_t period, real_t amplitude, real_t offset) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->period = period;
	signal_source->amplitude = amplitude;
	signal_source->offset = offset;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_saw_fcn, NULL);

}

static return_code signal_source_step_fcn(signal_source_step_T* signal_source, real_t time,vector_generic_T* y_vector) {
	*vector_type_at(y_vector,signal_realtime_T,0).ptr = time < signal_source->step_time ? signal_source->initial_state : signal_source->final_state;
	return return_OK;
}

/**
 *
 * @param signal_source signal source system structure pointer
 * @param step_time
 * @param initial_state
 * @param final_state
 * @return system control library error code
 */
return_code signal_source_step_init(signal_source_step_T* signal_source, real_t step_time, real_t initial_state, real_t final_state) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->initial_state = initial_state;
	signal_source->final_state = final_state;
	signal_source->step_time = step_time;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_step_fcn, NULL);

}

static return_code signal_source_ramp_fcn(signal_source_ramp_T* signal_source, real_t time, vector_generic_T* y_vector) {

	*vector_type_at(y_vector,signal_realtime_T,0).ptr = time * signal_source->slope;
	return return_OK;
}
/**
 *
 * @param signal_source signal source system structure pointer
 * @param slope
 * @return system control library error code
 */
return_code signal_source_ramp_init(signal_source_ramp_T* signal_source, real_t slope) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
#endif
	signal_source->slope = slope;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_ramp_fcn, NULL);

}


