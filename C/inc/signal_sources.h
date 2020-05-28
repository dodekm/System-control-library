/**
 * @file signal_sources.h
 */

#ifndef SIGNAL_SOURCE_H_
#define SIGNAL_SOURCE_H_

#include "systems.h"
#include "signal_process.h"

/** @addtogroup signal_sources Signal sources
 * @brief Time dependent signal sources systems.
 * Signal source evaluation interface via callback function.
 * Interface supports time offseting of ODE solver.
 * @note Output of signal source is only time dependent and stateless
 * # Implemented functionality
 * 	- constant signal
 * 	- sine signal
 * 	- square signal
 * 	- step signal
 * 	- ramp and saw signal
 * 	- special sampled signal and timeseries signal sources
 *
 * @{
 */

struct signal_source_generic;

/**
 * Signal source evaluation callback function prototype.
 * @param - pointer to signal source structure
 * @param - time to evaluate for
 * @param - target signal vector of type @ref signal_realtime_T
 * @return - system control library error code
 */
typedef return_code (*signal_source_fcn)(struct signal_source_generic*, real_t, vector_generic_T*);

/**
 * Generic signal source system structure.
 * Contains system interface and single output signal data.
 */
typedef struct signal_source_generic {
	system_generic_T interface; ///< generic system interface
	real_t output; ///< output signal data
	signal_source_fcn function; ///< evaluation callback function pointer
} signal_source_generic_T;

/**
 *
 */
typedef struct signal_source_timeseries {
	signal_source_generic_T; ///< generic inherited signal source structure
	signal_timeseries_T* timeseries; ///< timeseries structure pointer
} signal_source_timeseries_T;

/**
 *
 */
typedef struct signal_source_sampled {
	signal_source_generic_T; ///< generic inherited signal source structure
	signal_sampled_T* sampled_data; ///< sampled signal structure pointer
} signal_source_sampled_T;

/**
 * Constant level signal generator source structure.
 * Generates constant signal defined as:
 * y(t)=constant
 */
typedef struct signal_source_constant {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t constant; ///< constant value
} signal_source_constant_T;

/**
 *
 * Sine signal generator source structure.
 * Generates square signal defined as:
 * y(t)=amplitude*sin(2*pi*t/period)+offset
 */
typedef struct signal_source_sine {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t period; ///< signal period
	real_t amplitude; ///< signal amplitude
	real_t offset;   ///< signal offset
} signal_source_sine_T;

/**
 * Square signal generator source structure.
 * Generates square signal defined as:
 * y(t)= amplitude * ((mod(time, period) / period) > 0.5 ? 1 : -1) + offset;
 */
typedef struct signal_source_square {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t period; ///< signal period
	real_t amplitude; ///< signal amplitude
	real_t offset; ///< signal offset
} signal_source_square_T;

/**
 * Sawtooth signal generator source structure.
 * Generates sawtooth signal defined as:
 * y(t)=amplitude * ((mod(time, period) / period) * 2.0 - 1.0) +offset;
 */
typedef struct signal_source_saw {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t period;  ///< signal period
	real_t amplitude; ///< signal amplitude
	real_t offset; ///< signal offset
} signal_source_saw_T;

/**
 * Step signal generator source.
 * Generates discontinuous step signal defined by initial/final value and step time
 */
typedef struct signal_source_step {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t step_time; ///< time to step form initial to final value
	real_t initial_state; ///< initial output value
	real_t final_state; ///< final output value
} signal_source_step_T;

/**
 * Linear signal generator source.
 * Generates linear slope signal defined by proportional gain
 */
typedef struct signal_source_ramp {
	signal_source_generic_T; ///< generic inherited signal source structure
	real_t slope; ///< linear gain - output signal slope (derivative)
} signal_source_ramp_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code signal_source_generic_deinit(signal_source_generic_T*);
return_code signal_source_generic_init(signal_source_generic_T*, signal_source_fcn, system_deinit_fcn);

/**
 * Evaluation of signal source using explicit signal vector pointer argument.
 * @param signal_source signal source structure pointer
 * @param time time to evaluate for
 * @param y_vector destination signal vector of @ref signal_realtime_T
 * @return system control library error code
 */
static inline return_code signal_source_eval_direct(signal_source_generic_T* signal_source, real_t time, vector_generic_T* y_vector) {
	return signal_source->function(signal_source, time, y_vector);
}

/**
 * Evaluation of signal source using explicit scalar value pointer argument.
 * @param signal_source signal source structure pointer
 * @param time time to evaluate for
 * @param y_ptr
 * @return system control library error code
 */
static inline return_code signal_source_eval_direct_scalar(signal_source_generic_T* signal_source, real_t time, real_t* y_ptr) {

	vector_generic_T y_vector = { 0 };
	signal_realtime_T signal_y;
	signal_y.owner = NULL;
	signal_y.ptr = y_ptr;
	vector_type_init(&y_vector, signal_realtime_T, 1, &signal_y);
	return signal_source->function(signal_source, time, &y_vector);
}

/**
 * Evaluation of signal source using implicit system connections.
 * @param signal_source signal source structure pointer
 * @param time time to evaluate for
 * @return system control library error code
 */
static inline return_code signal_source_eval_linked(signal_source_generic_T* signal_source, real_t time) {
	return signal_source_eval_direct(signal_source, time, system_output_port_vector_ptr(&signal_source->interface));
}

return_code signal_source_timeseries_init(signal_source_timeseries_T*, signal_timeseries_T*);
return_code signal_source_sampled_init(signal_source_sampled_T*, signal_sampled_T*);
return_code signal_source_constant_init(signal_source_constant_T*, real_t);
return_code signal_source_sine_init(signal_source_sine_T*, real_t, real_t, real_t);
return_code signal_source_square_init(signal_source_square_T*, real_t, real_t, real_t);
return_code signal_source_saw_init(signal_source_saw_T*, real_t, real_t, real_t);
return_code signal_source_step_init(signal_source_step_T*, real_t, real_t, real_t);
return_code signal_source_ramp_init(signal_source_ramp_T*, real_t);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_SOURCE_H_ */
