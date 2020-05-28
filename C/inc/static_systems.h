/**
 * @file static_systems.h
 */

#ifndef STATIC_SYSTEMS_H_
#define STATIC_SYSTEMS_H_

#include "systems.h"

/** @addtogroup static_systems Static systems
 * @ingroup systems
 * @brief  Static systems interface and implementations of basic static systems.
 * Static systems are stateless.
 * Evaluation function callback interface - direct and linked mode.
 * Generic static systems interface.
 *
 * # Implementation of static systems
 * 	- saturation
 * 	- deadzone
 * 	- gain
 * 	- relay(hysteresis)
 * 	- quantizer
 * 	- sum,substract,product
 * 	- universal port operator
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif


struct static_system_generic;

/**
 * Static system evaluation function prototype.
 * Every static system must implement this callback function
 *
 * @param generic static system structure pointer
 * @param input signals vector
 * @param output signals vector
 * @return system control library error code
 */
typedef return_code (*static_system_eval_fcn_T)(struct static_system_generic*,const vector_generic_T*, vector_generic_T*);


/**
 * Generic static system interface structure.
 * Contains generic system interface for input/output ports signals
 * Contains system evaluation callback function pointer for system specific behavior
 */
typedef struct static_system_generic {
	system_generic_T interface; ///<generic system interface
	static_system_eval_fcn_T eval_fcn; ///<system evaluation function pointer
} static_system_generic_T;

/**
 * Generic static system with single input and single output port.
 * Contains generic static system structure
 * Contains input and output signal data
 */
typedef struct static_SISO_system_generic {
	static_system_generic_T;
	real_t input; ///< input signal data
	real_t output; ///<output signal data
} static_SISO_system_generic_T;

/**
 * Generic static system with multiple input and multiple output port.
 * Contains generic static system structure
 * Input/output signals data are system specific
 */
typedef struct static_MIMO_system_generic {
	static_system_generic_T;
} static_MIMO_system_generic_T;



/**
 * Evaluates static system with direct input and output signals.
 * Explicit signals arguments (non interconnected systems)
 *
 * @param system static system structure pointer
 * @param input_args vector of @ref signal_realtime_T input signals
 * @param output_args vector of @ref signal_realtime_T output signals
 * @return system control library error code
 */
static inline return_code static_system_eval_direct(static_system_generic_T* system,const vector_generic_T* input_args, vector_generic_T* output_args) {
	return system->eval_fcn(system, input_args, output_args);
}


/**
 * Evaluates static system with linked input and output signals.
 * Implicit signals arguments (using interconnected systems)
 *
 * @param system static system structure pointer
 * @return system control library error code
 */
static inline return_code static_system_eval_linked(static_system_generic_T* system) {
	return static_system_eval_direct(system, system_input_port_vector_ptr(&system->interface), system_output_port_vector_ptr(&system->interface));
}


return_code static_system_eval_direct_scalar(static_system_generic_T*, real_t, real_t*);

/**
 * Static system implementing generic scalar function.
 * y=f(u)
 * System uses unary operator function callback for evaluation e.g. sin(),abs() ...
 */
typedef struct static_system_generic_scalar_function {
	static_SISO_system_generic_T; ///<generic static SISO system
	real_unary_operator_T scalar_function; ///<unary operator callback
} static_system_generic_scalar_function_T;

return_code static_system_math_scalar_function_init(static_system_generic_scalar_function_T*, real_unary_operator_T);

/**
 * Saturation system structure.
 * Output is ceiling and floor limited by saturaation values
 * Represents discontinuous and nonlinear static system type
 * Contains generic SISO static system structure
 * Contains data for saturation high and low limit
 *
 */
typedef struct static_system_saturation {
	static_SISO_system_generic_T;
	real_t lower_limit; ///<output low bound
	real_t upper_limit; ///<output high bound
} static_system_saturation_T;

return_code static_system_saturation_init(static_system_saturation_T*, real_t, real_t);


/**
 * Deadzone system structure.
 * If input is within specific range, output remains zero
 * Represents discontinuous and nonlinear static system type
 * Contains generic SISO static system structure
 * Contains data for deadzone low and high threshold
 *
 */
typedef struct static_system_dead_zone {
	static_SISO_system_generic_T;
	real_t threshold_low; ///<input un-sensitivity low threshold
	real_t threshold_high;///<input un-sensitivity highthreshold
} static_system_dead_zone_T;

return_code static_system_dead_zone_init(static_system_dead_zone_T*, real_t, real_t);

typedef struct static_system_dead_zone_cancelation {
	static_SISO_system_generic_T;
	real_t deadzone;
	real_t treshold;
} static_system_dead_zone_cancelation_T;

return_code static_system_dead_zone_cancelation_init(static_system_dead_zone_cancelation_T*, real_t, real_t);


/**
 * Two state discontinuous system (regulator) with hysteresis.
 * Hysteresis needs a kind of feedback, so system is not pure stateless
 * Represents discontinuous and nonlinear system type
 * Contains generic SISO static system structure
 * Contains data for switching thresholds, output values and hysteresis state
 */
typedef struct static_system_relay {
	static_SISO_system_generic_T;
	real_t treshold_switch_on; ///<hysteresis on state threshold
	real_t treshold_switch_off;///<hysteresis off state threshold
	real_t value_on; ///<on state output value
	real_t value_off; ///<off state output value
	real_t y_last; ///<output state memory
} static_system_relay_T;

return_code static_system_relay_init(static_system_relay_T*, real_t, real_t, real_t, real_t);


/**
 * Represents continuous linear static system type
 * Contains generic SISO static system structure
 * Contains data for gain value
 */
typedef struct static_system_gain {
	static_SISO_system_generic_T;
	real_t gain; ///< system gain value
} static_system_gain_T;



return_code static_system_gain_init(static_system_gain_T*, real_t);

/**
 * Input signals quantizer operator system.
 * Represents discontinuous linear static system type
 * Contains generic SISO static system structure
 */
typedef struct static_system_quantizer {
	static_SISO_system_generic_T;
	real_t step_size; ///< quantization interval
} static_system_quantizer_T;

return_code static_system_quantizer_init(static_system_quantizer_T*, real_t);


/**
 * Input signals product operator system.
 * y=u_1*u_2
 * Represents continuous nonlinear static system type
 * Contains generic MIMO static system structure
 * Contains data for input and output signals
 */
typedef struct static_system_product {
	static_MIMO_system_generic_T;
	real_t input[2];  ///<input port signals data
	real_t output[1]; ///<output port signals data
} static_system_product_T;

return_code static_system_product_init(static_system_product_T*);


/**
 * Input signals sum operator system.
 * y=u_1+u_2
 * Represents continuous linear static system type
 * Contains generic MIMO static system structure
 * Contains data for input and output signals
 */
typedef struct static_system_sum {
	static_MIMO_system_generic_T;
	real_t input[2];  ///<input port signals data
	real_t output[1]; ///<output port signals data
} static_system_sum_T;

return_code static_system_sum_init(static_system_sum_T*);

/**
 * Input signals sum operator system.
 * y=u_1-u_2
 * Represents continuous linear static system type
 * Contains generic MIMO static system structure
 * Contains data for input and output signals
 */
typedef struct static_system_sub {
	static_MIMO_system_generic_T;
	real_t input[2];  ///<input port signals data
	real_t output[1]; ///<output port signals data
} static_system_sub_T;

return_code static_system_sub_init(static_system_sub_T*);

/**
 * Input signals universal operator MISO system.
 * Operators functions are provided via pointers array
 * Contains generic MIMO static system structure
 * Contains data for output signal
 * Input signals data are dynamically allocated
 */
typedef struct static_system_port_operator {
	static_MIMO_system_generic_T;
	vector_generic_T operators; ///<operators vector of type @ref real_binary_operator_T
	vector_generic_T input_signals; ///<input port signals data vector
	real_t output[1]; ///<output port signal data
} static_system_port_operator_T;

return_code static_system_port_operator_init(static_system_port_operator_T*, size_t, real_binary_operator_T[]);

#ifdef __cplusplus
}
#endif

/** @} */
#endif
