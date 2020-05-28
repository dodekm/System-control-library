/**
 * @file discrete_systems.h
 */

#ifndef DISCRETE_SYSTEMS_H_
#define DISCRETE_SYSTEMS_H_

#include "systems.h"
#include "matrix.h"

/**
 * @addtogroup discrete_systems Discrete systems
 * @ingroup systems
 * @brief Discrete systems structures and interface.
 *
 * # Implemented functionality:
 * 	- transfer functions
 * 	- discrete derivative and integration
 * 	- discrete filters - IIR, FIR, SOS
 * 	- discrete state space with GSL/BLAS support
 * 	- PSD and PID regulators
 * 	- RST polynomial regulator
 * 	- output saturation support and anti-windup
 * @{
 */

struct discrete_system_generic;
struct discrete_SISO_system_generic;
struct discrete_MIMO_system_generic;

/**
 * Discrete system step function prototype.
 * Updates system states and output
 * @param - discrete system structure pointer
 * @param - input signals vector pointer
 * @param - output signals vector pointer
 * @return system control library error code
 */
typedef return_code (*discrete_system_step_fcn_T)(struct discrete_system_generic*,const vector_generic_T*, vector_generic_T*);

/**
 * Generic discrete system structure.
 */
typedef struct discrete_system_generic {
	system_generic_T interface; ///< generic system interface
	discrete_system_step_fcn_T step_fcn;	///<step function pointer - updates system states and output
} discrete_system_generic_T;

/**
 * Discrete SISO system structure.
 * Contains data for single input and output signals
 */
typedef struct discrete_SISO_system_generic {
	discrete_system_generic_T; ///< inherited generic discrete system structure
	real_t input;	///<single input port variable
	real_t output; ///<single output port variable
} discrete_SISO_system_generic_T;

/**
 * Discrete MIMO system structure.
 */
typedef struct discrete_MIMO_system_generic {
	discrete_system_generic_T; ///< inherited generic discrete system structure
} discrete_MIMO_system_generic_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code discrete_system_generic_step_direct_scalar(discrete_system_generic_T*, real_t, real_t*);

/**
 * Evaluates discrete system step function with direct input and output signals.
 * Updates system states and output.
 * Explicit signals arguments (non interconnected systems).
 * @param system discrete system to step
 * @param u_vector vector of @ref signal_realtime_T input signals
 * @param y_vector vector of @ref signal_realtime_T output signals
 * @return system control library error code
 */
static inline return_code discrete_system_generic_step_direct(discrete_system_generic_T* system,const vector_generic_T* u_vector, vector_generic_T* y_vector) {
	return system->step_fcn(system, u_vector, y_vector);
}
/**
 * Evaluates discrete system step function with linked input and output signals.
 * Updates system states and output.
 * Implicit signals arguments (using interconnected systems).
 * @param system discrete system to step
 * @return system control library error code
 */
static inline return_code discrete_system_generic_step_linked(discrete_system_generic_T* system) {
	return system->step_fcn(system, system_input_port_vector_ptr(&system->interface), system_output_port_vector_ptr(&system->interface));
}

/**
 * Discrete state buffer structure.
 * Circular buffer for signal states history.
 * Contains inherited generic vector and indexing counter.
 */
typedef struct state_buffer {
	vector_generic_T; ///< inherited real number vector structure
	uint shift_idx; ///<indexing counter variable for circular buffering
} state_buffer_T;

/**
 * Generic discrete transfer function structure.
 */
typedef struct discrete_transfer_function {
	discrete_SISO_system_generic_T; ///<inherited generic discrete SISO system structure
	transfer_function_T; ///<numerator/denominator coefficients vector (ascending power of z)
	state_buffer_T input_states;	///< discrete circular state buffer for input signal history
	state_buffer_T output_states;  ///< discrete circular state  buffer for output signal history

#ifdef USE_CLIP
	real_t saturation_high; ///< output saturation high limit, default set to inf
	real_t saturation_low; ///< output saturation low limit, default set to -inf
#endif
} discrete_transfer_function_T;

return_code discrete_transfer_function_init(discrete_transfer_function_T*, size_t, size_t, real_t*, real_t*, real_t*, real_t*);
return_code discrete_transfer_function_states_set(discrete_transfer_function_T*,const real_t*,const real_t*);
return_code discrete_transfer_function_states_set_all(discrete_transfer_function_T*, real_t, real_t);
#ifdef USE_CLIP
return_code discrete_transfer_function_set_saturation(discrete_transfer_function_T*, real_t, real_t);
#endif

/**
 * Discrete polynomial RST regulator structure.
 * Regulator has two inputs:
 *  1. setpoint
 *  2. feedback
 *
 * Components of regulator:
 *  1. R autoregressive IIR output dynamics
 *  2. S feedback FIR dynamics
 *  3. T feedforward FIR dynamics
 */
typedef struct discrete_RST_regulator {
	discrete_MIMO_system_generic_T; ///<inherited generic discrete MIMO system structure
	state_buffer_T u_states; ///< discrete circular state  buffer for output signal history
	state_buffer_T y_states; ///< discrete circular state  buffer for feedback signal history
	state_buffer_T w_states; ///< discrete circular state  buffer for setpoint signal history
	vector_generic_T R_coeffs; ///< R polynomial coefficients vector (ascending power of z)
	vector_generic_T S_coeffs; ///< S polynomial coefficients vector (ascending power of z)
	vector_generic_T T_coeffs; ///< T polynomial coefficients vector (ascending power of z)
	real_t input[2];
	real_t output[1];
#ifdef USE_CLIP
	real_t saturation_high; ///<output saturation high limit, default set to inf
	real_t saturation_low; ///<output saturation low limit, default set to -inf
#endif
} discrete_RST_regulator_T;

return_code discrete_RST_regulator_init(discrete_RST_regulator_T*, size_t, size_t, size_t, real_t*, real_t*, real_t*, real_t*, real_t*, real_t*);
return_code discrete_RST_regulator_coeffs_set(discrete_RST_regulator_T*,const real_t*,const real_t*,const real_t*);
return_code discrete_RST_regulator_states_set(discrete_RST_regulator_T*,const real_t*,const real_t*,const real_t*);
return_code discrete_RST_regulator_states_set_all(discrete_RST_regulator_T*, real_t, real_t, real_t);
return_code discrete_RST_regulator_create_control_loop(discrete_RST_regulator_T*, discrete_SISO_system_generic_T*, system_generic_T*);
#ifdef USE_CLIP
return_code discrete_RST_regulator_set_saturation(discrete_RST_regulator_T*, real_t, real_t);
#endif

typedef discrete_transfer_function_T discrete_IIR_filter_DF_I_T; ///< discrete infinite impulse response filter in direct form I is defined as transfer function

#define discrete_IIR_filter_DF_I_init 		discrete_transfer_function_init
#define discrete_IIR_filter_DF_I_coeffs_set discrete_transfer_function_coeffs_set
#define discrete_IIR_filter_DF_I_states_set	discrete_transfer_function_states_set
#define discrete_IIR_filter_DF_I_states_set_all discrete_transfer_function_states_set_all

/**
 * Discrete infinite impulse response filter in direct form II.
 * Structure contains one state buffer.
 * Filter dynamic is equivalent to direct form I @ref discrete_IIR_filter_DF_I_T.
 */
typedef struct discrete_IIR_filter_DF_II {
	discrete_SISO_system_generic_T; ///<inherited generic discrete SISO system structure
	transfer_function_T; ///< filter numerator/denominator coefficients vector (ascending power of z)
	state_buffer_T states; ///< discrete circular state buffer for internal signal history
} discrete_IIR_filter_DF_II_T;

return_code discrete_IIR_filter_DF_II_init(discrete_IIR_filter_DF_II_T*, size_t, real_t*, real_t*, real_t*);
return_code discrete_IIR_filter_DF_II_states_set(discrete_IIR_filter_DF_II_T*,const real_t*);
return_code discrete_IIR_filter_DF_II_states_set_all(discrete_IIR_filter_DF_II_T*, real_t);
return_code discrete_IIR_filter_DF_II_coeffs_set(discrete_IIR_filter_DF_II_T*,const real_t*,const real_t*);

/**
 * Discrete finite impulse response filter structure.
 * Structure contains state buffer for input signal only.
 * Filter is always stable.
 */
typedef struct discrete_FIR_filter {
	discrete_SISO_system_generic_T;  ///<inherited generic discrete SISO system structure
	state_buffer_T states; ///< discrete circular state buffer for input signal history
	vector_generic_T coeffs; ///< filter coefficients vector (ascending power of z)
} discrete_FIR_filter_T;

return_code discrete_FIR_filter_init(discrete_FIR_filter_T*, size_t, real_t*, real_t*);
return_code discrete_FIR_filter_states_set(discrete_FIR_filter_T*,const real_t*);
return_code discrete_FIR_filter_states_set_all(discrete_FIR_filter_T*,real_t);
return_code discrete_FIR_filter_coeffs_set(discrete_FIR_filter_T*,const real_t*);
return_code discrete_FIR_filter_coeffs_set_from_impulse_response(discrete_FIR_filter_T*,discrete_system_generic_T*);

/**
 * Second order IIR filter section coefficients structure.
 */
typedef struct discrete_biquad_section_coeffs {
	real_t b_coeffs[3]; ///<numerator coefficients (ascending power of z)
	real_t a_coeffs[3]; ///<denominator coefficients (ascending power of z)
} discrete_biquad_section_coeffs_T;

/**
 * Second order IIR filter section states structure for direct form I.
 */
typedef struct discrete_biquad_section_states_DF_I {
	real_t input_states[2]; ///<input signal history states buffer data
	real_t output_states[2]; ///<output signal history states buffer data
} discrete_biquad_section_states_DF_I_T;

/**
 * Second order IIR filter section states structure for direct form II.
 */
typedef struct discrete_biquad_section_states_DF_II {
	real_t v_states[2];  ///<internal signal (v) history states buffer data
} discrete_biquad_section_states_DF_II_T;

/**
 * Generic discrete series of biquadratic filter.
 * Second order series (SOS) filter.
 * Serial connection of second order IIR filters.
 */
typedef struct discrete_biquad_SOS_filter {
	discrete_SISO_system_generic_T;  ///< inherited generic discrete SISO system structure
	vector_generic_T states_vector; ///< vector of filter stages states - common for direct form I and II (@ref discrete_biquad_section_states_DF_I_T and @ref discrete_biquad_section_states_DF_II_T
	vector_generic_T coeffs_vector; ///< vector of filter stages coefficients - common for direct form I and II (@ref discrete_biquad_section_coeffs_T)
} discrete_biquad_SOS_filter_T;

typedef discrete_biquad_SOS_filter_T discrete_biquad_SOS_filter_DF_I_T; ///< Direct form I of SOS filter defined as generic SOS filter

return_code discrete_biquad_SOS_filter_DF_I_init(discrete_biquad_SOS_filter_DF_I_T*, size_t, discrete_biquad_section_coeffs_T*, discrete_biquad_section_states_DF_I_T*);
return_code discrete_biquad_SOS_filter_DF_I_coeffs_set(discrete_biquad_SOS_filter_DF_I_T*,const discrete_biquad_section_coeffs_T*);
return_code discrete_biquad_SOS_filter_DF_I_states_set(discrete_biquad_SOS_filter_DF_I_T*,const discrete_biquad_section_states_DF_I_T*);
return_code discrete_biquad_SOS_filter_DF_I_states_set_all(discrete_biquad_SOS_filter_DF_I_T*, discrete_biquad_section_states_DF_I_T);

typedef discrete_biquad_SOS_filter_T discrete_biquad_SOS_filter_DF_II_T;  ///< Direct form II of SOS filter defined as generic SOS filter

return_code discrete_biquad_SOS_filter_DF_II_init(discrete_biquad_SOS_filter_DF_II_T*, size_t, discrete_biquad_section_coeffs_T*, discrete_biquad_section_states_DF_II_T*);
return_code discrete_biquad_SOS_filter_DF_II_coeffs_set(discrete_biquad_SOS_filter_DF_II_T*,const discrete_biquad_section_coeffs_T*);
return_code discrete_biquad_SOS_filter_DF_II_states_set(discrete_biquad_SOS_filter_DF_II_T*,const discrete_biquad_section_states_DF_II_T*);
return_code discrete_biquad_SOS_filter_DF_II_states_set_all(discrete_biquad_SOS_filter_DF_II_T*, discrete_biquad_section_states_DF_II_T);

/**
 * Discrete delay structure.
 * Delays input signal for n samples.
 * Simple FIR filter without coefficients.
 *  y(k)=u(k-n)
 */
typedef struct discrete_delay {
	discrete_SISO_system_generic_T;  ///<inherited generic discrete SISO system structure
	state_buffer_T states; ///<discrete circular state buffer for input signal history
} discrete_delay_T;

return_code discrete_delay_init(discrete_delay_T*, uint, real_t*);

/**
 * Static data for first order discrete transfer function.
 * Contains data for input/output history states (2) and numerator/denominator coefficients (2)
 */
typedef struct discrete_transfer_function_first_order_data {
	real_t input_states_data[2]; ///<data for input signal history
	real_t output_states_data[2];
	real_t numerator_coeffs_data[2];
	real_t denominator_coeffs_data[2];
} discrete_transfer_function_first_order_data_T;

/**
 * Discrete sumator (accumulator) structure.
 * y(k)=y(k-1)+u(k)
 */
typedef struct discrete_sumator {
	discrete_transfer_function_T;  ///<inherited generic discrete transfer function structure
	discrete_transfer_function_first_order_data_T; ///<static data (coefficients and states) for first order transfer function
} discrete_sumator_T;

return_code discrete_sumator_init(discrete_sumator_T*, real_t);

/**
 * Discrete difference structure.
 * y(k)=u(k)+u(k-1)
 */
typedef struct discrete_difference {

	discrete_transfer_function_T;	///<inherited generic discrete transfer function structure
	discrete_transfer_function_first_order_data_T;  ///<static data (coefficients and states) for first order transfer function
} discrete_difference_T;

return_code discrete_difference_init(discrete_difference_T*, real_t);

/**
 * Discrete proportional-sumation-difference (PSD) regulator structure.
 * Contains sumator and difference structures.
 * Gains can be modified during runtime.
 * Supports output saturation and anti-windup.
 */
typedef struct discrete_PSD_regulator {
	discrete_SISO_system_generic_T; ///<inherited generic discrete SISO system structure
	discrete_sumator_T sumator;  ///< discrete sumator structure
	discrete_difference_T difference; ///<discrete difference structure
	real_t P_gain;  ///< proportional gain - modifiable during runtime
	real_t S_gain;	///< sumator gain - modifiable during runtime
	real_t D_gain;	///< difference gain - modifiable during runtime
#ifdef USE_CLIP
	real_t saturation_high; ///<output saturation high limit, default set to inf
	real_t saturation_low; ///<output saturation low limit, default set to -inf
	real_t AW_gain;		///< anti-windup gain
	real_t u_cut;		///<state variable for anti-windup - output cut off
#endif
} discrete_PSD_regulator_T;

return_code discrete_PSD_regulator_init(discrete_PSD_regulator_T*, real_t, real_t, real_t);
#ifdef USE_CLIP
return_code discrete_PSD_regulator_set_saturation(discrete_PSD_regulator_T*, real_t, real_t, real_t);
#endif

typedef discrete_sumator_T discrete_integrator_T; ///<Discrete time integration using numerical integration approximations
return_code discrete_integrator_init(discrete_integrator_T*, real_t, real_t, approximation_method);

typedef discrete_difference_T discrete_derivative_T; ///< Discrete time derivative using numerical derivative approximations
return_code discrete_derivative_init(discrete_derivative_T*, real_t, real_t, real_t, approximation_method);

/**
 * Discrete proportional-integration-derivative (PID) regulator structure.
 * Contains discrete integrator and derivative structures.
 * Uses numerical derivative and integration approximations.
 * Gains can be modified during runtime.
 * Supports output saturation and anti-windup.
 */
typedef struct discrete_PID_regulator {
	discrete_SISO_system_generic_T; ///<inherited generic discrete SISO system structure
	PID_regulator_params_T;///<proportional,integrator and derivative gain
	discrete_integrator_T integrator; ///< discrete integrator structure
	discrete_derivative_T derivator; ///<discrete derivative structure
#ifdef USE_CLIP
	real_t saturation_high; ///<output saturation high limit, default set to inf
	real_t saturation_low; ///<output saturation low limit, default set to -inf
	real_t AW_gain; ///< anti-windup gain
	real_t u_cut;
#endif
} discrete_PID_regulator_T;

return_code discrete_PID_regulator_init(discrete_PID_regulator_T*, real_t, real_t, real_t, real_t, real_t, approximation_method);

#ifdef USE_CLIP
return_code discrete_PID_regulator_set_saturation(discrete_PID_regulator_T*, real_t, real_t, real_t);
#endif

/**
 * Discrete integration-proportional (IP) regulator structure.
 * Generic MISO system.
 * Contains discrete integrator structure.
 * Uses numerical integration approximation.
 * Integrator input is regulation error signal (e), while proportional gain input is feedback signal (y).
 * Gains can be modified during runtime.
 * Supports output saturation and anti-windup.
 */
typedef struct discrete_IP_regulator {
	discrete_MIMO_system_generic_T; ///<inherited generic discrete SISO system structure
	PID_regulator_params_T;///<proportional and integrator gain
	discrete_integrator_T integrator;  ///< discrete integrator structure
	real_t input[2];
	real_t output[1];
#ifdef USE_CLIP
	real_t saturation_high; ///< output saturation high limit, default set to inf
	real_t saturation_low; ///< output saturation low limit, default set to -inf
	real_t AW_gain; ///< anti-windup gain
	real_t u_cut; ///<state variable for anti-windup - output cut off
#endif
} discrete_IP_regulator_T;

return_code discrete_IP_regulator_init(discrete_IP_regulator_T*, real_t, real_t, real_t, approximation_method);
#ifdef USE_CLIP
return_code discrete_IP_regulator_set_saturation(discrete_IP_regulator_T*, real_t, real_t, real_t);
#endif

/**
 * Discrete state space structure.
 * Contains new x(k+1) and last x(k) states vector
 * Contains state space coefficients matrix A and vectors B and C
 * Uses standard matrix multiplication and addition:
 * - x(k+1)=Ax(k)+Bu(k)
 * - y(k)=Cx(k)
 * GSL/BLAS support.
 * @note (n) is order of system.
 *
 */
typedef struct discrete_state_space {
	discrete_SISO_system_generic_T; ///<inherited generic discrete SISO system structure
	state_space_T;  ///<inherited generic state space structure
	vector_generic_T Xk_1; ///< vector of new states x(k+1)
	vector_generic_T Xk_0; ///< vector of last states x(k)
} discrete_state_space_T;

return_code discrete_state_space_init(discrete_state_space_T*, size_t, real_t*, real_t*, real_t*, real_t*, real_t*);
return_code discrete_state_space_states_set(discrete_state_space_T*,const real_t*);
return_code discrete_state_space_states_set_all(discrete_state_space_T*, real_t);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* DYNAMIC_SYSTEMS_H_ */
