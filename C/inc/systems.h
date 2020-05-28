
/**
 * @file systems.h
 */

#ifndef SYSTEMS_H_
#define SYSTEMS_H_

#include "common_def.h"
#include "adt.h"
#include "vector.h"
#include "matrix.h"
#include "complex.h"

/**
 * @defgroup systems Systems
 */

/** @addtogroup systems_interface Systems Interface
 *  @ingroup systems
 *  @brief Generic systems abstraction and interface.
 *
 * # Implemented functionality
 * 	- signals abstraction
 * 	- generic system initialization and de-initialization
 * 	- common system input/output interface reading/writing ports of system
 * 	- creating connections (series,parallel,feedback)
 *
 * @{
 */

struct system_generic;

/**
 * System generic deinitialization function prototype.
 * @param  pointer to generic system structure
 * @return system control library error code
 */
typedef return_code (*system_deinit_fcn)(struct system_generic*);

/*!
 * Numeric signal abstraction structure.
 * Memory mapped signal abstraction structure
 * used in all all generic systems
 */
typedef struct signal_realtime {
	real_t* ptr; ///< signal value memory adress
	struct system_generic* owner; ///< pointer to owner system
} signal_realtime_T;

/*!
 * Generic system type/classification enum.
 * Classifies major system types e.g. dynamic, static and subsystems
 */
typedef enum {
	system_type_uninitialized, system_type_static_system = 1, system_type_dynamic_system, system_type_signal_source, system_type_signal_sink, system_type_subsystem
} system_type;

/**
 * Generic system time domain enum.
 * Classifies dynamic system time domain - discrete or continuous time
 */
typedef enum {
	system_time_domain_uninitialized, system_time_domain_discrete = 1, system_time_domain_continuous
} system_time_domain;

/**
 * Generic system abstraction structure and interface.
 * Contains informations about system type,time domain
 * Implements system input and output interface
 */
typedef struct system_generic {
	vector_generic_T input_port; ///<input port signals vector of type @ref signal_realtime_T
	vector_generic_T output_port; ///<output port signals vector of type @ref signal_realtime_T
	system_type type;  ///<type of system
	system_time_domain time_domain; ///< time domain of system (discrete/continuous)
	system_deinit_fcn deinit_fcn; ///<deinitialisation function pointer
	list_t execution_list; ///<linked list of subsystems
} system_generic_T;

#define system_input_port_struct(system,idx) (vector_type_at(&((system)->input_port),signal_realtime_T,idx))///< gets system input port realtime signal structure indexed by idx
#define system_output_port_struct(system,idx) (vector_type_at(&((system)->output_port),signal_realtime_T,idx)) ///< gets system output port realtime signal structure indexed by idx

#define system_input_port_real_ptr(system,idx) (system_input_port_struct((system),idx).ptr) ///< gets system input port signals real pointer indexed by idx
#define system_output_port_real_ptr(system,idx) (system_output_port_struct((system),idx).ptr) ///< gets system output port signals real pointer indexed by idx

#define system_input_port_real_value(system,idx) (*(system_input_port_real_ptr((system),idx)))  ///< gets system input port signals real value indexed by idx
#define system_output_port_real_value(system,idx) (*(system_output_port_real_ptr((system),idx))) ///< gets system output port signals real value indexed by idx

#define system_input_port_vector_ptr(system) (&((system)->input_port))  ///< gets system input port realtime signal structure array pointer
#define system_output_port_vector_ptr(system) (&((system)->output_port)) ///< gets system output port realtime signal structure array pointer

#define system_input_port_size(system)  vector_length(system_input_port_vector_ptr(system)) ///< gets system input port width/number of input signals
#define system_output_port_size(system) vector_length(system_output_port_vector_ptr(system))///< gets system output port width/number of output signals

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Generic transfer function system representation structure.
 * Contains numerator and denominator coefficients vectors
 */
typedef struct transfer_function {
	vector_generic_T numerator_coeffs; ///<numerator coefficients vector
	vector_generic_T denominator_coeffs; ///<denominator coefficients vector
} transfer_function_T;

return_code transfer_function_init(transfer_function_T*, size_t, size_t, real_t*, real_t*);
return_code transfer_function_deinit(transfer_function_T*);
return_code transfer_function_assert(const transfer_function_T*);
return_code transfer_function_coeffs_set(transfer_function_T*, const real_t*, const real_t*);

/**
 * Generic state space system representation structure.
 * Contains A,B,C matrices.
 */
typedef struct state_space {
	matrix_T A;  ///< coefficients matrix A - nxn
	vector_generic_T B;  ///< coefficients vector B
	vector_generic_T C;  ///< coefficients vector C
} state_space_T;

return_code state_space_init(state_space_T*, size_t, real_t*, real_t*, real_t*);
return_code state_space_deinit(state_space_T*);
return_code state_space_coeffs_set(state_space_T*, const real_t*, const real_t*, const real_t*);
return_code state_space_assert(const state_space_T*);

/**
 * Generic zeros-poles-gain system representation structure.
 * Contains zeros,poles complex vectors and gain.
 */
typedef struct zero_pole_gain {
	vector_generic_T zeros; ///<zeros complex vector
	vector_generic_T poles; ///<poles complex vector
	real_t gain; ///< system gain

} zeros_poles_gain_T;
return_code zeros_poles_gain_init(zeros_poles_gain_T*, size_t, complex_t*, size_t, complex_t*, real_t);
return_code zeros_poles_gain_deinit(zeros_poles_gain_T*);
return_code zeros_poles_gain_assert(const zeros_poles_gain_T*);

return_code system_interface_init(system_generic_T*, system_type, system_time_domain, size_t, real_t*, size_t, real_t*, system_deinit_fcn);
return_code system_interface_deinit(system_generic_T*);
return_code systems_connect_serial(system_generic_T*[], size_t, uint[], uint[], system_generic_T*);
return_code systems_connect_paralel(system_generic_T*[], size_t, system_generic_T*, system_generic_T*);
return_code systems_connect_feedback(system_generic_T*, system_generic_T*, system_generic_T*, system_generic_T*);

return_code system_deinit(system_generic_T*);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SYSTEMS_H_ */
