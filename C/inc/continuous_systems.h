/**
 * @file continuous_systems.h
 */

#ifndef INC_CONTINUOUS_SYSTEMS_H_
#define INC_CONTINUOUS_SYSTEMS_H_

#include "systems.h"
#include "ode_solver.h"

/**
 * @addtogroup continuous_systems Continuous systems
 * @brief Continuous dynamic systems interface and implementations of basic systems types.
 *
 * Generic continuous systems interface and interface with ODE solver - system states derivatives and system output update callback functions.
 * # Implementation of continuous dynamic systems:
 * 	- State space
 * 	- Transfer function
 * 	- Integrator
 * 	- PID regulator
 * 	- SISO nonlinear systems
 * @{
 */

typedef ODE_solver_system_derivatives_fcn continuous_system_derivatives_fcn; ///<type definition from ODE solver's system states derivative function prototype
typedef ODE_solver_system_update_output_fcn continuous_system_update_output_fcn; ///<type definition from ODE solver's system update output function prototype

/**
 * Generic continuous system structure.
 * Contains:
 * 	- system interface
 * 	- states and states derivatives vector
 * 	- callback functions
 */
typedef struct continuous_system_generic {
	system_generic_T interface; ///< generic system interface
	vector_generic_T X; ///< system states values vector
	vector_generic_T dX; ///< system states derivatives vector
	continuous_system_derivatives_fcn derrivatives_fcn; ///< system states derivatives function pointer
	continuous_system_update_output_fcn update_output_fcn; ///< system output update function pointer
} continuous_system_generic_T;


/**
 * Generic SISO continuous system structure.
 * Inherited generic continuous system.
 * Adds variables for single input and single output signal.
 */
typedef struct continuous_SISO_system_generic_T {
	continuous_system_generic_T; ///< generic continuous system interface
	real_t input; ///< system input signal variable
	real_t output; ///< system output signal variable
} continuous_SISO_system_generic_T;

/**
 * Generic MIMO continuous system structure.
 * Inherited generic continuous system.
 */
typedef struct continuous_MIMO_system_generic {
	continuous_system_generic_T; ///< generic continuous system interface
} continuous_MIMO_system_generic_T;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns system states vector pointer.
 * Wrapper for access to continuous system states vector.
 * @param system continuous system structure pointer
 * @return pointer to states vector
 */
static inline vector_generic_T* continuous_system_generic_get_states(continuous_system_generic_T* system) {
	return &system->X;
}

/**
 * Loads data to system states vector.
 * Wrapper for setting states of continuous system.
 *
 * @param system continuous system structure pointer
 * @param X pointer to system states data
 * @return system control library error code
 */
static inline return_code continuous_system_generic_states_set(continuous_system_generic_T* system,const real_t* X) {
	return vector_generic_load(&system->X, X);
}
/**
 * Returns states derivatives of continuous system according to updated states of the system.
 * Wrapper for continuous system states derivatives callback function.
 * @param system continuous system structure pointer
 * @param time step time
 * @param step_type step type
 * @return system states derivatives vector
 */
static inline const vector_generic_T* continuous_system_generic_get_derivatives(continuous_system_generic_T* system, real_t time, ODE_solver_step_t step_type) {
	return system->derrivatives_fcn(system, time, step_type);
}

/**
 * Updates output signal of continuous system according to updated states of the system.
 * Wrapper for continuous system update output callback function.
 * @param system continuous system structure pointer
 * @param time step time
 * @param step_type step type
 * @return system control library error code
 */
static inline return_code continuous_system_generic_update_output(continuous_system_generic_T* system, real_t time, ODE_solver_step_t step_type) {
	return system->update_output_fcn(system, time, step_type);
}
/**
 * Returns order of continuous dynamic system (number of states).
 * @param system continuous system structure pointer
 * @return order of system
 */
static inline size_t continuous_system_generic_get_order(continuous_system_generic_T* system) {
	return vector_length(&system->X);
}

/**
 * Continuous integrator structure.
 * Numerical integrating of input signal.
 * Contains static data for state value and state derivative.
 */
typedef struct continuous_integrator {
	continuous_SISO_system_generic_T;
	real_t dX_data[1]; ///<data for states derivatives
	real_t X_data[1];	///<data for states values
} continuous_integrator_T;
return_code continuous_integrator_init(continuous_integrator_T*);


/**
 * Continuous state space system representation structure.
 * Contains state space coefficients matrix A and vectors B,C.
 * Uses standard matrix multiplication and addition:
 *  - dx/dt=Ax(t)+Bu(t)
 *  - y(t)=Cx(t)
 * GSL/BLAS support.
 * @note (n) is order of system
 */
typedef struct continuous_state_space {
	continuous_SISO_system_generic_T;
	state_space_T; ///<inherited generic state space structure
} continuous_state_space_T;

return_code continuous_state_space_init(continuous_state_space_T*, size_t, real_t*, real_t*, real_t*, real_t*, real_t*);

/**
 * Continuous transfer function system representation structure.
 * Contains numerator and denominator coefficients vectors.
 * States are in canonical form (derivatives degrees).
 */
typedef struct continuous_transfer_function {
	continuous_SISO_system_generic_T;
	transfer_function_T; ///<numerator/denominator coefficients vector (ascending power of s)
} continuous_transfer_function_T;

return_code continuous_transfer_function_init(continuous_transfer_function_T*, size_t, size_t, real_t*, real_t*, real_t*, real_t*);

/**
 * Continuous first order transfer function representation structure.
 * Contains generic transfer function structure.
 * Contains additional static data for:
 * 	- numerator and denominator coefficients
 * 	- states values and states derivatives
 *
 */
typedef struct continuous_transfer_function_first_order {
	continuous_transfer_function_T;
	real_t numerator_data[1]; ///<data for numerator coefficients
	real_t denominator_data[2]; ///<data for denominator coefficients
	real_t dX_data[1];	///<data for states derivatives
	real_t X_data[1]; ///<data for states values
} continuous_transfer_function_first_order_T;

return_code continuous_transfer_function_first_order_init(continuous_transfer_function_first_order_T*, real_t, real_t);

/**
 * Continuous second order transfer function representation structure.
 * Contains generic transfer function structure.
 * Contains additional static data for:
 * 	- numerator and denominator coefficients
 * 	- states values and states derivatives
 *
 */
typedef struct continuous_transfer_function_second_order {
	continuous_transfer_function_T;
	real_t numerator_data[1]; ///<data for numerator coefficients
	real_t denominator_data[3];  ///<data for denominator coefficients
	real_t dX_data[2]; ///<data for states derivatives
	real_t X_data[2];  ///<data for states values
} continuous_transfer_function_second_order_T;

return_code continuous_transfer_function_second_order_aperiodic_init(continuous_transfer_function_second_order_T*, real_t, real_t, real_t);
return_code continuous_transfer_function_second_order_periodic_init(continuous_transfer_function_second_order_T*, real_t, real_t, real_t);


/**
 * Continuous PID controller structure.
 * Proportional - integration - derivative continuous controller.
 * Uses filtered derivative.
 * Gains are runtime modifiable.
 * Contains additional static data for states values and states derivatives.
 */
typedef struct continuous_PID_regulator {
	continuous_SISO_system_generic_T;
	PID_regulator_params_T;///<proportional,integrator and derivative gain
	real_t N_gain;		///<derivative filter gain
	real_t dX_data[2];  ///<data for states derivatives
	real_t X_data[2];   ///<data for states values
} continuous_PID_regulator_T;

return_code continuous_PID_regulator_init(continuous_PID_regulator_T*, real_t, real_t, real_t, real_t);


/**
 * Custom continuous system states derivatives function prototype.
 * @param - system states derivatives array
 * @param - system states array
 * @param - system parameters array
 * @param - system input value
 * @param - time
 * @return system control library error code
 */
typedef return_code (*continuous_custom_system_derrivatives_fcn_T)(real_t[],const real_t[],const real_t[], real_t, real_t);


/**
 * Custom continuous system output function prototype.
 * @param - system states array
 * @param - system parameters array
 * @param - system input value
 * @param - system output value pointer
 * @return system control library error code
 */
typedef return_code (*continuous_custom_system_output_fcn_T)(const real_t[],const real_t[], real_t, real_t*);


/**
 * Continuous linear/nonlinear custom system representation structure.
 * Interface for generic linear/nonlinear continuous systems.
 * Simplified states derivatives and output update callback interface.
 * Allows system parameters propagation.
 */
typedef struct continuous_custom_SISO_system {
	continuous_SISO_system_generic_T;
	vector_generic_T params; ///< vector of system parameters
	continuous_custom_system_derrivatives_fcn_T custom_derrivatives_fcn;  ///<simplified states derivatives callback function
	continuous_custom_system_output_fcn_T custom_update_output_fcn; ///<simplified output update callback function
} continuous_custom_SISO_system_T;

return_code continuous_custom_SISO_system_init(continuous_custom_SISO_system_T*, continuous_custom_system_derrivatives_fcn_T, continuous_custom_system_output_fcn_T, size_t, size_t, real_t*);
return_code continuous_custom_SISO_system_params_set(continuous_custom_SISO_system_T*,const real_t*);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_CONTINUOUS_SYSTEMS_H_ */
