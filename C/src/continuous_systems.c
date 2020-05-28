#include "continuous_systems.h"
#include "gsl_wrapper.h"
#include "signal_process.h"

return_code continuous_system_generic_init(continuous_system_generic_T*, size_t, real_t*, real_t*, continuous_system_derivatives_fcn, continuous_system_update_output_fcn);
return_code continuous_system_generic_deinit(continuous_system_generic_T*);
return_code continuous_SISO_system_generic_deinit(continuous_SISO_system_generic_T*);
return_code continuous_MIMO_system_generic_deinit(continuous_MIMO_system_generic_T*);
return_code continuous_SISO_system_generic_init(continuous_SISO_system_generic_T*, size_t, real_t*, real_t*, continuous_system_derivatives_fcn, continuous_system_update_output_fcn, system_deinit_fcn);
return_code continuous_MIMO_system_generic_init(continuous_MIMO_system_generic_T*, size_t, real_t*, real_t*, continuous_system_derivatives_fcn, continuous_system_update_output_fcn, size_t, real_t*, size_t, real_t*, system_deinit_fcn);

/**
 * Generic continuous system initialization.
 * Initializes states and states derivatives vector with appropriate dimensions.
 * Registers states derivatives and output update callback function pointers.
 *
 * @param system continuous system structure pointer
 * @param order system order - number of continuous states
 * @param dX system states derivatives vector data pointer
 * @param X system states vector data pointer
 * @param derivatives_get_fcn system states derivatives function pointer
 * @param update_output_fcn system output update function pointer
 * @return system control library error code
 */
return_code continuous_system_generic_init(continuous_system_generic_T* system, size_t order, real_t* dX, real_t* X, continuous_system_derivatives_fcn derivatives_get_fcn, continuous_system_update_output_fcn update_output_fcn) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	if (derivatives_get_fcn == NULL)
		return return_NULLPTR;
	if (update_output_fcn == NULL)
		return return_NULLPTR;
#endif

	returnval = vector_type_init(&system->X, real_t, order, X);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_init(&system->dX, real_t, order, dX);
	if (returnval != return_OK)
		return returnval;
	vector_type_set_all(&system->X, real_t, 0);
	vector_type_set_all(&system->dX, real_t, 0);

	system->derrivatives_fcn = derivatives_get_fcn;
	system->update_output_fcn = update_output_fcn;

	return return_OK;
}
/**
 * Continuous system structure deinitialization.
 * Deinitializes states vector and states derivatives vector.
 * Resets states derivatives and output update callback function pointers.
 *
 * @param system continuous system structure pointer
 * @return system control library error code
 */
return_code continuous_system_generic_deinit(continuous_system_generic_T* system) {
	system->interface.type = system_type_uninitialized;
	system->interface.time_domain = system_time_domain_uninitialized;
	system->derrivatives_fcn = NULL;
	system->update_output_fcn = NULL;
	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&system->X);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&system->dX);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}
/**
 * Continuous SISO system structure deinitialization.
 * Resets input and output signals values.
 * Calls generic continuous system deinitialization function.
 *
 * @param system continuous system structure pointer
 * @return system control library error code
 */
return_code continuous_SISO_system_generic_deinit(continuous_SISO_system_generic_T* system) {

	system->input = 0;
	system->output = 0;
	return continuous_system_generic_deinit((continuous_system_generic_T*) system);

}
/**
 * Generic continuous SISO system initialization.
 * Initializes system interface - input and output ports.
 * Calls generic continuous system initialization.
 * Registers deinit function.
 * @see continuous_system_generic_init
 * @see system_interface_atomic_init
 * @param system continuous system structure pointer
 * @param order system order - number of continuous states
 * @param dX system states derivatives data pointer
 * @param X system states data pointer
 * @param derivatives_get_fcn system states derivatives function pointer
 * @param update_output_fcn system output update function pointer
 * @param deinit_fcn
 * @return system control library error code
 */
return_code continuous_SISO_system_generic_init(continuous_SISO_system_generic_T* system, size_t order, real_t* dX, real_t* X, continuous_system_derivatives_fcn derivatives_get_fcn, continuous_system_update_output_fcn update_output_fcn, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;

	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) continuous_SISO_system_generic_deinit;

	returnval = system_interface_init(&system->interface, system_type_dynamic_system, system_time_domain_continuous, 1, &system->input, 1, &system->output, deinit_fcn);
	if (returnval != return_OK)
		return returnval;
	system->input = 0;
	system->output = 0;

	return continuous_system_generic_init((continuous_system_generic_T*) system, order, dX, X, derivatives_get_fcn, update_output_fcn);
}
/**
 * Continuous MIMO system structure deinitialization.
 * Calls generic continuous system deinitialization function
 *
 * @param system continuous system structure pointer
 * @return system control library error code
 */
return_code continuous_MIMO_system_generic_deinit(continuous_MIMO_system_generic_T* system) {

	return continuous_system_generic_deinit((continuous_system_generic_T*) system);
}

/**
 * Generic continuous MIMO system initialization.
 * Initializes system interface - input and output ports.
 * Calls generic continuous system initialization.
 * Registers deinit function.
 * @see continuous_system_generic_init
 * @see system_interface_atomic_init
 *
 * @param system continuous system structure pointer
 * @param order system order - number of continuous states
 * @param dX system states derivatives data pointer
 * @param X system states data pointer
 * @param derivatives_get_fcn system states derivatives function pointer
 * @param update_output_fcn system output update function pointer
 * @param input_port_width number of input port signals - port width
 * @param input_port_ptr pointer to input port signals data
 * @param output_port_width number of output port signals - port width
 * @param output_port_ptr pointer to output port signals data
 * @param deinit_fcn system deinitialization function pointer
 * @return system control library error code
 */

return_code continuous_MIMO_system_generic_init(continuous_MIMO_system_generic_T* system, size_t order, real_t* dX, real_t* X, continuous_system_derivatives_fcn derivatives_get_fcn, continuous_system_update_output_fcn update_output_fcn, size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;

	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) continuous_MIMO_system_generic_deinit;

	returnval = system_interface_init(&system->interface, system_type_dynamic_system, system_time_domain_continuous, input_port_width, input_port_ptr, output_port_width, output_port_ptr, deinit_fcn);
	if (returnval != return_OK)
		return returnval;

	memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);

	return continuous_system_generic_init((continuous_system_generic_T*) system, order, dX, X, derivatives_get_fcn, update_output_fcn);
}

static return_code continuous_integrator_update_output(continuous_integrator_T* integrator, real_t time, ODE_solver_step_t step_type) {
	(void) time;
	(void) step_type;
	system_output_port_real_value(&integrator->interface, 0) = vector_type_at(&integrator->X, real_t, 0);
	return return_OK;
}

static const vector_generic_T* continuous_integrator_get_derivatives(continuous_integrator_T* integrator, real_t time, ODE_solver_step_t step_type) {

	(void) time;
	(void) step_type;
	vector_type_at(&integrator->dX,real_t,0) = system_input_port_real_value(&integrator->interface, 0);
	return &integrator->dX;
}

return_code continuous_integrator_init(continuous_integrator_T* integrator) {
	return continuous_SISO_system_generic_init((continuous_SISO_system_generic_T*) integrator, 1, integrator->dX_data, integrator->X_data, (continuous_system_derivatives_fcn) continuous_integrator_get_derivatives, (continuous_system_update_output_fcn) continuous_integrator_update_output, (system_deinit_fcn) NULL);
}

static const vector_generic_T* continuous_state_space_get_derivatives(continuous_state_space_T*, real_t, ODE_solver_step_t);
static return_code continuous_state_space_update_output(continuous_state_space_T*, real_t, ODE_solver_step_t);
static return_code continuous_state_space_deinit(continuous_state_space_T*);

/**
 * Continuous state space initialization.
 * Initializes generic continuous system interface.
 * Initializes coefficients matrix A, and vectors B,C according to system order.
 * @see continuous_SISO_system_generic_init
 *
 * @param state_space continuous state space structure pointer
 * @param order system order - number of continuous states
 * @param A matrix A data pointer, if NULL - function performs dynamic allocation
 * @param B vector B data pointer, if NULL - function performs dynamic allocation
 * @param C vector C data pointer, if NULL - function performs dynamic allocation
 * @param dX system states derivatives data pointer, if NULL - function performs dynamic allocation
 * @param X system states values data pointer, if NULL - function performs dynamic allocation
 * @return system control library error code
 */

return_code continuous_state_space_init(continuous_state_space_T* state_space, size_t order, real_t* A, real_t* B, real_t* C, real_t* dX, real_t* X) {
	return_code returnval = return_OK;
	returnval = state_space_init((state_space_T*) &state_space->A, order, A, B, C);
	if (returnval != return_OK)
		return returnval;

	return continuous_SISO_system_generic_init((continuous_SISO_system_generic_T*) state_space, order, dX, X, (continuous_system_derivatives_fcn) continuous_state_space_get_derivatives, (continuous_system_update_output_fcn) continuous_state_space_update_output, (system_deinit_fcn) continuous_state_space_deinit);
}
static return_code continuous_state_space_deinit(continuous_state_space_T* state_space) {
	return_code returnval = return_OK;
	returnval = state_space_deinit((state_space_T*) &state_space->A);
	if (returnval != return_OK)
		return returnval;
	return continuous_SISO_system_generic_deinit((continuous_SISO_system_generic_T*) state_space);
}

static const vector_generic_T* continuous_state_space_get_derivatives(continuous_state_space_T* state_space, real_t time, ODE_solver_step_t step_type) {
	(void) time;
	real_t u = system_input_port_real_value(&state_space->interface, 0);
#ifdef USE_GSL
	const gsl_matrix A_gsl = matrix_to_gsl_matrix(&state_space->A);
	const gsl_vector B_gsl = vector_real_to_gsl_vector(&state_space->B);
	const gsl_vector X_gsl = vector_real_to_gsl_vector(&state_space->X);
	gsl_vector dX_gsl = vector_real_to_gsl_vector(&state_space->dX);
	gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1.0, &A_gsl, &X_gsl, 0.0, &dX_gsl));
	gsl_error_code_to_return_code(gsl_blas_daxpy(u, &B_gsl, &dX_gsl));
#else
	matrix_T X_mat=vector_real_to_col_matrix(&state_space->X);
	matrix_T dX_mat=vector_real_to_col_matrix(&state_space->dX);
	matrix_T B_mat=vector_real_to_col_matrix(&state_space->B);
	matrix_multiply(&state_space->A, &X_mat, &dX_mat);
	matrix_element_wise_operation_and_scalar_operation(&B_mat, &dX_mat, &dX_mat, u, real_sum, real_mul);

#endif
	return &state_space->dX;
}

static return_code continuous_state_space_update_output(continuous_state_space_T* state_space, real_t time, ODE_solver_step_t step_type) {

#ifdef USE_GSL
	const gsl_vector C_gsl = vector_real_to_gsl_vector(&state_space->C);
	const gsl_vector X_gsl = vector_real_to_gsl_vector(&state_space->X);
	return gsl_error_code_to_return_code(gsl_blas_ddot(&C_gsl, &X_gsl, system_output_port_real_ptr(&state_space->interface, 0)));
#else
	return_code returnval = return_OK;
	returnval = vector_real_dot_product(&state_space->C, &state_space->X, system_output_port_real_ptr(&state_space->interface, 0));
	if (returnval != return_OK)
	return returnval;
	return return_OK;
#endif

}

static const vector_generic_T* continuous_transfer_function_get_derivatives(continuous_transfer_function_T*, real_t, ODE_solver_step_t);
static return_code continuous_transfer_function_update_output(continuous_transfer_function_T*, real_t, ODE_solver_step_t);
static return_code continuous_transfer_function_deinit(continuous_transfer_function_T*);

/**
 * Continuous transfer function initialization.
 * Initializes generic continuous system interface.
 * Initializes numerator and denominator coefficients vectors according to order.
 * Denominator order must be greater or equal than numerator order.
 * @see continuous_SISO_system_generic_init
 * @param transfer_function continuous transfer function structure pointer
 * @param numerator_order order of numerator
 * @param denominator_order order of denominator
 * @param numerator_coeffs_ptr numerator coefficients data pointer (ascending power of s) , if NULL - function performs dynamic allocation
 * @param denominator_coeffs_ptr denominator coefficients data pointer (ascending power of s) , if NULL -function performs dynamic allocation
 * @param dX system states derivatives data pointer, if NULL - function performs dynamic allocation
 * @param X system states values data pointer, if NULL - function performs dynamic allocation
 * @return system control library error code
 */
return_code continuous_transfer_function_init(continuous_transfer_function_T* transfer_function, size_t numerator_order, size_t denominator_order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* dX, real_t* X) {

	return_code returnval = return_OK;
#ifdef ASSERT_DIMENSIONS
	if (numerator_order > denominator_order)
		return return_WRONG_DIMENSIONS;
#endif
	returnval = transfer_function_init((transfer_function_T*) &transfer_function->numerator_coeffs, numerator_order, denominator_order, numerator_coeffs_ptr, denominator_coeffs_ptr);
	if (returnval != return_OK)
		return returnval;

	return continuous_SISO_system_generic_init((continuous_SISO_system_generic_T*) transfer_function, denominator_order, dX, X, (continuous_system_derivatives_fcn) continuous_transfer_function_get_derivatives, (continuous_system_update_output_fcn) continuous_transfer_function_update_output, (system_deinit_fcn) continuous_transfer_function_deinit);

}

static return_code continuous_transfer_function_deinit(continuous_transfer_function_T* transfer_function) {

	return_code returnval = return_OK;
	returnval = transfer_function_deinit((transfer_function_T*) &transfer_function->numerator_coeffs);
	if (returnval != return_OK)
		return returnval;

	return continuous_SISO_system_generic_deinit((continuous_SISO_system_generic_T*) transfer_function);
}

static const vector_generic_T* continuous_transfer_function_get_derivatives(continuous_transfer_function_T* transfer_function, real_t time, ODE_solver_step_t step_type) {

	(void) time;
	(void) step_type;
	size_t system_order = vector_length(&transfer_function->X);

	vector_type_at(&transfer_function->dX,real_t,system_order-1) = 0;
	for (uint i = 0; i < system_order; i++) {
		if (i < system_order - 1) {
			vector_type_at(&transfer_function->dX,real_t,i) = vector_type_at(&transfer_function->X, real_t, i + 1);
		}
		vector_type_at(&transfer_function->dX,real_t,system_order-1) += vector_type_at(&transfer_function->X,real_t,i) * (-vector_type_at(&transfer_function->denominator_coeffs, real_t, i));
	}
	vector_type_at(&transfer_function->dX,real_t,system_order-1) += system_input_port_real_value(&transfer_function->interface, 0);
	vector_type_at(&transfer_function->dX,real_t,system_order-1) /= vector_type_at(&transfer_function->denominator_coeffs, real_t, system_order);

	return &transfer_function->dX;

}

static return_code continuous_transfer_function_update_output(continuous_transfer_function_T* transfer_function, real_t time, ODE_solver_step_t step_type) {
	(void) time;
	(void) step_type;
	real_t y = 0;
	size_t system_order = vector_length(&transfer_function->X);
	for (uint i = 0; i < vector_length(&transfer_function->numerator_coeffs); i++) {
		if (i < system_order)
			y += vector_type_at(&transfer_function->numerator_coeffs,real_t,i) * vector_type_at(&transfer_function->X, real_t, i);
		else if (i == system_order)
			y += vector_type_at(&transfer_function->numerator_coeffs,real_t,i) * vector_type_at(&transfer_function->dX, real_t, i - 1);
	}
	system_output_port_real_value(&transfer_function->interface, 0) = y;

	return return_OK;
}
/**
 * Initializes statically allocated first order continuous transfer function.
 * @param transfer_function continuous transfer function structure pointer
 * @param T system time constant
 * @param K system gain
 * @return system control library error code
 */
return_code continuous_transfer_function_first_order_init(continuous_transfer_function_first_order_T* transfer_function, real_t T, real_t K) {
	return_code returnval = return_OK;
	returnval = continuous_transfer_function_init((continuous_transfer_function_T*) transfer_function, 0, 1, transfer_function->numerator_data, transfer_function->denominator_data, transfer_function->dX_data, transfer_function->X_data);
	if (returnval != return_OK)
		return returnval;
	transfer_function->numerator_data[0] = K;
	transfer_function->denominator_data[0] = 1;
	transfer_function->denominator_data[1] = T;

	return return_OK;
}
/**
 * Initializes statically allocated second order aperiodic transfer function.
 * @param transfer_function continuous transfer function structure pointer
 * @param T_1 system time constant 1
 * @param T_2 system time constant 2
 * @param K system gain
 * @return system control library error code
 */
return_code continuous_transfer_function_second_order_aperiodic_init(continuous_transfer_function_second_order_T* transfer_function, real_t T_1, real_t T_2, real_t K) {
	return_code returnval = return_OK;
	returnval = continuous_transfer_function_init((continuous_transfer_function_T*) transfer_function, 0, 2, transfer_function->numerator_data, transfer_function->denominator_data, transfer_function->dX_data, transfer_function->X_data);
	if (returnval != return_OK)
		return returnval;
	transfer_function->numerator_data[0] = K;
	transfer_function->denominator_data[0] = 1;
	transfer_function->denominator_data[1] = T_1 + T_2;
	transfer_function->denominator_data[2] = T_1 * T_2;
	return return_OK;
}

/**
 * Initializes statically allocated second order periodic transfer function.
 * @param transfer_function continuous transfer function structure pointer
 * @param omega_0 system frequency
 * @param B system damping
 * @param K system gain
 * @return system control library error code
 */
return_code continuous_transfer_function_second_order_periodic_init(continuous_transfer_function_second_order_T* transfer_function, real_t omega_0, real_t B, real_t K) {

	return_code returnval = return_OK;
	returnval = continuous_transfer_function_init((continuous_transfer_function_T*) transfer_function, 0, 2, transfer_function->numerator_data, transfer_function->denominator_data, transfer_function->dX_data, transfer_function->X_data);
	if (returnval != return_OK)
		return returnval;
	transfer_function->numerator_data[0] = K * POW2(omega_0);
	transfer_function->denominator_data[0] = POW2(omega_0);
	transfer_function->denominator_data[1] = 2 * B * omega_0;
	transfer_function->denominator_data[2] = 1;
	return return_OK;
}

static return_code continuous_PID_regulator_update_output(continuous_PID_regulator_T* PID_regulator, real_t time, ODE_solver_step_t step_type) {
	(void) time;
	(void) step_type;
	real_t y, y_P, y_I, y_D;
	y_P = PID_regulator->P_gain * system_input_port_real_value(&PID_regulator->interface, 0);
	y_I = PID_regulator->I_gain * vector_type_at(&PID_regulator->X,real_t, 0);
	y_D = PID_regulator->D_gain * PID_regulator->N_gain * (system_input_port_real_value(&PID_regulator->interface, 0) - vector_type_at(&PID_regulator->X,real_t,1));
	y = y_P + y_I + y_D;
	system_output_port_real_value(&PID_regulator->interface, 0) = y;
	return return_OK;
}

static const vector_generic_T* continuous_PID_regulator_get_derivatives(continuous_PID_regulator_T* PID_regulator, real_t time, ODE_solver_step_t step_type) {

	(void) time;
	(void) step_type;
	vector_type_at(&PID_regulator->dX,real_t,0) = system_input_port_real_value(&PID_regulator->interface, 0);
	vector_type_at(&PID_regulator->dX,real_t,1) = PID_regulator->N_gain * (system_input_port_real_value(&PID_regulator->interface, 0) - vector_type_at(&PID_regulator->X, real_t, 1));
	return &PID_regulator->dX;
}

/**
 * PID controller initialization.
 * Initializes generic continuous system interface.
 * Sets P,I,D gains.
 * @see continuous_SISO_system_generic_init
 *
 * @param PID_regulator PID controller structure pointer
 * @param P_gain proportional gain
 * @param I_gain integrator gain
 * @param D_gain derivative gain
 * @param N_gain derivative filtration gain
 * @return system control library error code
 */
return_code continuous_PID_regulator_init(continuous_PID_regulator_T* PID_regulator, real_t P_gain, real_t I_gain, real_t D_gain, real_t N_gain) {

	PID_regulator->P_gain = P_gain;
	PID_regulator->I_gain = I_gain;
	PID_regulator->D_gain = D_gain;
	PID_regulator->N_gain = N_gain;
	return continuous_SISO_system_generic_init((continuous_SISO_system_generic_T*) PID_regulator, 2, PID_regulator->dX_data, PID_regulator->X_data, (continuous_system_derivatives_fcn) continuous_PID_regulator_get_derivatives, (continuous_system_update_output_fcn) continuous_PID_regulator_update_output, (system_deinit_fcn) NULL);

}

static const vector_generic_T* continuous_nonlinear_system_get_derivatives(continuous_custom_SISO_system_T*, real_t, ODE_solver_step_t);
static return_code continuous_nonlinear_system_update_output(continuous_custom_SISO_system_T*, real_t, ODE_solver_step_t);

static return_code continuous_SISO_nonlinear_system_deinit(continuous_custom_SISO_system_T* system) {
	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&system->params);
	if (returnval != return_OK)
		return returnval;
	system->custom_derrivatives_fcn = NULL;
	system->custom_update_output_fcn = NULL;
	return continuous_SISO_system_generic_deinit((continuous_SISO_system_generic_T*) system);
}

/**
 * Continuous custom linear/nonlinear custom system initialization.
 * Registers custom simplified states derivatives and output update callback functions.
 * Initializes continuous system interface.
 * Initializes system parameters vector.
 *
 * @param system system structure pointer
 * @param derrivatives_fcn system derivatives callback -  function pointer
 * @param output_fcn system output update callback -  function pointer
 * @param order order of system - number of continuous states
 * @param n_params number of system parameters
 * @param params_data system parameters data pointer, if NULL performs dynamic allocation
 * @return system control library error code
 */
return_code continuous_custom_SISO_system_init(continuous_custom_SISO_system_T* system, continuous_custom_system_derrivatives_fcn_T derrivatives_fcn, continuous_custom_system_output_fcn_T output_fcn, size_t order, size_t n_params, real_t* params_data) {
	return_code returnval = return_OK;

	if (derrivatives_fcn == NULL || output_fcn == NULL)
		return return_NULLPTR;

	returnval = vector_type_init(&system->params, real_t, n_params, params_data);
	if (returnval != return_OK)
		return returnval;
	system->custom_derrivatives_fcn = derrivatives_fcn;
	system->custom_update_output_fcn = output_fcn;

	return continuous_SISO_system_generic_init((continuous_SISO_system_generic_T*) system, order, NULL, NULL, (continuous_system_derivatives_fcn) continuous_nonlinear_system_get_derivatives, (continuous_system_update_output_fcn) continuous_nonlinear_system_update_output, (system_deinit_fcn) continuous_SISO_nonlinear_system_deinit);
}
/**
 * Loads values to continuous custom linear/nonlinear system parameters numeric vector.
 * @see vector_generic_load
 * @param system system structure pointer
 * @param params_data_ptr system parameters data pointer
 * @return system control library error code
 */
return_code continuous_custom_SISO_system_params_set(continuous_custom_SISO_system_T* system, const real_t* params_data_ptr) {

	return vector_type_load(&system->params, params_data_ptr);
}

static const vector_generic_T* continuous_nonlinear_system_get_derivatives(continuous_custom_SISO_system_T* system, real_t time, ODE_solver_step_t step_type) {

	(void) step_type;
	system->custom_derrivatives_fcn(system->dX.data_ptr, system->X.data_ptr, system->params.data_ptr, system_input_port_real_value(&system->interface, 0), time);
	return &system->dX;
}

static return_code continuous_nonlinear_system_update_output(continuous_custom_SISO_system_T* system, real_t time, ODE_solver_step_t step_type) {

	(void) time;
	(void) step_type;
	return system->custom_update_output_fcn(system->X.data_ptr, system->params.data_ptr, system_input_port_real_value(&system->interface, 0), system_output_port_real_ptr(&system->interface, 0));

}

