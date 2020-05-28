#include "ode_solver.h"

/**
 * Runge-Kutta family ODE solver initialization.
 * Initializes solvers Butcher's table matrices witch provided data pointers
 * @param solver ODE solver structure pointer
 * @param order order of solver - number of minor steps, affects matrices dimensions
 * @param A_data pointer to A matrix data
 * @param B_data pointer to B vector data
 * @param C_data pointer to C vector data
 * @return system control library error code
 */
return_code ODE_solver_RK_generic_init(ODE_solver_RK_explicit_generic_T* solver, size_t order, const real_t* A_data, const real_t* B_data, const real_t* C_data) {

	return_code returnval = return_OK;

	returnval = matrix_init(&solver->A, order, order, A_data);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&solver->B, real_t, order, B_data);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&solver->C, real_t, order, C_data);
	if (returnval != return_OK)
		return returnval;
	solver->time = 0;

	return return_OK;
}

/**
 * Runge-Kutta family ODE solver de-initialization.
 * Deinitializes solvers Butcher's table matrices and working matrices
 * @param solver ODE solver structure pointer
 * @return system control library error code
 */
return_code ODE_solver_RK_explicit_generic_deinit(ODE_solver_RK_explicit_generic_T* solver) {

	return_code returnval = return_OK;
	returnval = matrix_deinit(&solver->A);
	if (returnval != return_OK)
		return returnval;
	returnval =  vector_generic_deinit(&solver->B);
	if (returnval != return_OK)
		return returnval;
	returnval =  vector_generic_deinit(&solver->C);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_deinit(&solver->k);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&solver->Xn);
	if (returnval != return_OK)
		return returnval;

	return return_OK;
}

static const real_t ForwardEuler_A_values[1][1] = { { 0 } };
static const real_t ForwardEuler_B_values[1] = { 1 };
static const real_t ForwardEuler_C_values[1] = { 0 };

return_code ODE_solver_RK_explicit_ForwardEuler_init(ODE_solver_RK_explicit_generic_T* solver) {

	return ODE_solver_RK_generic_init(solver, 1, *ForwardEuler_A_values, ForwardEuler_B_values, ForwardEuler_C_values);

}

static const real_t Midpoint_A_values[2][2] = { { 0, 0 }, { 0, 0.5 } };
static const real_t Midpoint_B_values[2] = { 0, 1 };
static const real_t Midpoint_C_values[2] = { 0, 0.5 };

return_code ODE_solver_RK_explicit_midpoint_init(ODE_solver_RK_explicit_generic_T* solver) {

	return ODE_solver_RK_generic_init(solver, 2, *Midpoint_A_values, Midpoint_B_values, Midpoint_C_values);

}

static const real_t Ralston_A_values[3][3] = { { 0, 0, 0 }, { 0.5, 0, 0 }, { 0, 3.0 / 4.0, 0 } };
static const real_t Ralston_B_values[3] = { 0, 0.5, 3.0 / 4.0 };
static const real_t Ralston_C_values[3] = { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0 };

return_code ODE_solver_RK_explicit_Ralston_init(ODE_solver_RK_explicit_generic_T* solver) {

	return ODE_solver_RK_generic_init(solver, 3, *Ralston_A_values, Ralston_B_values, Ralston_C_values);
}

static const real_t Kutta4_A_values[4][4] = { { 0, 0, 0, 0 }, { 0.5, 0, 0, 0 }, { 0, 0.5, 0, 0 }, { 0, 0, 1, 0 } };
static const real_t Kutta4_B_values[4] = { 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 };
static const real_t Kutta4_C_values[4] = { 0, 0.5, 0.5, 1 };

return_code ODE_solver_RK_explicit_Kutta4_init(ODE_solver_RK_explicit_generic_T* solver) {

	return ODE_solver_RK_generic_init(solver, 4, *Kutta4_A_values, Kutta4_B_values, Kutta4_C_values);
}

/**
 * Runge-Kutta family ODE solver iteration.
 * Solves new system states using initialized RK ODE solver.
 * Function uses dynamically allocated working matrices
 *
 * @param solver ODE solver structure pointer
 * @param step_size numerical integration fixed step size (step time)
 * @param system pointer to system
 * @param get_derivatives_fcn system states derivatives function pointer
 * @param update_output_fcn system output update function pointer
 * @param X target system states vector
 * @return system control library error code
 */
return_code ODE_solver_RK_explicit_generic_solve(ODE_solver_RK_explicit_generic_T* solver, real_t step_size, void* system, ODE_solver_system_derivatives_fcn get_derivatives_fcn, ODE_solver_system_update_output_fcn update_output_fcn, vector_generic_T* X) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (solver == NULL || system == NULL || get_derivatives_fcn == NULL)
		return return_NULLPTR;
	returnval = vector_assert(X);
	if (returnval != return_OK)
		return returnval;
#endif

	uint system_order = vector_length(X);
	uint solver_order = solver->A.n_rows;

	if (matrix_assert(&solver->k) != return_OK || solver->k.n_rows != solver_order || solver->k.n_cols != system_order) {
		matrix_deinit(&solver->k);
		returnval = matrix_init(&solver->k, solver_order, system_order, NULL);
		if (returnval != return_OK)
			return returnval;
	}

	returnval =vector_generic_copy(&solver->Xn, X);
	if (returnval != return_OK)
		return returnval;

	for (uint i = 0; i < solver_order; i++) {
		if (i > 0) {
			for (uint n = 0; n < system_order; n++) {
				real_t Xn_minor = vector_type_at(&solver->Xn,real_t,n);
				for (uint j = 0; j < i; j++) {
					Xn_minor += matrix_at(&solver->A, i, j)* matrix_at(&solver->k, j, n);
				}
				vector_type_at(X,real_t,n)=Xn_minor;
			}
		}
		real_t time_minor=solver->time+step_size*vector_type_at(&solver->C,real_t,i);

		if(update_output_fcn!=NULL)
		{
			returnval=update_output_fcn(system, solver->time, ODE_solver_step_minor);
			if (returnval != return_OK)
			return returnval;
		}
		const vector_generic_T* dX = get_derivatives_fcn(system,time_minor,ODE_solver_step_minor);
#ifdef ASSERT_NULLPTR
		returnval =vector_assert(dX);
		if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
		if (vector_length(dX)!=system_order)
		return return_WRONG_DIMENSIONS;
#endif
		for (uint n = 0; n < system_order; n++) {
			matrix_at(&solver->k,i,n)=step_size*vector_type_at(dX,real_t,n);
		}
	}
	for (uint n = 0; n < system_order; n++) {
		vector_type_at(X,real_t,n)=vector_type_at(&solver->Xn,real_t,n);
		for (uint i = 0; i < solver_order; i++) {
			vector_type_at(X,real_t,n)+=vector_type_at(&solver->B,real_t,i)*matrix_at(&solver->k,i,n);
		}
	}
	solver->time += step_size;
	if (update_output_fcn != NULL) {
		returnval = update_output_fcn(system, solver->time, ODE_solver_step_major);
		if (returnval != return_OK)
			return returnval;
	}
	get_derivatives_fcn(system, solver->time, ODE_solver_step_major);

	return return_OK;
}
#ifdef USE_GSL
static int GSL_ODE_solver_derivatives_fcn_wrapper(double, const double[], double[], void*);

return_code ODE_solver_GSL_init(ODE_solver_GSL* solver, const gsl_odeiv2_step_type * step_type, size_t system_order, real_t eps_abs, real_t eps_rel) {

#ifdef ASSERT_NULLPTR
	if (solver == NULL)
		return return_NULLPTR;
	if (step_type == NULL)
		return return_NULLPTR;
#endif

	solver->system_structure.dimension = system_order;
	solver->system_structure.function = GSL_ODE_solver_derivatives_fcn_wrapper;
	solver->system_structure.jacobian = NULL;
	solver->system_structure.params = NULL;

	gsl_odeiv2_step* gsl_step = gsl_odeiv2_step_alloc(step_type, system_order);
	gsl_odeiv2_control * gsl_control = gsl_odeiv2_control_y_new(eps_abs, eps_rel);
	gsl_odeiv2_evolve * gsl_evolve = gsl_odeiv2_evolve_alloc(system_order);

#ifdef ASSERT_NULLPTR
	if (gsl_step == NULL)
		return return_NULLPTR;
	if (gsl_control == NULL)
		return return_NULLPTR;
	if (gsl_evolve == NULL)
		return return_NULLPTR;
#endif
	solver->gsl_step = gsl_step;
	solver->gsl_control = gsl_control;
	solver->gsl_evolve = gsl_evolve;
	return return_OK;
}
return_code ODE_solver_GSL_deinit(ODE_solver_GSL* solver) {
#ifdef ASSERT_NULLPTR
	if (solver == NULL)
		return return_NULLPTR;
#endif
	gsl_odeiv2_step_free(solver->gsl_step);
	gsl_odeiv2_control_free(solver->gsl_control);
	gsl_odeiv2_evolve_free(solver->gsl_evolve);
	return return_OK;
}

typedef struct ODE_solver_GSL_system_params {
	void* system_ptr;
	ODE_solver_system_derivatives_fcn get_derivatives_fcn;
	vector_generic_T* X;
} ODE_solver_system_params;

return_code ODE_solver_GSL_solve(ODE_solver_GSL* solver, real_t step_size, void* system_ptr, ODE_solver_system_derivatives_fcn get_derivatives_fcn, ODE_solver_system_update_output_fcn update_output_fcn, vector_generic_T* X) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (solver == NULL || system_ptr == NULL || get_derivatives_fcn == NULL)
		return return_NULLPTR;
	returnval = vector_assert(X);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(X) != solver->system_structure.dimension)
		return return_WRONG_DIMENSIONS;
#endif
	ODE_solver_system_params system_params = { system_ptr, get_derivatives_fcn, X };
	solver->system_structure.params = (void*) &system_params;
	returnval = gsl_error_code_to_return_code(gsl_odeiv2_evolve_apply_fixed_step(solver->gsl_evolve, solver->gsl_control, solver->gsl_step, &solver->system_structure, &solver->time, step_size, X->data_ptr));
	if (returnval != return_OK)
		return returnval;
	if (update_output_fcn != NULL) {
		returnval = update_output_fcn(system_ptr, solver->time, ODE_solver_step_major);
		if (returnval != return_OK)
			return returnval;
	}
	return return_OK;
}

static int GSL_ODE_solver_derivatives_fcn_wrapper(double time, const double x_ptr[], double dx_ptr[], void *params) {

	return_code returnval = return_OK;
	ODE_solver_system_params* system_params = (ODE_solver_system_params*) params;
	real_t* X_original_ptr = system_params->X->data_ptr;
	system_params->X->data_ptr = x_ptr;

	const vector_generic_T* dX_system = system_params->get_derivatives_fcn(system_params->system_ptr, time, ODE_solver_step_minor);
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(dX_system);
	if (returnval != return_OK)
		goto ret;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(dX_system)!= vector_length(system_params->X)) {
		returnval = return_WRONG_DIMENSIONS;
		goto ret;
	}
#endif
	vector_generic_T dX_solver = { 0 };
	returnval = vector_type_init(&dX_solver,real_t,vector_length( dX_system), dx_ptr);
	if (returnval != return_OK)
		goto ret;
	returnval = vector_generic_copy(&dX_solver, dX_system);
	if (returnval != return_OK)
		goto ret;
	ret: return return_code_to_gsl_error_code(returnval);
	system_params->X->data_ptr = X_original_ptr;
}
#endif
