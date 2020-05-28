/**
 * @file ode_solver.h
 */

#ifndef INC_ODE_SOLVER_H_
#define INC_ODE_SOLVER_H_

#include "common_def.h"
#include "matrix.h"
#include "gsl_wrapper.h"

/** @addtogroup ode_solver ODE solvers
 *  @brief Ordinary differential equation numerical solving algorithms.
 * Fixed step explicit ODE solvers (numerical integration) implementation.
 * Continuous systems interface (states derivatives and output update functions).
 * Runge-Kuttas Butcher's table.
 *
 * # Implemented explicit Runge-Kutta family solvers:
 * 	- ForwardEuler method
 * 	- Midpoint method
 * 	- Ralston method
 * 	- Runge-Kutta 4 method
 *	- GSL ODE solver wrapper
 *	@{
 */

#ifdef __cplusplus
extern "C" {
#endif
/**
 * ODE solver step type enumerator.
 * Numerical integration step type (minor,major)
 */
typedef enum {
	ODE_solver_step_minor = 1, ///< minor numerical integration step
	ODE_solver_step_major	///< major numerical integration step
} ODE_solver_step_t;

/**
 * ODE solver system states derivatives function prototype.
 * @param - pointer to system structure
 * @param - current step time
 * @param - current step type
 * @return states derivatives vector
 */
typedef const vector_generic_T* (*ODE_solver_system_derivatives_fcn)(void*, real_t, ODE_solver_step_t);

/**
 * ODE solver system output update function prototype.
 * @param - pointer to system structure
 * @param - current step time
 * @param - current step type
 * @return system control library error code
 */
typedef return_code (*ODE_solver_system_update_output_fcn)(void*, real_t, ODE_solver_step_t);

/**
 * Generic Runge-Kutta family ODE solver structure.
 * Contains Butcher's table coefficients matrices and working vectors
 */
typedef struct ODE_solver_RK_explicit_generic {
	real_t time; ///< current solver/simulation time
	matrix_T A; ///< Butcher's table matrix A (minor steps operating points weights)
	vector_generic_T B; ///< Butcher's table vector B (weights of derivatives weighted average)
	vector_generic_T C; ///< Butcher's table vector C (time points of minor steps)
	matrix_T k; ///< States derivatives working matrix
	vector_generic_T Xn;///< States working vector
} ODE_solver_RK_explicit_generic_T;

return_code ODE_solver_RK_generic_init(ODE_solver_RK_explicit_generic_T*, size_t,const real_t*,const real_t*,const real_t*);
return_code ODE_solver_RK_explicit_generic_solve(ODE_solver_RK_explicit_generic_T*, real_t, void*, ODE_solver_system_derivatives_fcn, ODE_solver_system_update_output_fcn, vector_generic_T*);
return_code ODE_solver_RK_explicit_generic_deinit(ODE_solver_RK_explicit_generic_T*);

static inline real_t ODE_solver_RK_explicit_generic_get_time(ODE_solver_RK_explicit_generic_T* solver) {
	return solver->time;
}

return_code ODE_solver_RK_explicit_ForwardEuler_init(ODE_solver_RK_explicit_generic_T*);
return_code ODE_solver_RK_explicit_midpoint_init(ODE_solver_RK_explicit_generic_T*);
return_code ODE_solver_RK_explicit_Ralston_init(ODE_solver_RK_explicit_generic_T*);
return_code ODE_solver_RK_explicit_Kutta4_init(ODE_solver_RK_explicit_generic_T*);

#ifdef USE_GSL
typedef struct ODE_solver_GSL {
	gsl_odeiv2_system system_structure;
	gsl_odeiv2_step* gsl_step;
	gsl_odeiv2_control* gsl_control;
	gsl_odeiv2_evolve* gsl_evolve;
	real_t time;
} ODE_solver_GSL;

return_code ODE_solver_GSL_init(ODE_solver_GSL*,const  gsl_odeiv2_step_type *, size_t, real_t, real_t);
return_code ODE_solver_GSL_deinit(ODE_solver_GSL*);
return_code ODE_solver_GSL_solve(ODE_solver_GSL*, real_t, void*, ODE_solver_system_derivatives_fcn, ODE_solver_system_update_output_fcn, vector_generic_T*);
#endif

#ifdef __cplusplus
}
#endif
/** @} */

#endif /* INC_ODE_SOLVER_H_ */
