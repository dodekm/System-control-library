/**
 * @file system_identification.h
 */

#ifndef INC_SYSTEM_IDENTIFICATION_H_
#define INC_SYSTEM_IDENTIFICATION_H_

#include "common_def.h"
#include "vector.h"
#include "polynom.h"
#include "matrix.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup system_identification System identification algorithms
 * @brief System identification (parameters estimate) algorithms using least squares fitting method.
 * Suitable for linear system parameters estimation where y=thetaT*f(u).
 * Estimates parameters minimizing sum of squared residuals.
 *
 * # Implemented functionality
 * - Single run least squares fitting (offline identification) using SVD decomposition  -> gsl_multifit_linear
 * - Recursive least squares fitting (online identification) algorithm implementation
 * - Single run and recursive version of discrete transfer function identification - estimation of numerator and denominator coefficients
 * */

/**
 * @addtogroup  least_squares Least squares method
 * @ingroup system_identification
 * @{
 */
#ifdef USE_GSL
return_code system_ident_generic_linear_regression(const matrix_T*, const vector_generic_T*, vector_generic_T*, real_t*);
return_code system_ident_polynomial_fit(const vector_generic_T*, const vector_generic_T*, polynom_T*, size_t, real_t*);
return_code system_ident_discrete_transfer_function(const vector_generic_T*, const vector_generic_T*, polynom_T*, polynom_T*, size_t, size_t, real_t*);
#endif

#ifdef __cplusplus
}
#endif

/**
 * @}
 */



/**
 * @addtogroup recursive_least_squares Recursive least squares method
 * @ingroup system_identification
 * @{
 */

/**
 * Recursive least squares method instance structure.
 * Contains data and other preallocated sub-objects required for recursive least squares algorithm
 * Preallocated object dimensions depend on number of estimated parameters provided during initialization
 */
typedef struct system_ident_recursive_least_squares {
	matrix_T Pk_0; ///< Dispersion matrix (old)
	matrix_T Pk_1; ///< Dispersion matrix (new)
	matrix_T YhT;  ///< Y*hT product result matrix
	vector_generic_T theta; ///< estimated parameters vector
	vector_generic_T Y; ///< corrections vector
	real_t lambda;  ///<forgetting factor
	real_t error; ///< error of estimate
} system_ident_recursive_least_squares_T;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns recursive least squares current estimate error.
 * @param RLS recursive least squares instance pointer
 * @return current estimate error
 */
static inline real_t system_ident_recursive_least_squares_get_error(const system_ident_recursive_least_squares_T* RLS) {
	return RLS->error;
}

/**
 * Sets recursive least squares fitting forgetting factor.
 * @param RLS recursive least squares instance pointer
 * @param lambda forgetting factor
 */
static inline void system_ident_recursive_least_squares_set_lambda(system_ident_recursive_least_squares_T* RLS, real_t lambda) {
	RLS->lambda = lambda;
}

/**
 * Returns recursive least squares estimated parameters vector.
 * @param RLS recursive least squares instance pointer
 * @return estimated parameters vector
 */
static inline const vector_generic_T* system_ident_recursive_least_squares_get_estimated_params(system_ident_recursive_least_squares_T* RLS) {
	return &RLS->theta;
}
return_code system_ident_transfer_function_vector_h_iterate(vector_generic_T*, real_t, real_t, size_t, size_t);
return_code system_ident_recursive_least_squares_init(system_ident_recursive_least_squares_T*, size_t, real_t, real_t);
return_code system_ident_recursive_least_squares_reset(system_ident_recursive_least_squares_T*, real_t);
return_code system_ident_recursive_least_squares_deinit(system_ident_recursive_least_squares_T*);
return_code system_ident_recursive_least_squares_estimate_generic(system_ident_recursive_least_squares_T*, const vector_generic_T*, real_t);
return_code system_ident_recursive_least_squares_estimate_discrete_transfer_function(system_ident_recursive_least_squares_T*, const vector_generic_T*, real_t, polynom_T*, polynom_T*);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_SYSTEM_IDENTIFICATION_H_ */
