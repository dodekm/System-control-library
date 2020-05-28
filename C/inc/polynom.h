
/**
 * @file polynom.h
 */

#ifndef INC_POLYNOM_H_
#define INC_POLYNOM_H_

#include "common_def.h"
#include "complex.h"
#include "vector.h"

/** @addtogroup polynom Polynomial
 *  @brief Polynomial interface and functions.
 *
 * # Implemented functionality:
 * 	- polynomial real and complex evaluation
 * 	- polynomial multiplication
 * 	- polynomial addition
 * 	- polynomial power for real and complex coefficients
 * 	- polynomial construction from roots (factors)
 * 	- polynomial roots finding
 *  GSL support
 *
 *
 * @{
 */


#define POLYNOM_ROOTS_FIND_N_ITER 100 ///<polynomial root find default number of iterations
#define POLYNOM_ROOTS_FIND_INITIAL_SPACE_SIZE 10.0 ///<polynomial root find default space size
#define POLYNOM_ROOTS_FIND_TOLERANCE 0.0001 ///<polynomial root find default convergence tolerance

/**
 * polynomial type is defined as generic vector
 */
typedef vector_generic_T polynom_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code polynom_eval(const polynom_T*, real_t, real_t*);
return_code polynom_eval_complex(const polynom_T*, complex_t, complex_t*);
return_code polynom_mul(const polynom_T*,const polynom_T*, polynom_T*);
return_code polynom_add(const polynom_T*,const polynom_T*, polynom_T*);
return_code polynom_pow_real(const polynom_T*, polynom_T*, uint);
return_code polynom_pow_complex(const polynom_T*, polynom_T*, uint);
return_code polynom_from_roots(polynom_T*,const vector_generic_T*);
return_code polynom_find_roots(const polynom_T*,vector_generic_T*);
return_code polynom_find_roots_DurandKerner(const polynom_T*, vector_generic_T*, real_t, real_t, uint);
#ifdef USE_GSL
return_code polynom_find_roots_GSL(const polynom_T*, vector_generic_T*);
#endif

/**
 * Second order polynomial coefficients type
 */
typedef struct polynom_second_order {
	real_t coeffs[3];
} polynom_quadratic_T;

/**
 * Second order polynomial roots type
 */
typedef struct roots_second_order {
	complex_t roots[2];
} roots_quadratic_T;

roots_quadratic_T polynom_quadratic_roots(polynom_quadratic_T);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_POLYNOM_H_ */
