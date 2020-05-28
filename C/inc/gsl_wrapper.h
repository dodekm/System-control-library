/**
 * @file gsl_wrapper.h
 */

#ifndef INC_GSL_WRAPPER_H_
#define INC_GSL_WRAPPER_H_

#include "common_def.h"
#include "complex.h"
#include "vector.h"
#include "matrix.h"

/** @addtogroup wrappers GSL wrappers
 *  @brief Conversion functions between GSL library structures (vectors and matrices ) and equivalent system control library structures.
 * Complex numbers and complex vectors conversions are also supported.
 * For mutual compatibility with GSL library errorcode/returnval conversion is provided.
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

vector_generic_T matrix_to_vector_real(const matrix_T*);
matrix_T vector_real_to_col_matrix(const  vector_generic_T*);
matrix_T vector_real_to_row_matrix(const vector_generic_T*);

#ifdef __cplusplus
}
#endif

#ifdef USE_GSL
#include "gsl/gsl_errno.h"
#include "gsl/gsl_block.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_permute.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_blas.h"
#include "gsl/gsl_linalg.h"
#include "gsl/gsl_eigen.h"
#include "gsl/gsl_poly.h"
#include "gsl/gsl_odeiv2.h"
#include "gsl/gsl_fit.h"
#include "gsl/gsl_multifit.h"

#ifdef __cplusplus
extern "C" {
#endif

gsl_matrix matrix_to_gsl_matrix(const matrix_T*);
gsl_matrix* matrix_to_gsl_matrix_dynamic_copy(const matrix_T*);
matrix_T gsl_matrix_to_matrix(const gsl_matrix*);

gsl_vector vector_real_to_gsl_vector(const vector_generic_T*);
gsl_vector* vector_real_to_gsl_vector_dynamic_copy(const vector_generic_T*);
vector_generic_T gsl_vector_to_vector_real(const gsl_vector*);
gsl_vector matrix_to_gsl_vector(const matrix_T*);

gsl_vector_complex vector_complex_to_gsl_vector_complex(const vector_generic_T*);
vector_generic_T gsl_vector_complex_to_vector_complex(const gsl_vector_complex*);


return_code gsl_error_code_to_return_code(int);
int return_code_to_gsl_error_code(return_code);

/**
 * Converts  GSL interleaved double array format complex number to system control library complex number structure (real and imaginary part).
 * @param A GSL interleaved complex number structure (double array)
 * @return System control library complex number structure
 */
static inline complex_t gsl_complex_to_complex(gsl_complex A) {
	complex_t C = { real:A.dat[0], imag:A.dat[1] };
	return C;
}

/**
 * Converts system control library complex number structure (real and imaginary part) to GSL interleaved double array format complex number.
 * @param A System control library complex number structure
 * @return GSL complex number structure (double array length of 2)
 */
static inline gsl_complex complex_to_gsl_complex(complex_t A) {
	gsl_complex C = { 0 };
	C.dat[0] = A.real;
	C.dat[1] = A.imag;
	return C;
}

#ifdef __cplusplus
}
#endif

#endif

/** @} */

#endif /* INC_GSL_WRAPPER_H_ */
