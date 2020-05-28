/**
 * @file complex.h
 */

#ifndef INC_COMPLEX_H_
#define INC_COMPLEX_H_

#include "common_def.h"
#include "vector.h"

/**
 *  @addtogroup complex Complex numbers algebra
 *  @brief Complex numbers interface and basic algebra.
 *
 * # Implemented functionality
 * - Complex numbers representations structure
 * - Polar (goniometric) complex number representation
 * - Magnitude and phase of complex number
 * - Conversion functions
 * - Basic and advanced complex numbers algebra
 * - Complex exponential and logarithm
 * @{
 */

/**
 * Component representation of complex number.
 * Contains real and imaginary part
 */
typedef struct complex {
	real_t real; ///<real part of complex number
	real_t imag; ///<imaginary part of complex number
} complex_t;

/**
 * Goniometric representation of complex number.
 * Contains magnitude (size) and phase (argument)
 */
typedef struct polar {
	real_t mag; ///<magnitude (Euclidean size)
	real_t phase; ///<phase (argument)
} polar_t;

/**
 * Complex number binary arithmetical operator function prototype.
 * C=f(A,B).
 * Example operators:
 * - @ref complex_add
 * - @ref complex_sub
 * - @ref complex_mul
 * - @ref complex_div
 * @param - complex number A
 * @param - complex number B
 * @return complex number C
 */
typedef complex_t (*complex_to_complex_binary_operator_T)(complex_t, complex_t);

/**
 * Complex number unary arithmetical operator function prototype.
 * B=f(A).
 * Example operators:
 * - @ref complex_conj
 * @param - complex number A
 * @return - complex number B
 */
typedef complex_t (*complex_to_complex_unary_operator_T)(complex_t);

/**
 * Complex number to real number operator function prototype.
 * B=f(A)
 * Example operators:
 * - @ref complex_magnitude
 * - @ref complex_phase
 * - @ref complex_real_part
 * - @ref complex_imag_part
 * @param - complex number A
 * @return real number
 */
typedef real_t (*complex_to_real_unary_operator_T)(complex_t);

#ifdef __cplusplus
extern "C" {
#endif

return_code vector_complex_to_complex_binary_operator(const vector_generic_T*, const vector_generic_T*, vector_generic_T*, complex_to_complex_binary_operator_T);
return_code vector_complex_to_complex_unary_operator(const vector_generic_T*, vector_generic_T*, complex_to_complex_unary_operator_T);
return_code vector_complex_to_real_unary_operator(const vector_generic_T*, vector_generic_T*, complex_to_real_unary_operator_T);

return_code vector_complex_to_polar(const vector_generic_T*, vector_generic_T*);
return_code vector_polar_to_complex(const vector_generic_T*, vector_generic_T*);

/**
 * Complex number addition.
 * C=A+B. Addition by components.
 * @param A complex number A
 * @param B complex number B
 * @return sum of complex number
 */
static inline complex_t complex_add(complex_t A, complex_t B) {
	complex_t C = { real:A.real + B.real, imag:A.imag + B.imag };
	return C;
}

/**
 * Complex number substraction.
 * C=A-B. Difference by components.
 * @param A complex number A
 * @param B complex number B
 * @return difference of complex number
 */

static inline complex_t complex_sub(complex_t A, complex_t B) {
	complex_t C = { real:A.real - B.real, imag:A.imag - B.imag };
	return C;
}
/**
 * Complex number multiplication.
 * C=A*B
 * @param A complex number A
 * @param B complex number B
 * @return product of complex number
 */

static inline complex_t complex_mul(complex_t A, complex_t B) {
	complex_t C = { real:A.real * B.real - A.imag * B.imag, imag:A.real * B.imag + A.imag * B.real };
	return C;
}

/**
 * Complex number division.
 * C=A/B
 * @param A complex number A
 * @param B complex number B
 * @return difference of complex number
 */

static inline complex_t complex_div(complex_t A, complex_t B) {
	real_t den = POW2(B.real) + POW2(B.imag);
	complex_t C = { real:(A.real * B.real + A.imag * B.imag) / den, imag:(A.imag * B.real - B.imag * A.real) / den };
	return C;
}

/**
 * Complex number conjungate.
 * B={A.real,-A.imag}
 * @param A complex number A
 * @return conjungate of complex number
 */

static inline complex_t complex_conj(complex_t A) {
	complex_t C = { real:A.real, imag:-A.imag };
	return C;
}

/**
 * Complex number magnitude.
 * Euclidean size of complex number {modul)
 * @param A complex number
 * @return magnitude
 */

static inline real_t complex_magnitude(complex_t A) {
	return sqrt(POW2(A.real) + POW2(A.imag));
}

/**
 * Complex number phase.
 * Argument of complex number.
 * Uses atan2.
 * @param A complex number
 * @return phase
 */

static inline real_t complex_phase(complex_t A) {
	return atan2(A.imag, A.real);
}

/**
 * Extracts real part of complex number.
 * @param A complex number
 * @return A.real
 */
static inline real_t complex_real_part(complex_t A) {
	return A.real;
}
/**
 * Extracts imaginary part of complex number.
 * @param A complex number
 * @return A.imag
 */
static inline real_t complex_imag_part(complex_t A) {
	return A.imag;
}

/**
 * Complex number exponential.
 * @param C complex number
 * @return complex number
 */
static inline complex_t complex_exponential(complex_t C) {
	real_t real_exponential = exp(C.real);
	complex_t C_exp = { real:real_exponential * cos(C.imag), imag:real_exponential * sin(C.imag) };
	return C_exp;
}
/**
 * Complex number logarithm.
 * @param C complex number
 * @return complex number
 */
static inline complex_t complex_log(complex_t C) {
	real_t magnitude = complex_magnitude(C);
	real_t phase = complex_phase(C);
	complex_t C_log = { real:log(magnitude), imag:phase };
	return C_log;
}

/**
 * Converts component to goniometric  representation of complex number.
 * @param C component representation of complex number
 * @return goniometric representation of complex number
 */

static inline polar_t complex_to_polar(complex_t C) {
	polar_t P = { mag:complex_magnitude(C), phase:complex_phase(C) };
	return P;
}

/**
 * Converts goniometric to component representation of complex number.
 * @param P goniometric representation of complex number
 * @return component representation of complex number
 */
static inline complex_t polar_to_complex(polar_t P) {
	complex_t C = { real:P.mag * cos(P.phase), imag:P.mag * sin(P.phase) };
	return C;
}

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_COMPLEX_H_ */
