/**
 * @file common_def.h
 */

#ifndef COMMON_DEF_H_
#define COMMON_DEF_H_

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>


/** @addtogroup common_def Common interface
 * @brief Common definitions and configurations for system control library.
 *
 * Contains system control library common:
 * - structures
 * - macros
 * - enumerators
 * - operators functions
 * @{
 */

#define ASSERT_NULLPTR ///<configuration macro - enables pointers and structures runtime verification (safe but slower)
#define ASSERT_DIMENSIONS ///<configuration macro - vectors/ matrices runtime dimensions verification (safe but slower)

#define USE_GSL ///<configuration macro - enables GNU Scientific Library integration. If not defined - some functions become unavailable
#define USE_CLIP ///<configuration macro - enables signal saturation interface

#define _FLOAT 0
#define _DOUBLE !_FLOAT

//#define NUMERIC_FLOATING_USE _FLOAT ///<configuration macro - common real number data type selection - float
#define NUMERIC_FLOATING_USE _DOUBLE ///<configuration macro - common real number data type selection - double

#ifdef USE_GSL
#undef NUMERIC_FLOATING_USE
#define NUMERIC_FLOATING_USE _DOUBLE
#endif

#ifdef NUMERIC_FLOATING_USE
#if NUMERIC_FLOATING_USE == _FLOAT
typedef float real_t;  ///<default real number floating point type

#elif NUMERIC_FLOATING_USE == _DOUBLE
typedef double real_t; ///<default real number floating point type
#endif
#else
#error "NUMERIC_FLOATING_USE not defined"
#endif

/**
 * boolean type (byte)
 */
typedef int8_t bool_t;

enum {
	bool_error = -1, bool_false = 0, bool_true = !bool_false,
};

enum {
	uninitialised = 0,
};
/**
 * Generic return code.
 * Return value/error code for most system control library functions
 */
typedef enum {
	return_OK = 0, ///<All OK - function succ./pass
	return_ERROR, ///<Generic error
	return_BUSY, ///<Object is busy or locked
	return_NULLPTR, ///<Passed pointer is NULL or malloc failed (returned NULL)
	return_INDEX_OUT_OF_RANGE, ///<index check failed (array access)
	return_WRONG_DIMENSIONS ///< dimension check failed (matrix or vector)
} return_code;

/**
 * Allocation type info for vectors and matrices.
 */
typedef enum {
	allocation_type_unallocated = 0, ///<Default unallocated state
	allocation_type_static, ///<Global/static/stack data - does not free memory
	allocation_type_dynamic, ///<Heap data - does free memory
} allocation_type_enum;

/**
 * Derivative approximation methods.
 * Discretization methods.
 */
typedef enum {
	approximation_method_forward_euler = 1, approximation_method_backward_euler, approximation_method_trapezoidal_rule, approximation_method_zero_pole_match
} approximation_method;

/**
 * Common control loop signals structure.
 */
typedef struct {
	uint64_t tick; ///<sample counter
	real_t w; ///<control setpoint
	real_t y; ///<system output
	real_t u; ///<control output
} control_loop_signals_T;

/**
 * PID regulator parameters structure.
 */
typedef struct PID_regulator_params {
	real_t P_gain;  ///<proportional gain
	real_t I_gain;  ///<integral gain
	real_t D_gain;  ///<derivative gain
} PID_regulator_params_T;

#define array_length(array) sizeof(array)/sizeof(array[0])
#define array_n_rows(array) (array_length(array))
#define array_n_cols(array) (array_length(array[0]))

#define MAX(A,B) ((A)>(B)?(A):(B)) ///<returns greater of numbers A and B
#define MIN(A,B) ((A)<(B)?(A):(B)) ///<returns lesser of numbers A and B

#define CLIP_TOP(X,A) ((X)>(A)?(A):(X))  ///<saturates numbers greater than A
#define CLIP_BOTTOM(X,A)  ((X)<(A)?(A):(X)) ///<saturates numbers lesser than A
#define NEG_CLIP(X) CLIP_BOTTOM(X,0) ///<saturates negative numbers to 0
#define POS_CLIP(X) CLIP_TOP(X,0) ///<saturates  positive numbers to 0
#define SIGN(X) ((X)<0?-1:((X)>0)) ///<sign of the number X, returns -1,0,1
#define MOD(A,B) ((((A)%(B))+(B))%(B)) ///<integer division remainder, supports negative numbers
#define RANDF(A,B) (((real_t) rand() / (real_t) (RAND_MAX))* ((B)-(A))+(A)) ///< random number from interval <a,b>
#define POW2(X) ((X)*(X))  ///<second power of number/square
#define POW3(X) ((X)*(X)*(X))///<third power of number/cube

/**
 * Real number unary operator/function prototype.
 * Represents generic unary function pointer e.g. sin,sign,recip.
 * @param - input numeric argument
 * @return - result numeric argument
 */
typedef real_t (*real_unary_operator_T)(real_t);

/**
 * Real number binary operator/function prototype.
 * Represents general binary function pointer e.g. C=A+B, C=atan2(A,B)
 * @param - input real number argument A
 * @param - input real number argument B
 * @return numeric result
 */
typedef real_t (*real_binary_operator_T)(real_t, real_t);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Reciprocal (1/x) of the real number.
 * Unary operator function
 * @param A input real number
 * @return output real number
 */
static inline real_t real_recip(real_t A) {

	return 1.0 / A;
}
/**
 * Negative (-x) of the real number.
 * Unary operator function
 * @param A input real number
 * @return output real number
 */
static inline real_t real_neg(real_t A) {
	return -A;
}

/**
 * Sum of two  real numbers.
 * Binary operator function
 * @param A input real number
 * @param B input real number
 * @return output real number
 */

static inline real_t real_sum(real_t A, real_t B) {
	return A + B;
}

/**
 * Substraction of two  real numbers.
 * Binary operator function
 * @param A input real number
 * @param B input real number
 * @return output real number
 */

static inline real_t real_sub(real_t A, real_t B) {
	return A - B;
}

/**
 * Product of two  real numbers.
 * Binary operator function
 * @param A input real number
 * @param B input real number
 * @return output real number
 */

static inline real_t real_mul(real_t A, real_t B) {
	return A * B;
}
/**
 * Ratio of two  real numbers.
 * Binary operator function
 * @param A input real number - numerator
 * @param B input real number - denominator
 * @return output real number
 */

static inline real_t real_div(real_t A, real_t B) {
	return A / B;
}

/**
 * Modulo (division remainder) of two  real numbers.
 * Binary operator function
 * @param A input real number - numerator
 * @param B input real number - denominator
 * @return output real number
 */

static inline real_t real_mod(real_t A, real_t B) {
	return fmod(A, B);
}

/**
 * Linear transformation - line equation y=k*+q.
 * @param x input real number
 * @param k gain
 * @param q offset
 * @return output real number
 */

static inline real_t transform_linear(real_t x, real_t k, real_t q) {
	return (k * x + q);
}

/**
 * Quantize continuous space to discrete by interval.
 * @param x input real number
 * @param step quantization interval
 * @return output real number
 */

static inline real_t quantize(real_t x, real_t step) {
	return ((int) (x / step)) * step;
}

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* COMMON_DEF_H_ */
