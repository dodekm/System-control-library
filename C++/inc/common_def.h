#ifndef COMMON_DEF_H_
#define COMMON_DEF_H_

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#include <functional>
#include <exception>

//#define ASSERT_NULLPTR ///<configuration macro - enables pointers and structures runtime verification (safe but slower)
//#define ASSERT_DIMENSIONS ///<configuration macro - vectors/ matrices runtime dimensions verification (safe but slower)
//#define USE_GSL ///<configuration macro - enables GNU Scientific Library integration. If not defined - some functions become unavailable

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
#include "gsl/gsl_fit.h"
#include "gsl/gsl_multifit.h"
#endif

#define _FLOAT 1
#define _DOUBLE 2
#define _FIXED 3
//#define NUMERIC_FLOATING_USE _FLOAT ///<configuration macro - common real number data type selection - float
//#define NUMERIC_FLOATING_USE _DOUBLE ///<configuration macro - common real number data type selection - double
#define NUMERIC_FLOATING_USE _FLOAT

#ifdef USE_GSL
#undef NUMERIC_FLOATING_USE
#define NUMERIC_FLOATING_USE _DOUBLE
#endif

#ifdef NUMERIC_FLOATING_USE
#if NUMERIC_FLOATING_USE == _FLOAT
typedef float real_t;

#elif NUMERIC_FLOATING_USE == _DOUBLE
typedef double real_t;
#elif NUMERIC_FLOATING_USE == _FIXED
#include "fixmath.h"
typedef Fix16 real_t;
#endif
#else
#error "NUMERIC_FLOATING_USE not defined"
#endif

namespace SystemControl {

/**
 * Generic return code.
 * Return value/error code for most system control library functions
 */
typedef enum {
	exception_OK = 0, ///<All OK - function succ./pass
	exception_ERROR, ///<Generic error
	exception_NULLPTR, ///<Passed pointer is NULL or malloc failed (returned NULL)
	exception_INDEX_OUT_OF_RANGE, ///<Index check failed (array access)
	exception_WRONG_DIMENSIONS ///< Dimension check failed (matrix or vector)
} exception_code_enum;

class exception_code: public std::exception {

public:
	exception_code(exception_code_enum value = exception_OK) :
			value(value) {
	}
	exception_code_enum get_value() const {
		return value;
	}
	explicit operator int() {
		return static_cast<int>(value);
	}
	bool operator==(const exception_code& other) const {
		return this->value == other.value;
	}
	bool operator!=(const exception_code& other) const {
		return this->value != other.value;
	}
	exception_code& operator=(const exception_code& other) {
		this->value = other.value;
		return *this;
	}
	const char* what() const noexcept
	{
		switch (value) {
		case exception_ERROR:
			return "Generic error";
		case exception_NULLPTR:
			return "Passed pointer is NULL or malloc failed (returned NULL)";
		case exception_INDEX_OUT_OF_RANGE:
			return "Index check failed (array access)";
		case exception_WRONG_DIMENSIONS:
			return "Dimension check failed (matrix or vector)";
		default:
			return "Unknown error";
		}
	}

#ifdef USE_GSL

	exception_code(int error) {
		switch (error) {
		case GSL_SUCCESS:
			value = exception_OK;
			break;
		case GSL_ENOMEM:
			value = exception_NULLPTR;
			break;
		case GSL_EFAULT:
			value = exception_NULLPTR;
			break;
		case GSL_EBADLEN:
			value = exception_WRONG_DIMENSIONS;
			break;
		case GSL_ENOTSQR:
			value = exception_WRONG_DIMENSIONS;
			break;
		default:
			value = exception_ERROR;
			break;
		}
	}

	int to_gsl_error_code() const {
		switch (value) {
		case exception_OK:
			return GSL_SUCCESS;
		case exception_ERROR:
			return GSL_FAILURE;
		case exception_NULLPTR:
			return GSL_EFAULT;
		case exception_WRONG_DIMENSIONS:
			return GSL_EBADLEN;
		default:
			return GSL_FAILURE;

		}
	}
#endif

private:
	exception_code_enum value = exception_OK;
};
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
 * Discretization methods
 */
typedef enum {
	approximation_method_forward_euler = 1, approximation_method_backward_euler, approximation_method_trapezoidal_rule, approximation_method_zero_pole_match
} approximation_method;

typedef enum {
	step_minor = 1, step_major
} step_type;

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


template <typename T> void swap(T& a,T& b)
{
	T tmp=a;
	a=b;
	b=tmp;
}

namespace Operators {

template<typename T>
using unary_operator=std::function<T(T)>;
template<typename T>
using binary_operator=std::function<T(T,T)>;

template<typename T>
inline T recip(T A) {

	return T(1.0) / A;
}

template<typename T>
inline T neg(T A) {
	return -A;
}

template<typename T>
inline T sum(T A, T B) {
	return A + B;
}

template<typename T>
inline T sub(T A, T B) {
	return A - B;
}

template<typename T>
inline T mul(T A, T B) {
	return A * B;
}

template<typename T>
inline T div(T A, T B) {
	return A / B;
}

inline real_t real_mod(real_t A, real_t B) {
	return ::fmod((double) A, (double) B);
}

inline real_t transform_linear(real_t x, real_t k, real_t q) {
	return (k * x + q);
}

inline real_t quantize(real_t x, real_t step) {
	return ((real_t) (int) (x / step)) * step;
}

inline real_t mag2dB(real_t mag) {
	if (mag > 0.0)
		return 20.0 * ::log10((double) mag);
	else
		return NAN;
}

inline real_t dB2mag(real_t dB) {
	return ::exp((double) (dB / 20.0 * ::log(10)));
}
}
}

#endif /* COMMON_DEF_H_ */
