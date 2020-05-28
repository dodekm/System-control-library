#include "signal_process.h"
#include "gsl_wrapper.h"

/**
 * Performs real number vector elements order reverse.
 * Function operates in-place.
 * Asserts vectors first.
 * @param vector vector to reverse
 * @return system control library error code
 */
return_code vector_real_reverse(vector_generic_T* vector) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(vector, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector);

	for (uint i = 0; i < length / 2; i++) {
		real_t a = vector_type_at(vector, real_t, i);
		real_t b = vector_type_at(vector, real_t, length - 1 - i);
		vector_type_at(vector,real_t,i) = b;
		vector_type_at(vector,real_t,length-1-i) = a;
	}
	return return_OK;
}

/**
 * Discrete convolution of real numbers vectors  f*g.
 * Length of resulting vector is length_f + length_g - 1.
 * Vectors f and g are free in length.
 * Asserts vectors first.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_f vector to convolute
 * @param vector_g vector to convolute - usually kernel
 * @param vector_f_g resulting convolution (destination) vector
 * @return system control library error code
 */

return_code vector_real_convolute(const vector_generic_T* vector_f, const vector_generic_T* vector_g, vector_generic_T* vector_f_g) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_f);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_g);
	if (returnval != return_OK)
		return returnval;
#endif

	int length_f = vector_length(vector_f);
	int length_g = vector_length(vector_g);
	int length_f_g = length_f + length_g - 1;

	if (vector_assert(vector_f_g) != return_OK) {
		returnval = vector_type_init(vector_f_g, real_t, length_f_g, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_f_g) != length_f_g)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_f, real_t) || !vector_is_type(vector_g, real_t) || !vector_is_type(vector_f_g, real_t))
		return return_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < length_f_g; n++) {
		int kmin = (n >= length_g - 1) ? n - (length_g - 1) : 0;
		int kmax = (n < length_f - 1) ? n : length_f - 1;
		real_t sum = 0;
		for (int k = kmin; k <= kmax; k++) {
			sum += vector_type_at(vector_f, real_t, k) * vector_type_at(vector_g, real_t, n - k);
		}
		vector_type_at(vector_f_g,real_t,n) = sum;
	}
	return return_OK;
}

/**
 * Discrete convolution of complex numbers vectors  f*g.
 * Length of resulting vector is length_f + length_g - 1.
 * Vectors f and g are free in length.
 * Asserts vectors first.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_f vector to convolute
 * @param vector_g vector to convolute - kernel
 * @param vector_f_g resulting convolution (destination) vector
 * @return system control library error code
 */

return_code vector_complex_convolute(const vector_generic_T* vector_f, const vector_generic_T* vector_g, vector_generic_T* vector_f_g) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_f);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_g);
	if (returnval != return_OK)
		return returnval;
#endif
	int length_f = vector_length(vector_f);
	int length_g = vector_length(vector_g);
	int length_f_g = length_f + length_g - 1;

	if (vector_assert(vector_f_g) != return_OK) {
		returnval = vector_type_init(vector_f_g, complex_t, length_f_g, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_f_g) != length_f_g)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_f, complex_t) || !vector_is_type(vector_g, complex_t) || !vector_is_type(vector_f_g, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < length_f_g; n++) {
		int kmin = (n >= length_g - 1) ? n - (length_g - 1) : 0;
		int kmax = (n < length_f - 1) ? n : length_f - 1;
		complex_t sum = { 0, 0 };
		for (int k = kmin; k <= kmax; k++) {
			sum = complex_add(sum, complex_mul(vector_type_at(vector_f, complex_t, k), vector_type_at(vector_g, complex_t, n - k)));
		}
		vector_type_at(vector_f_g,complex_t,n) = sum;
	}

	return return_OK;

}

/**
 * Discrete convolution of real numbers vectors  f*g while g is symmetric kernel.
 * Length of resulting vector is length_f + length_g - 1.
 * Vectors f and g are free in length.
 * Vector g must be odd.
 * Asserts vectors first.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 *
 * @param vector_f vector to convolute
 * @param vector_g vector to convolute - kernel
 * @param vector_f_g resulting convolution (destination) vector
 * @return system control library error code
 */
return_code vector_real_convolute_symetric_kernel(const vector_generic_T* vector_f, const vector_generic_T* vector_g, vector_generic_T* vector_f_g) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_f);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_g);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_g) % 2 == 0)
		return return_WRONG_DIMENSIONS;
#endif

	int length_f = vector_length(vector_f);
	int length_g = vector_length(vector_g);
	int length_f_g = length_f + length_g - 1;

	if (vector_assert(vector_f_g) != return_OK) {
		returnval = vector_type_init(vector_f_g, real_t, length_f_g, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_f_g) != length_f_g)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_f, real_t) || !vector_is_type(vector_g, real_t) || !vector_is_type(vector_f_g, real_t))
		return return_WRONG_DIMENSIONS;
#endif

	int M = (vector_length(vector_g) - 1) / 2;

	for (int n = 0; n < length_f_g; n++) {
		real_t sum = 0;
		for (int m = -M; m <= M; m++) {
			int f_idx = n - m;
			f_idx = CLIP_BOTTOM(f_idx, 0);
			f_idx = CLIP_TOP(f_idx, length_f - 1);
			int g_idx = m + M;
			g_idx = CLIP_BOTTOM(g_idx, 0);
			g_idx = CLIP_TOP(f_idx, length_g - 1);
			sum += vector_type_at(vector_f,real_t,f_idx) * vector_type_at(vector_g, real_t, g_idx);
		}
		vector_type_at(vector_f_g,real_t,n) = sum;
	}
	return return_OK;
}

/**
 * Performs linear transformation of real numbers vector elements.
 * Scaling and offseting of all elements of vector.
 * B(i)=k*A(i)+q.
 * Function can work in-place (source and destination pointers can be same).
 * Asserts vectors first.
 * Vectors must have same length.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_src source vector
 * @param vector_dst destination vector
 * @param k gain - multiplicative factor
 * @param q offset - addition factor
 * @return system control library error code
 */
return_code vector_real_transform_linear(const vector_generic_T* vector_src, vector_generic_T* vector_dst, real_t k, real_t q) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
#endif
	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_src), NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_src))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_dst, real_t) || !vector_is_type(vector_src, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_src);
	real_t* src_ptr = vector_type_data_ptr(vector_src, real_t);
	real_t* dst_ptr = vector_type_data_ptr(vector_dst, real_t);
	for (uint i = 0; i < length; i++) {
		dst_ptr[i] = transform_linear(src_ptr[i], k, q);
	}
	return return_OK;
}
/**
 * Vector element wise unary operation.
 * B(i)=f(A(i)).
 * Asserts vectors first.
 * Vectors must have same length.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Function can work in-place (source and destination pointers can be same)
 * @param vector_src source vector A
 * @param vector_dst destination vector B
 * @param operator operator function pointer f()
 * @return system control library error code
 */
return_code vector_real_element_unary_operator(const vector_generic_T* vector_src, vector_generic_T* vector_dst, real_unary_operator_T operator) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif

	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_src), NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_src))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_src, real_t) || !vector_is_type(vector_dst, real_t))
		return return_WRONG_DIMENSIONS;
#endif

	real_t* src_ptr = vector_type_data_ptr(vector_src, real_t);
	real_t* dst_ptr = vector_type_data_ptr(vector_dst, real_t);

	size_t length = vector_length(vector_src);
	for (uint i = 0; i < length; i++) {
		dst_ptr[i] = operator(src_ptr[i]);
	}
	return return_OK;
}

/**
 *
 * Vector-vector element wise binary operation.
 * C(i)=f(A(i),B(i)).
 * Asserts vectors first.
 * Vectors must have same length.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Function can work in-place (source and destination pointers can be same)
 *
 * @param vector_A source vector A
 * @param vector_B source vector B
 * @param vector_dst destination vector C
 * @param operator operator function pointer f()
 * @return system control library error code
 */

return_code vector_real_element_binary_operator(const vector_generic_T* vector_A, const vector_generic_T* vector_B, vector_generic_T* vector_dst, real_binary_operator_T operator) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_B);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif

	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_A), NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_A) || vector_length(vector_dst) != vector_length(vector_B))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_A, real_t) || !vector_is_type(vector_B, real_t) || !vector_is_type(vector_dst, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	real_t* A_ptr = vector_type_data_ptr(vector_A, real_t);
	real_t* B_ptr = vector_type_data_ptr(vector_B, real_t);
	real_t* dst_ptr = vector_type_data_ptr(vector_dst, real_t);
	size_t length = vector_length(vector_A);
	for (uint i = 0; i < length; i++) {
		dst_ptr[i] = operator(A_ptr[i], B_ptr[i]);
	}
	return return_OK;

}

/**
 * Vector-scalar element wise binary operation.
 * C(i)=f(A(i),B).
 * Asserts vectors first.
 * Vectors must have same length.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Function can work in-place (source and destination pointers can be same).
 *
 * @param vector_src source vector A
 * @param vector_dst destination vector C
 * @param scalar_value scalar value B
 * @param operator operator function pointer f()
 * @return system control library error code
 */
return_code vector_real_scalar_operator(const vector_generic_T* vector_src, vector_generic_T* vector_dst, real_t scalar_value, real_binary_operator_T operator) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif

	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_src), NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_src))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_dst, real_t) || !vector_is_type(vector_src, real_t))
		return return_WRONG_DIMENSIONS;
#endif

	real_t* src_ptr = vector_type_data_ptr(vector_src, real_t);
	real_t* dst_ptr = vector_type_data_ptr(vector_dst, real_t);
	size_t length = vector_length(vector_dst);
	for (uint i = 0; i < length; i++) {
		dst_ptr[i] = operator(src_ptr[i], scalar_value);
	}
	return return_OK;

}

/**
 * Vector real number 1D linear interpolation.
 * Vector vector_x and vector_y must be same length.
 * Vector vector_x_q and vector_y_q must be same length.
 *
 * @param vector_x vector of sample points x
 * @param vector_y vector of sample values f(x)
 * @param vector_x_q vector of coordinates of the query points
 * @param vector_y_q destination vector of interpolated values
 * @return system control library error code
 */

return_code vector_real_interpolate(const vector_generic_T* vector_x, const vector_generic_T* vector_y, const vector_generic_T* vector_x_q, vector_generic_T* vector_y_q) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR

	returnval = vector_assert(vector_x);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_assert(vector_x_q);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_y);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_assert(vector_y_q);
	if (returnval != return_OK) {
		returnval = vector_type_init(vector_y_q, real_t, vector_length(vector_x_q), NULL);
		if (returnval != return_OK)
			return returnval;
	}

#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_x) != vector_length(vector_y))
		return return_WRONG_DIMENSIONS;
	if (vector_length(vector_x_q) != vector_length(vector_y_q))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_x, real_t) || !vector_is_type(vector_x_q, real_t))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_y, real_t) || !vector_is_type(vector_y_q, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length_x = vector_length(vector_x);
	size_t length_x_q = vector_length(vector_x_q);

	int j = 0;
	for (uint i = 0; i < length_x_q; i++) {
		while (1) {
			if (j < length_x - 2 && vector_type_at(vector_x_q,real_t,i) > vector_type_at(vector_x, real_t, j) && vector_type_at(vector_x_q,real_t,i) > vector_type_at(vector_x, real_t, j + 1))
				j++;
			else if (j > 0 && vector_type_at(vector_x_q,real_t,i) < vector_type_at(vector_x, real_t, j) && vector_type_at(vector_x_q,real_t,i) < vector_type_at(vector_x, real_t, j + 1))
				j--;
			else
				break;
		}

		real_t k = (vector_type_at(vector_y,real_t,j+1) - vector_type_at(vector_y, real_t, j)) / (vector_type_at(vector_x,real_t,j+1) - vector_type_at(vector_x, real_t, j));
		real_t y = vector_type_at(vector_y,real_t,j) + (vector_type_at(vector_x_q,real_t,i) - vector_type_at(vector_x, real_t, j)) * k;
		vector_type_at(vector_y_q,real_t,i) = y;
	}

	return return_OK;
}
/**
 * Real number vector elements difference.
 * B(i)=A(i+1)-A(i).
 * Asserts vectors first.
 * Destination vector has length of source vector-1.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 *
 * @param vector_src source vector
 * @param vector_dst destination vector
 * @return system control library error code
 */

return_code vector_real_diff(const vector_generic_T* vector_src, vector_generic_T* vector_dst) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
#endif
	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_src)-1, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_src) - 1)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_dst, real_t) || !vector_is_type(vector_src, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_src) - 1;
	for (uint n = 0; n < length; n++) {
		vector_type_at(vector_dst,real_t,n) = vector_type_at(vector_src,real_t,n+1) - vector_type_at(vector_src, real_t, n);
	}
	return return_OK;
}

/**
 * Real number vector elements integration (cumulative sum).
 * B(i)=sum(A(j)) for j=1:i.
 * Asserts vectors first.
 * Destination vector has same length as source vector.
 * If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_src source vector
 * @param vector_dst destination vector
 * @return system control library error code
 */

return_code vector_real_integrate(const vector_generic_T* vector_src, vector_generic_T* vector_dst) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR

	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
#endif
	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, vector_length(vector_src), NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_dst) != vector_length(vector_src))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_src, real_t) || !vector_is_type(vector_dst, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_src);
	real_t sum = 0;
	for (uint n = 0; n < length; n++) {
		sum += vector_type_at(vector_src, real_t, n);
		vector_type_at(vector_dst,real_t,n) = sum;
	}
	return return_OK;
}

/**
 * Sum of real number vector elements.
 * @param vector vector to sum
 * @param sum_ptr pointer to sum result
 * @return system control library error code
 */
return_code vector_real_sum(const vector_generic_T* vector, real_t* sum_ptr) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (sum_ptr == NULL)
		return return_NULLPTR;
	returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(vector, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector);
	real_t sum = 0;
	for (uint n = 0; n < length; n++) {
		sum += vector_type_at(vector, real_t, n);
	}
	*sum_ptr = sum;
	return return_OK;
}

/**
 * Dot product of real number vectors.
 * sum(A(i)*B(i)).
 * Asserts vectors first.
 * Vectors must have same length.
 * Function has GSL/BLAS support
 * @param vector_A source vector A
 * @param vector_B source vector B
 * @param dot_product_ptr pointer to result
 * @return system control library error code
 */

return_code vector_real_dot_product(const vector_generic_T* vector_A, const vector_generic_T* vector_B, real_t* dot_product_ptr) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (dot_product_ptr == NULL)
		return return_NULLPTR;
	returnval = vector_assert(vector_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_B);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_A) != vector_length(vector_B))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_A, real_t) || !vector_is_type(vector_B, real_t))
		return return_WRONG_DIMENSIONS;
#endif

#ifdef USE_GSL
	gsl_vector vector_A_gsl = vector_real_to_gsl_vector(vector_A);
	gsl_vector vector_B_gsl = vector_real_to_gsl_vector(vector_B);
	return gsl_error_code_to_return_code(gsl_blas_ddot(&vector_A_gsl, &vector_B_gsl, dot_product_ptr));

#else
	size_t length = vector_length(vector_A);
	real_t product_sum = 0;
	for (uint n = 0; n < length; n++) {
		product_sum += vector_type_at(vector_A, real_t, n) * vector_type_at(vector_B, real_t, n);
	}
	*dot_product_ptr = product_sum;
	return return_OK;
#endif
}
/**
 * Filtrates real signal vector with FIR filter kernel.
 * Performs finite impulse response digital filtration of signal.
 * - source and destination signals must have different data pointer (cannot filtrate in place)
 * - source and destination signals must be same length
 * - If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_src source signal vector
 * @param filter_kernel filter kernel coefficients (ascending power of z)
 * @param vector_dst destination signal vector
 * @return system control library error code
 */

return_code vector_real_filtrate(const vector_generic_T* vector_src, const vector_generic_T* filter_kernel, vector_generic_T* vector_dst) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(filter_kernel);
	if (returnval != return_OK)
		return returnval;
#endif

	int signal_length = vector_length(vector_src);
	int kernel_length = vector_length(filter_kernel);

	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, signal_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_src) != vector_length(vector_dst))
		return return_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < signal_length; n++) {
		real_t sum = 0;
		for (int k = 0; k < kernel_length; k++) {
			int i = NEG_CLIP(n - kernel_length + 1 + k);
			sum += vector_type_at(vector_src, real_t,i) * vector_type_at(filter_kernel, real_t, k);
		}
		vector_type_at(vector_dst,real_t,n) = sum;
	}

	return return_OK;
}

//TODO test filtrate

/**
 * Filtrates real signal vector with symmetric FIR filter kernel.
 * Performs forward and backward finite impulse response digital filtration of signal.
 * - source and destination signals must have different data pointer (cannot filtrate in place)
 * - source and destination signals must be same length
 * - If destination vector is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param vector_src source signal vector
 * @param filter_kernel filter kernel coefficients (ascending power of z - from negative to positive), kernel length must be odd
 * @param vector_dst destination signal vector
 * @return system control library error code
 */
return_code vector_real_filtrate_symetric(const vector_generic_T* vector_src, const vector_generic_T* filter_kernel, vector_generic_T* vector_dst) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_src);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(filter_kernel);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (vector_length(filter_kernel) % 2 == 0)
		return return_WRONG_DIMENSIONS;
#endif
	int signal_length = vector_length(vector_src);
	int kernel_length = (vector_length(filter_kernel) - 1) / 2;

	if (vector_assert(vector_dst) != return_OK) {
		returnval = vector_type_init(vector_dst, real_t, signal_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_src) != vector_length(vector_dst))
		return return_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < signal_length; n++) {
		real_t sum = 0;
		for (int k = -kernel_length; k <= kernel_length; k++) {
			int i = n + k;
			i = CLIP_BOTTOM(i, 0);
			i = CLIP_TOP(i, signal_length - 1);
			sum += vector_type_at(vector_src, real_t,i) * vector_type_at(filter_kernel, real_t, k + kernel_length);
		}
		vector_type_at(vector_dst,real_t,n) = sum;
	}
	return return_OK;
}
/**
 *
 * Initializes sampled signal structure.
 * Procedure similar to @ref vector_generic_init
 * @param signal_sampled sampled signal to initialize
 * @param length number of samples - vector length
 * @param Ts sample time
 * @param data_ptr pointer to signal data
 * @return system control library error code
 */

return_code signal_sampled_init(signal_sampled_T* signal_sampled, size_t length, real_t Ts, real_t* data_ptr) {
	return_code returnval = vector_type_init((vector_generic_T* ) signal_sampled, real_t, length, data_ptr);
	if (returnval != return_OK)
		return returnval;

	signal_sampled->Ts = Ts;
	signal_sampled->idx = 0;
	return return_OK;
}

/**
 * Resets sampled signal index and sets all elements to 0
 * @param signal_sampled sampled signal to reset
 * @return system control library error code
 */
return_code signal_sampled_reset(signal_sampled_T* signal_sampled) {
	return_code returnval = vector_type_set_all((vector_generic_T* ) signal_sampled, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	signal_sampled->idx = 0;

	return return_OK;
}

/**
 * Deinitializes sampled signal structure.
 * Procedure similar to @ref vector_generic_deinit
 * @param signal_sampled sampled signal to initialize
 * @return system control library error code
 */
return_code signal_sampled_deinit(signal_sampled_T* signal_sampled) {
	return_code returnval = vector_generic_deinit((vector_generic_T*) signal_sampled);
	if (returnval != return_OK)
		return returnval;
	signal_sampled->idx = 0;
	signal_sampled->Ts = 0;
	return return_OK;
}
/**
 * Samples value to sampled signal vector.
 * If sampled signal is full returns return_INDEX_OUT_OF_RANGE
 * @param signal_sampled sampled signal to sample
 * @param value pointer to value to sample
 * @return system control library error code
 */
return_code signal_sampled_sample_value(signal_sampled_T* signal_sampled, real_t value) {
	return_code returnval = vector_type_set_value_safe((vector_generic_T* ) signal_sampled, real_t, signal_sampled->idx, value)
	;
	if (returnval == return_OK)
		signal_sampled->idx++;
	return returnval;
}

/**
 * Initializes timeseries signal structure.
 * Initializes both values and time vectors.
 * Procedure similar to @ref vector_generic_init
 *
 * @param signal_timeseries timeseries to initialize
 * @param length number of timeseries elements
 * @param values_ptr pointer to values vector data
 * @param time_ptr pointer to time vector data
 * @return system control library error code
 */
return_code signal_timeseries_init(signal_timeseries_T* signal_timeseries, size_t length, real_t* values_ptr, real_t* time_ptr) {
	return_code returnval = return_OK;
	returnval = vector_type_init(&signal_timeseries->values_vector, real_t, length, values_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&signal_timeseries->time_vector, real_t, length, time_ptr);
	if (returnval != return_OK)
		return returnval;

	signal_timeseries->idx = 0;
	return return_OK;
}
/**
 * Deinitializes timeseries signal structure.
 * Deinitializes both values and time vectors
 * Procedure similar to @ref vector_generic_deinit
 *
 * @param signal_timeseries timeseries to deinitialize
 * @return system control library error code
 */
return_code signal_timeseries_deinit(signal_timeseries_T* signal_timeseries) {
	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&signal_timeseries->time_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&signal_timeseries->values_vector);
	if (returnval != return_OK)
		return returnval;
	signal_timeseries->idx = 0;
	return return_OK;
}
/**
 * Loads data to values and time vector of timeseries.
 * Resets index
 * @see vector_generic_load
 * @param signal_timeseries timeseries to be loaded
 * @param values_ptr pointer to values data
 * @param time_ptr pointer to time data
 * @return system control library error code
 */

return_code signal_timeseries_load_data(signal_timeseries_T* signal_timeseries, const real_t* values_ptr, const real_t* time_ptr) {

	return_code returnval = return_OK;
	returnval = vector_type_load(&signal_timeseries->values_vector, values_ptr);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_type_load(&signal_timeseries->time_vector, time_ptr);
	if (returnval != return_OK)
		return returnval;
	signal_timeseries->idx = 0;
	return return_OK;

}

/**
 * Resets timeseries index and sets values and time vector elements to 0.
 * @param signal_timeseries timeseries to reset
 * @return system control library error code
 */
return_code signal_timeseries_reset(signal_timeseries_T* signal_timeseries) {
	return_code returnval = return_OK;
	returnval = vector_type_set_all(&signal_timeseries->values_vector, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_set_all(&signal_timeseries->time_vector, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	signal_timeseries->idx = 0;
	return return_OK;
}
/**
 * Writes value to timeseries.
 * If time is out of range returns return_INDEX_OUT_OF_RANGE
 * @param signal_timeseries timeseries to write
 * @param time time to write value at
 * @param value value to write
 * @return system control library error code
 */
return_code signal_timeseries_write(signal_timeseries_T* signal_timeseries, real_t time, real_t value) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR

	returnval = vector_assert(&signal_timeseries->time_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&signal_timeseries->values_vector);
	if (returnval != return_OK)
		return returnval;
#endif
	if (time > vector_type_at(&signal_timeseries->time_vector, real_t, signal_timeseries->idx)) {
		if (signal_timeseries->idx + 1 >= vector_length(&signal_timeseries->time_vector))
			return return_INDEX_OUT_OF_RANGE;
		signal_timeseries->idx++;
	} else if (time < vector_type_at(&signal_timeseries->time_vector, real_t, signal_timeseries->idx)) {
		signal_timeseries->idx = 0;
	}
	vector_type_at(&signal_timeseries->time_vector,real_t,signal_timeseries->idx) = time;
	vector_type_at(&signal_timeseries->values_vector,real_t,signal_timeseries->idx) = value;
	return return_OK;

}
/**
 * Reads value from timeseries.
 * If time is out of range returns return_INDEX_OUT_OF_RANGE
 * @param signal_timeseries timeseries to read
 * @param time time to read value at
 * @param value_ptr  pointer to result value
 * @return system control library error code
 */
return_code signal_timeseries_read(signal_timeseries_T* signal_timeseries, real_t time, real_t* value_ptr) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (value_ptr == NULL)
		return return_NULLPTR;
	returnval = vector_assert(&signal_timeseries->time_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&signal_timeseries->values_vector);
	if (returnval != return_OK)
		return returnval;
#endif
	while (1) {
		if (signal_timeseries->idx > 0) {
			real_t time_n_left = vector_type_at(&signal_timeseries->time_vector, real_t, signal_timeseries->idx - 1);
			if (time_n_left > time) {
				signal_timeseries->idx--;
				continue;
			}
		}
		if (signal_timeseries->idx < vector_length(&signal_timeseries->time_vector) - 1) {
			real_t time_n_right = vector_type_at(&signal_timeseries->time_vector, real_t, signal_timeseries->idx + 1);
			if (time_n_right <= time) {
				signal_timeseries->idx++;
				continue;
			}
		}
		break;
	}
	*value_ptr = vector_type_at(&signal_timeseries->values_vector, real_t, signal_timeseries->idx);
	return return_OK;
}

