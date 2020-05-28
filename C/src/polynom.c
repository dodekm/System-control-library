#include "polynom.h"
#include "signal_process.h"
#include "gsl_wrapper.h"

/**
 * Evaluation of polynomial.
 * Uses Horner scheme to evaluate polynomial for real argument x.
 * Function has GSL support
 *
 * @param polynom polynomial to evaluate
 * @param x value to evaluate
 * @param y_ptr pointer to result
 * @return system control library error code
 *
 * @ingroup examples
 * # Example
 *
 * 		 real_t polynom_coeffs[] = {-3.0,1.0,2.0,-4.0,1.5};
 *	     polynom_T polynom = { 0 };
 *	     vector_init_from_array(&polynom, polynom_coeffs);
 *	     real_t y=0;
 *	     polynom_eval(&polynom,2.5,&y);
 *
*/

return_code polynom_eval(const polynom_T* polynom, real_t x, real_t* y_ptr) {
	return_code returnval = return_OK;
	if (y_ptr == NULL)
		return return_NULLPTR;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(polynom, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length((vector_generic_T* ) polynom);
	real_t y = 0;
#ifdef USE_GSL
	y = gsl_poly_eval(vector_type_data_ptr((vector_generic_T* ) polynom, real_t), length, x);
#else
	real_t x_pow = 1.0;
	for (uint i = 0; i < length; i++) {
		y += vector_type_at((vector_generic_T*) polynom,real_t,i) * x_pow;
		x_pow *= x;
	}
#endif
	*y_ptr = y;
	return return_OK;
}


/**
 * Evaluation of polynomial.
 * Uses Horner scheme to evaluate polynomial for complex argument.
 * Function has GSL support
 * @param polynom polynomial to evaluate
 * @param x value to evaluate
 * @param y_ptr pointer to result
 * @return system control library error code
 */
return_code polynom_eval_complex(const polynom_T* polynom, complex_t x, complex_t* y_ptr) {

	return_code returnval = return_OK;
	if (y_ptr == NULL)
		return return_NULLPTR;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(polynom, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length((vector_generic_T* ) polynom);
	complex_t y = { 0, 0 };
#ifdef USE_GSL
	y = gsl_complex_to_complex(gsl_poly_complex_eval(vector_type_data_ptr((vector_generic_T* ) polynom, real_t), length, complex_to_gsl_complex(x)));
#else
	complex_t x_pow = {1.0, 0.0};
	for (uint i = 0; i < length; i++) {
		complex_t a = {vector_type_at((vector_generic_T* ) polynom, real_t, i), 0};
		y = complex_add(y, complex_mul(x_pow, a));
		x_pow = complex_mul(x_pow, x);
	}
#endif
	*y_ptr = y;
	return return_OK;
}
/**
 * Polynomial multiplication.
 * C(x)=A(x)*B(x)
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Uses discrete vector convolution.
 * @see vector_real_convolute, @see vector_complex_convolute
 * @param polynom_A polynomial A
 * @param polynom_B polynomial B
 * @param polynom_Dst polynomial C
 * @return system control library error code
 *
 * @ingroup examples
 * # Example
 *
 *	     real_t polynom_P_coeffs[] = { -3.0, 1.0, 2.0, -4.0, 1.5 };
 *       polynom_T polynom_P = { 0 };
 *       vector_init_from_array(&polynom_P, polynom_P_coeffs);
 *       real_t polynom_Q_coeffs[] = { 2.0,2.0,-1.0,3.0 };
 *       polynom_T polynom_Q = { 0 };
 *       vector_init_from_array(&polynom_Q, polynom_Q_coeffs);
 *       polynom_T polynom_R = { 0 };
 *       polynom_mul(&polynom_P,&polynom_Q,&polynom_R);
 *
 */
return_code polynom_mul(const polynom_T* polynom_A, const polynom_T* polynom_B, polynom_T* polynom_Dst) {
	if (vector_is_type(polynom_A,real_t) && vector_is_type(polynom_B, real_t))
		return vector_real_convolute(polynom_A, polynom_B, polynom_Dst);
	else if (vector_is_type(polynom_A,complex_t) && vector_is_type(polynom_B, complex_t))
		return vector_complex_convolute(polynom_A, polynom_B, polynom_Dst);
	else
		return return_ERROR;
}
/**
 * Polynomial addition.
 * Sum coefficients of polynomial.
 * Works also if A and B are different degree.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * @param polynom_A polynomial A
 * @param polynom_B polynomial B
 * @param polynom_Dst polynomial C
 * @return system control library error code
 */
return_code polynom_add(const polynom_T* polynom_A, const polynom_T* polynom_B, polynom_T* polynom_Dst) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert((vector_generic_T*) polynom_B);
	if (returnval != return_OK)
		return returnval;

#endif
	size_t length = MAX(vector_length(polynom_A), vector_length(polynom_B));
	if (vector_assert(polynom_Dst) != return_OK) {
		returnval = vector_type_init(polynom_Dst, real_t, length, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(polynom_Dst) != length)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom_A, real_t) || !vector_is_type(polynom_B, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	for (uint i = 0; i < length; i++) {
		real_t x = 0;
		if (i < vector_length(polynom_A))
			x += vector_type_at((vector_generic_T* ) polynom_A, real_t, i);
		if (i < vector_length(polynom_B))
			x += vector_type_at((vector_generic_T* ) polynom_B, real_t, i);
		vector_type_at((vector_generic_T*) polynom_Dst,real_t,i) = x;
	}
	return return_OK;
}
/**
 * Power of real coefficients polynomial.
 * P(x)^n.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * @param polynom_base source polynomial
 * @param polynom_pow destination polynomial
 * @param power power of polynomial
 * @return system control library error code
 */
return_code polynom_pow_real(const polynom_T* polynom_base, polynom_T* polynom_pow, uint power) {

	if (power == 0)
		return return_OK;
	return_code returnval;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom_base);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t polynom_length = power * (vector_length(polynom_base) - 1) + 1;
	if (vector_assert(polynom_pow) != return_OK) {
		returnval = vector_type_init(polynom_pow, real_t, polynom_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(polynom_pow) != polynom_length)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom_base, real_t) || !vector_is_type(polynom_pow, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	polynom_T polynom_n0_pow = { 0 };
	polynom_T polynom_n1_pow = { 0 };
	returnval = vector_type_init(&polynom_n0_pow, real_t, 1, NULL);
	if (returnval != return_OK)
		return returnval;
	vector_type_at(&polynom_n0_pow,real_t,0) = 1;
	for (uint i = 0; i < power; i++) {
		vector_generic_deinit(&polynom_n1_pow);
		returnval = polynom_mul(&polynom_n0_pow, polynom_base, &polynom_n1_pow);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&polynom_n0_pow, &polynom_n1_pow);
		if (returnval != return_OK)
			goto ret;
	}
	returnval = vector_generic_copy(polynom_pow, &polynom_n0_pow);
	ret: vector_generic_deinit(&polynom_n0_pow);
	vector_generic_deinit(&polynom_n1_pow);
	return returnval;

}
/**
 * Power of complex coefficients polynomial.
 * P(x)^n.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * @param polynom_base source polynomial
 * @param polynom_pow destination polynomial
 * @param power power of polynomial
 * @return system control library error code
 */

return_code polynom_pow_complex(const polynom_T* polynom_base, polynom_T* polynom_pow, uint power) {
	if (power < 1)
		return return_OK;
	return_code returnval;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom_base);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t polynom_length = power * (vector_length(polynom_base) - 1) + 1;
	if (vector_assert(polynom_pow) != return_OK) {
		returnval = vector_type_init(polynom_pow, complex_t, polynom_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(polynom_pow) != polynom_length)
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom_base, complex_t) || !vector_is_type(polynom_pow, complex_t))
		return return_WRONG_DIMENSIONS;
#endif
	polynom_T polynom_n0_pow = { 0 };
	polynom_T polynom_n1_pow = { 0 };
	returnval = vector_type_init(&polynom_n0_pow, complex_t, 1, NULL);
	if (returnval != return_OK)
		return returnval;
	vector_type_at(&polynom_n0_pow,complex_t,0).real = 1;
	vector_type_at(&polynom_n0_pow,complex_t,0).imag = 0;
	for (uint i = 0; i < power; i++) {
		vector_generic_deinit(&polynom_n1_pow);
		returnval = polynom_mul(&polynom_n0_pow, polynom_base, &polynom_n1_pow);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&polynom_n0_pow, &polynom_n1_pow);
		if (returnval != return_OK)
			goto ret;
	}
	returnval = vector_generic_copy(polynom_pow, &polynom_n0_pow);
	ret: vector_generic_deinit(&polynom_n0_pow);
	vector_generic_deinit(&polynom_n1_pow);
	return returnval;
}

/**
 * Creates polynomial from given roots.
 * Polynomial degree equals number of roots.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Function uses dynamic allocation for temporal vector
 * @param polynom destination polynomial
 * @param roots vector of complex roots
 * @return system control library error code
 */

return_code polynom_from_roots(polynom_T* polynom, const vector_generic_T* roots) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(roots);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(roots, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	size_t n_roots = vector_length(roots);

	polynom_T polynom_i0_stage = { 0 };
	polynom_T polynom_i1_stage = { 0 };
	returnval = vector_type_init(&polynom_i0_stage, complex_t, 1, NULL);
	if (returnval != return_OK)
		return returnval;
	vector_type_at(&polynom_i0_stage,complex_t,0).real = 1;
	vector_type_at(&polynom_i0_stage,complex_t,0).imag = 0;

	for (uint i = 0; i < n_roots; i++) {
		complex_t polynom_root_coeffs[2] = { 0 };

		polynom_root_coeffs[0].real = -vector_type_at(roots, complex_t, i).real;
		polynom_root_coeffs[0].imag = -vector_type_at(roots, complex_t, i).imag;
		polynom_root_coeffs[1].real = 1;
		polynom_root_coeffs[1].imag = 0;

		if (isnan(polynom_root_coeffs[0].real) || isnan(polynom_root_coeffs[0].imag) || isinf(polynom_root_coeffs[0].real) || isinf(polynom_root_coeffs[0].imag))
			continue;
		polynom_T polynom_root = { 0 };
		returnval = vector_type_init(&polynom_root, complex_t, 2, polynom_root_coeffs);
		if (returnval != return_OK)
			goto ret;
		vector_generic_deinit(&polynom_i1_stage);
		returnval = polynom_mul(&polynom_i0_stage, &polynom_root, &polynom_i1_stage);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&polynom_i0_stage, &polynom_i1_stage);
		if (returnval != return_OK)
			goto ret;

	}
	if (vector_assert(polynom) != return_OK) {
		returnval = vector_type_init(polynom, real_t, vector_length(&polynom_i0_stage), NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (vector_length(polynom) != vector_length(&polynom_i0_stage))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom, real_t))
		return return_WRONG_DIMENSIONS;
#endif

	returnval = vector_complex_to_real_unary_operator(&polynom_i0_stage, polynom, complex_real_part);
	if (returnval != return_OK)
		goto ret;
	ret: vector_generic_deinit(&polynom_i0_stage);
	vector_generic_deinit(&polynom_i1_stage);
	return returnval;
}
/**
 * Polynomial roots finding.
 * Finds complex roots of polynomial using Durand-Kerner method.
 * Number of roots equals polynomial degree.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions
 * @param polynom polynomial to find roots of
 * @param roots destination complex roots
 * @param space  roots search space (default @ref POLYNOM_ROOTS_FIND_INITIAL_SPACE_SIZE)
 * @param tolerance convergence satisfaction tolerance  (default @ref POLYNOM_ROOTS_FIND_TOLERANCE)
 * @param max_iterations maximum number of iterations (default @ref POLYNOM_ROOTS_FIND_N_ITER)
 * @return system control library error code
 *
 */

return_code polynom_find_roots_DurandKerner(const polynom_T* polynom, vector_generic_T* roots, real_t space, real_t tolerance, uint max_iterations) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom);
	if (returnval != return_OK)
		return returnval;
#endif
	uint n_roots = vector_length(polynom) - 1;
	if (vector_assert(roots) != return_OK) {
		returnval = vector_type_init(roots, complex_t, n_roots, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (n_roots != vector_length(roots))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom, real_t) || !vector_is_type(roots, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	if (space == 0.0)
		space = POLYNOM_ROOTS_FIND_INITIAL_SPACE_SIZE;
	if (max_iterations == 0)
		max_iterations = POLYNOM_ROOTS_FIND_N_ITER;
	if (tolerance == 0.0)
		tolerance = POLYNOM_ROOTS_FIND_TOLERANCE;
	for (uint n = 0; n < n_roots; n++) {
		vector_type_at(roots,complex_t,n) = (complex_t){RANDF(-space, space),RANDF(-space, space)};
	}

	for (uint i = 0; i < max_iterations; i++) {
		bool_t found_all = bool_true;
		for (uint n = 0; n < n_roots; n++) {
			complex_t den = { 1, 0 };
			for (uint k = 0; k < n_roots; k++) {
				if (k != n)
					den = complex_mul(den, complex_sub(vector_type_at(roots, complex_t, n), vector_type_at(roots, complex_t, k)));
			}
			complex_t f_x = { 0, 0 };
			polynom_eval_complex(polynom, vector_type_at(roots, complex_t, n), &f_x);
			complex_t new_root = complex_sub(vector_type_at(roots, complex_t, n), complex_div(f_x, den));

			if (complex_magnitude(complex_sub(new_root, vector_type_at(roots, complex_t, n))) > tolerance) {
				found_all = bool_false;
				vector_type_at(roots,complex_t,n) = new_root;
			}
		}
		if (found_all)
			return return_OK;
	}

	return return_OK;
}

/**
 * Polynomial roots finding.
 * Number of roots equals polynomial degree.
 * If destination polynomial is uninitialized - function performs dynamic initialization with appropriate dimensions.
 * Wrapper for GSL gsl_poly_complex_solve.
 * Function uses dynamic allocation for solver workspace.
 * Requires GSL
 * @param polynom polynomial to find roots of
 * @param roots destination complex roots
 * @return system control library error code
 */

#ifdef USE_GSL
return_code polynom_find_roots_GSL(const polynom_T* polynom, vector_generic_T* roots) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert((vector_generic_T*) polynom);
	if (returnval != return_OK)
		return returnval;
#endif
	uint n_roots = vector_length(polynom) - 1;
	if (vector_assert(roots) != return_OK) {
		returnval = vector_type_init(roots, complex_t, n_roots, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (n_roots != vector_length(roots))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(polynom, real_t) || !vector_is_type(roots, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc(n_roots + 1);
	if (w == NULL)
		return return_NULLPTR;
	returnval = gsl_error_code_to_return_code(gsl_poly_complex_solve((double*) vector_generic_get_data_ptr(polynom), n_roots + 1, w, (double*) vector_generic_get_data_ptr(roots)));
	gsl_poly_complex_workspace_free(w);
	return returnval;
}
#endif

/**
 * General polynomial roots find.
 * Uses function depending on the USE_GSL switch.
 * @param polynom polynomial to find roots of
 * @param roots destination complex roots
 * @return system control library error code
 */
return_code polynom_find_roots(const polynom_T* polynom, vector_generic_T* roots) {

#ifdef USE_GSL
	return polynom_find_roots_GSL(polynom, roots);
#else
	return polynom_find_roots_DurandKerner(polynom, roots, POLYNOM_ROOTS_FIND_INITIAL_SPACE_SIZE, POLYNOM_ROOTS_FIND_TOLERANCE, POLYNOM_ROOTS_FIND_N_ITER);
#endif

}
/**
 * Quadratic equation solver.
 * Function computes analytical roots for general second degree polynomial
 * @param polynom coeffs of quadratic polynomial
 * @return complex conjungated roots of polynomial
 */
roots_quadratic_T polynom_quadratic_roots(polynom_quadratic_T polynom) {
	roots_quadratic_T roots = { 0 };
	complex_t D_sqrt = { 0 };
	real_t D = 0;
	real_t a, b, c;
	a = polynom.coeffs[2];
	b = polynom.coeffs[1];
	c = polynom.coeffs[0];

	D = b * b - 4 * a * c;
	if (D < 0) {
		D_sqrt.imag = sqrt(-D);
		D_sqrt.real = 0;
	} else {
		D_sqrt.real = sqrt(D);
		D_sqrt.imag = 0;
	}
	roots.roots[0].real = (-b + D_sqrt.real) / 2 / a;
	roots.roots[0].imag = (D_sqrt.imag) / 2 / a;
	roots.roots[1].real = (-b - D_sqrt.real) / 2 / a;
	roots.roots[1].imag = (-D_sqrt.imag) / 2 / a;
	return roots;
}

