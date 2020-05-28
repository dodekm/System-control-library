#include "system_identification.h"
#include "gsl_wrapper.h"
#include "signal_process.h"

#ifdef USE_GSL

/**
 * Single run least squares fitting for generic linear system.
 *
 * Output of model is linear to parameters - y=X*theta.
 * Requires GSL - function is wrapper for gsl_multifit_linear.
 *
 * > The best-fit is found by singular value decomposition of the matrix X using the modified Golub-Reinsch SVD
 * > algorithm, with column scaling to improve the accuracy of the singular values. Any components which have
 * > zero singular value (to machine precision) are discarded from the fit.
 *
 * @param X_mat regression matrix - predictor variables (y_length X params_length)
 * @param y_vector measured system outputs vector
 * @param params_vector destination vector of estimated parameters
 * @param chisq destination pointer to estimate error (can be NULL)
 * @return system control library error code
 */
return_code system_ident_generic_linear_regression(const matrix_T* X_mat, const vector_generic_T* y_vector, vector_generic_T* params_vector, real_t* chisq) {

	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(y_vector);
	if (returnval != return_OK)
	return returnval;
	returnval = vector_assert(params_vector);
	if (returnval != return_OK)
	return returnval;
	returnval = matrix_assert(X_mat);
	if (returnval != return_OK)
	return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (vector_length(y_vector) != X_mat->n_rows)
	return return_WRONG_DIMENSIONS;
	if (vector_length(params_vector) != X_mat->n_cols)
	return return_WRONG_DIMENSIONS;
#endif

	size_t n_params = vector_length(params_vector);
	size_t n_observ = vector_length(y_vector);

	gsl_vector c = vector_real_to_gsl_vector(params_vector);
	const gsl_vector y = vector_real_to_gsl_vector(y_vector);
	const gsl_matrix X_mat_gsl = matrix_to_gsl_matrix(X_mat);

	gsl_matrix* cov = NULL;
	gsl_multifit_linear_workspace * work = NULL;

	cov = gsl_matrix_alloc(n_params, n_params);
	if (cov == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	work = gsl_multifit_linear_alloc(n_observ, n_params);
	if (work == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_multifit_linear(&X_mat_gsl, &y, &c, cov, chisq, work));
	ret: gsl_multifit_linear_free(work);
	gsl_matrix_free(cov);

	return returnval;

}
/**
 * Polynomial fitting.
 * Estimates coefficients of polynomial, fitting input values x and output values y.
 * Requires GSL - @see system_ident_generic_linear_regression
 *
 * @param x_vector vector of polynomial function inputs
 * @param y_vector vector of polynomial function outputs
 * @param polynom destination estimated polynomial structure pointer (can be uninitialized)
 * @param degree degree of fitted polynomial
 * @param chisq destination pointer to estimate error (can be NULL)
 * @return system control library error code
 */
return_code system_ident_polynomial_fit(const vector_generic_T* x_vector, const vector_generic_T* y_vector, polynom_T* polynom, size_t degree, real_t* chisq) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(x_vector);
	if (returnval != return_OK)
	return returnval;
	returnval = vector_assert(y_vector);
	if (returnval != return_OK)
	return returnval;

#endif

	if (vector_assert(polynom) != return_OK) {
		returnval = vector_type_init(polynom, real_t, degree + 1, NULL);
		if (returnval != return_OK)
		return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(x_vector) != vector_length(y_vector))
	return return_WRONG_DIMENSIONS;
	if (vector_length(polynom) != degree + 1)
	return return_WRONG_DIMENSIONS;
#endif

	size_t n_params = degree + 1;
	size_t n_observ = vector_length(x_vector);
	matrix_T X_mat = {0};

	returnval = matrix_init(&X_mat, n_observ, n_params, NULL);
	if (returnval != return_OK)
	return returnval;

	for (uint i = 0; i < n_observ; i++) {
		real_t x_pow = 1;
		real_t x_i = vector_type_at(x_vector, real_t, i);
		for (uint j = 0; j < n_params; j++) {
			matrix_at(&X_mat,i,j)=x_pow;
			x_pow*=x_i;
		}
	}

	returnval = system_ident_generic_linear_regression(&X_mat, y_vector, polynom, chisq);
	matrix_deinit(&X_mat);
	return returnval;
}

/**
 * Single run least squares method for discrete transfer function parameters estimation.
 * Requires GSL.
 * Creates and fills regression matrix.
 * Processes estimated parameters vector (theta) and extracts numerator and denominator part.
 *
 * @see system_ident_generic_linear_regression
 * @param u_vector vector of system inputs
 * @param y_vector vector of system outputs
 * @param numerator estimated numerator coefficients vector pointer
 * @param denominator estimated denominator coefficients vector pointer
 * @param nb order of numerator
 * @param na order of denominator
 * @param chisq destination pointer to estimate error (can be NULL)
 * @return system control library error code
 */
return_code system_ident_discrete_transfer_function(const vector_generic_T* u_vector, const vector_generic_T* y_vector, polynom_T* numerator, polynom_T* denominator, size_t nb, size_t na, real_t* chisq) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(u_vector);
	if (returnval != return_OK)
	return returnval;
	returnval = vector_assert(y_vector);
	if (returnval != return_OK)
	return returnval;
#endif

	if (vector_assert(numerator) != return_OK) {
		returnval = vector_type_init(numerator, real_t, nb + 1, NULL);
		if (returnval != return_OK)
		return returnval;
	}

	if (vector_assert(denominator) != return_OK) {
		returnval = vector_type_init(denominator, real_t, na + 1, NULL);
		if (returnval != return_OK)
		return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(u_vector) != vector_length(y_vector))
	return return_WRONG_DIMENSIONS;
	if (vector_length(numerator) != nb + 1)
	return return_WRONG_DIMENSIONS;
	if (vector_length(denominator) != na + 1)
	return return_WRONG_DIMENSIONS;
#endif

	size_t n_params = nb + na;
	size_t n_observ = vector_length(y_vector);
	matrix_T X_mat = {0};

	returnval = matrix_init(&X_mat, n_observ - 1, n_params, NULL);
	if (returnval != return_OK)
	return returnval;

	for (int i = 1; i < (int)n_observ; i++) {
		for (int j = 0; j < (int)nb; j++) {
			if (i - (int) nb + j < 0)
			matrix_at(&X_mat,i-1,j)=0;
			else
			matrix_at(&X_mat,i-1,j)=vector_type_at(u_vector,real_t,i-(int)nb+j);
		}
		for (int j = 0; j < (int)na; j++) {
			if(i-(int)na+j<0)
			matrix_at(&X_mat,i-1,j+(int)nb)=0;
			else
			matrix_at(&X_mat,i-1,j+(int)nb)=-vector_type_at(y_vector,real_t,i-(int)na+j);
		}
	}
	vector_generic_T theta = {0};
	returnval = vector_type_init(&theta, real_t, n_params, NULL);
	if (returnval != return_OK)
	goto ret;

	vector_generic_T y_vector_cut = {0};
	vector_subvector(&y_vector_cut, y_vector, n_observ - 1, 1);

	returnval = system_ident_generic_linear_regression(&X_mat, &y_vector_cut, &theta, chisq);

	vector_generic_T numerator_subvector = {0};
	vector_subvector(&numerator_subvector, numerator, nb, 0);
	vector_generic_T denominator_subvector = {0};
	vector_subvector(&denominator_subvector, denominator, na, 0);
	vector_generic_T theta_numerator = {0};
	vector_subvector(&theta_numerator, &theta, nb, 0);
	vector_generic_T theta_denominator = {0};
	vector_subvector(&theta_denominator, &theta, na, nb);
	vector_generic_copy(&numerator_subvector, &theta_numerator);
	vector_generic_copy(&denominator_subvector, &theta_denominator);

	vector_type_at(numerator,real_t,nb) = 0;
	vector_type_at(denominator,real_t,na) = 1;

	ret: matrix_deinit(&X_mat);
	vector_generic_deinit(&theta);
	return returnval;
}

#endif
/**
 * Initializes recursive least squares algorithm and its' sub-objects.
 * Performs pre-allocation of required objects depending on number of estimated parameters.
 * Sets dispersion matrix as diagonal
 * @param RLS recursive least squares instance pointer
 * @param n_params number of estimated parameters
 * @param lambda forgetting factor <0,1>
 * @param P0_initial_values initial diagonal values of dispersion matrix P (e.g. 10^10)
 * @return system control library error code
 */
return_code system_ident_recursive_least_squares_init(system_ident_recursive_least_squares_T* RLS, size_t n_params, real_t lambda, real_t P0_initial_values) {


	return_code returnval = return_OK;
	returnval = matrix_init(&RLS->Pk_0, n_params, n_params, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_init(&RLS->Pk_1, n_params, n_params, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_init(&RLS->YhT, n_params, n_params, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&RLS->theta, real_t, n_params, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&RLS->Y, real_t, n_params, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_diagonal_operation(&RLS->Pk_0, &RLS->Pk_0, P0_initial_values, real_sum);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_diagonal_operation(&RLS->Pk_1, &RLS->Pk_1, P0_initial_values, real_sum);
	if (returnval != return_OK)
		return returnval;
	RLS->lambda = lambda;
	RLS->error = 0;

	return return_OK;
}

/**
 * Recursive least squares instance reset.
 * Sets all sub-objects values to zero.
 * Sets dispersion matrix as diagonal
 * @param RLS recursive least squares instance pointer
 * @param P0_initial_values initial diagonal values of dispersion matrix P (e.g. 10^10)
 * @return system control library error code
 */
return_code system_ident_recursive_least_squares_reset(system_ident_recursive_least_squares_T* RLS, real_t P0_initial_values) {

	return_code returnval = return_OK;
	returnval = matrix_set_all_elements(&RLS->Pk_0, 0);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_set_all_elements(&RLS->Pk_1, 0);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_diagonal_operation(&RLS->Pk_0, &RLS->Pk_0, P0_initial_values, real_sum);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_diagonal_operation(&RLS->Pk_1, &RLS->Pk_1, P0_initial_values, real_sum);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_set_all_elements(&RLS->YhT, 0);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_set_all(&RLS->theta, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_set_all(&RLS->Y, real_t, 0)
	;
	if (returnval != return_OK)
		return returnval;
	RLS->error = 0;
	return return_OK;

}

/**
 * Recursive least squares instance de-initialization.
 * De-initializes all sub-objects
 * @param RLS recursive least squares instance pointer
 * @return system control library error code
 */
return_code system_ident_recursive_least_squares_deinit(system_ident_recursive_least_squares_T* RLS) {

	return_code returnval = return_OK;
	returnval = matrix_deinit(&RLS->Pk_0);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_deinit(&RLS->Pk_1);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_deinit(&RLS->YhT);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&RLS->theta);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&RLS->Y);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}

/**
 * Recursive least squares algorithm instance iteration.
 * Updates dispersion matrix and estimated parameters vector.
 * GSL/BLAS support for linear operations.
 * Function does not use dynamic allocation.
 * Function uses only basic linear algebra operations (not including matrix inversion)
 * @param RLS recursive least squares instance pointer
 * @param hk_1 regression vector
 * @param yk_1 measured output value
 * @return system control library error code
 */
return_code system_ident_recursive_least_squares_estimate_generic(system_ident_recursive_least_squares_T* RLS, const vector_generic_T* hk_1, real_t yk_1) {

	return_code returnval = return_OK;
#ifdef  ASSERT_NULLPTR

	returnval = vector_assert(hk_1);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef  ASSERT_DIMENSIONS
	if (vector_length(hk_1) != vector_length(&RLS->theta))
		return return_WRONG_DIMENSIONS;
#endif
#ifdef USE_GSL
	const gsl_vector hk_1_gsl = vector_real_to_gsl_vector(hk_1);
	gsl_vector theta = vector_real_to_gsl_vector(&RLS->theta);
	gsl_vector Y = vector_real_to_gsl_vector(&RLS->Y);
	gsl_matrix Pk_0 = matrix_to_gsl_matrix(&RLS->Pk_0);
	gsl_matrix Pk_1 = matrix_to_gsl_matrix(&RLS->Pk_1);
	gsl_matrix YhT = matrix_to_gsl_matrix(&RLS->YhT);
	real_t lambda = RLS->lambda;

	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&Pk_0, &Pk_1)); //Pk_0=Pk_1
	if (returnval != return_OK)
	return returnval;

	real_t y_estimated_k_1 = 0;
	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&hk_1_gsl, &theta, &y_estimated_k_1));//y_estimated_k_1=hk_1_T*theta
	if (returnval != return_OK)
	return returnval;

	real_t error_k_1 = yk_1 - y_estimated_k_1;
	RLS->error = error_k_1;

	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1, &Pk_0, &hk_1_gsl, 0, &Y));//Pk_0*hk_1
	if (returnval != return_OK)
	return returnval;
	real_t rho = 0;

	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&hk_1_gsl, &Y, &rho));//rho=hk_1_T*Pk_0*hk_1
	if (returnval != return_OK)
	return returnval;
	rho += lambda;
	returnval = gsl_error_code_to_return_code(gsl_vector_scale(&Y, 1.0 / rho));
	if (returnval != return_OK)
	return returnval;

	gsl_matrix Yk_1_mat = gsl_matrix_view_vector(&Y, Y.size, 1).matrix;
	const gsl_matrix hk_1_T_mat = gsl_matrix_const_view_vector(&hk_1_gsl, 1, hk_1_gsl.size).matrix;
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0 / lambda, &Yk_1_mat, &hk_1_T_mat, 0, &YhT));//YhT=Yk_1*hk_1_T
	if (returnval != return_OK)
	return returnval;

	returnval = gsl_error_code_to_return_code(gsl_matrix_add_diagonal(&YhT, 1.0 / lambda));
	if (returnval != return_OK)
	return returnval;
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, &YhT, &Pk_0, 0, &Pk_1));
	if (returnval != return_OK)
	return returnval;

	returnval = gsl_error_code_to_return_code(gsl_blas_daxpy(error_k_1, &Y, &theta));//theta_k1=theta_k0+e*Y;
	if (returnval != return_OK)
	return returnval;

#else
	vector_generic_T* theta = &RLS->theta;
	vector_generic_T* Y = &RLS->Y;
	matrix_T* Pk_0 = &RLS->Pk_0;
	matrix_T* Pk_1 = &RLS->Pk_1;
	matrix_T* YhT = &RLS->YhT;
	real_t lambda = RLS->lambda;

	returnval = matrix_copy(Pk_0, Pk_1);
	if (returnval != return_OK)
		return returnval;

	real_t y_estimated_k_1 = 0;
	returnval = vector_real_dot_product(hk_1, theta, &y_estimated_k_1);
	if (returnval != return_OK)
		return returnval;
	real_t error_k_1 = yk_1 - y_estimated_k_1;
	RLS->error = error_k_1;

	matrix_T hk_1_col_mat = vector_real_to_col_matrix(hk_1);
	matrix_T Yk_1_mat = vector_real_to_col_matrix(Y);
	returnval = matrix_multiply(Pk_0, &hk_1_col_mat, &Yk_1_mat);
	if (returnval != return_OK)
		return returnval;
	real_t rho = 0;
	returnval = vector_real_dot_product(hk_1, Y, &rho);
	if (returnval != return_OK)
		return returnval;
	rho += lambda;
	returnval = vector_real_scalar_operator(Y, Y, rho, real_div);
	if (returnval != return_OK)
		return returnval;

	matrix_T hk_1_T_mat = vector_real_to_row_matrix(hk_1);
	returnval = matrix_multiply(&Yk_1_mat, &hk_1_T_mat, YhT);
	if (returnval != return_OK)
		return returnval;

	returnval = matrix_diagonal_operation(YhT, YhT, 1.0, real_sub);
	if (returnval != return_OK)
		return returnval;

	returnval = matrix_scalar_operation(YhT, YhT, -lambda, real_div);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_multiply(YhT, Pk_0, Pk_1);
	if (returnval != return_OK)
		return returnval;

	returnval = vector_real_scalar_operator(Y, Y, error_k_1, real_mul);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_real_element_binary_operator(theta, Y, theta, real_sum);
	if (returnval != return_OK)
		return returnval;
#endif
	return return_OK;

}

/**
 * Iterates regression vector of discrete transfer function with new input u_k1 measure y_k1.
 * Shifts history states of system input and output signals
 *
 * @param hk_1 regression vector
 * @param u_k1 transfer function input value
 * @param y_k1 measured transfer function output value
 * @param nb order of numerator
 * @param na order of denominator
 * @return system control library error code
 */
return_code system_ident_transfer_function_vector_h_iterate(vector_generic_T* hk_1, real_t u_k1, real_t y_k1, size_t nb, size_t na) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(hk_1);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t n_params = nb + na;
#ifdef ASSERT_DIMENSIONS
	if (vector_length(hk_1) != n_params)
		return return_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < nb - 1; i++) {
		vector_type_at(hk_1,real_t,i) = vector_type_at(hk_1, real_t, i + 1);
	}
	vector_type_at(hk_1,real_t,nb-1) = u_k1;

	for (uint i = nb; i < nb + na - 1; i++) {
		vector_type_at(hk_1,real_t,i) = vector_type_at(hk_1, real_t, i + 1);
	}
	vector_type_at(hk_1,real_t,nb+na-1) = -y_k1;

	return return_OK;
}

/**
 * Estimates discrete transfer function parameters - numerator and denominator coefficients using recursive least squares method.
 * Wrapped call of @ref system_ident_recursive_least_squares_estimate_generic.
 * Processes estimated parameters vector(theta) and extracts numerator and denominator part.
 * Regression vector of discrete transfer function must be iterated individually after calling this function
 *
 * @param RLS recursive least squares instance pointer
 * @param hk_1 regression vector
 * @param yk_1 measured transfer function output value
 * @param numerator transfer function estimated numerator coefficients vector
 * @param denominator transfer function estimated denominator coefficients vector
 * @return system control library error code
 */
return_code system_ident_recursive_least_squares_estimate_discrete_transfer_function(system_ident_recursive_least_squares_T* RLS, const vector_generic_T* hk_1, real_t yk_1, polynom_T* numerator, polynom_T* denominator) {

	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(hk_1);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(numerator);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(denominator);
	if (returnval != return_OK)
		return returnval;
#endif

	size_t nb = vector_length(numerator) - 1;
	size_t na = vector_length(denominator) - 1;
	size_t n_params = nb + na;
#ifdef ASSERT_DIMENSIONS
	if (vector_length(hk_1) != n_params)
		return return_WRONG_DIMENSIONS;
#endif
	returnval = system_ident_recursive_least_squares_estimate_generic(RLS, hk_1, yk_1);
	if (returnval != return_OK)
		return returnval;

	vector_generic_T* theta =&RLS->theta;

	vector_generic_T numerator_subvector = { 0 };
	vector_subvector(&numerator_subvector, numerator, nb, 0);
	vector_generic_T denominator_subvector = { 0 };
	vector_subvector(&denominator_subvector, denominator, na, 0);
	vector_generic_T theta_numerator = { 0 };
	vector_subvector(&theta_numerator, theta, nb, 0);
	vector_generic_T theta_denominator = { 0 };
	vector_subvector(&theta_denominator, theta, na, nb);
	vector_generic_copy(&numerator_subvector, &theta_numerator);
	vector_generic_copy(&denominator_subvector, &theta_denominator);

	vector_type_at(numerator,real_t,nb) = 0;
	vector_type_at(denominator,real_t,na) = 1;

	return return_OK;

}

