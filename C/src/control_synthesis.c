
#include "control_synthesis.h"
#include "gsl_wrapper.h"

extern complex_t convert_continuous_root_to_discrete_root(complex_t, real_t);
extern return_code vector_real_sum(const vector_generic_T*, real_t* );

/**
 * Function creates convolution matrix of kernel vector g.
 * @param g convolution kernel vector
 * @param mat destination convolution matrix, dimensions: (f_length + g_length - 1) x f_length
 * @param f_length expected length of vector f
 * @return system control library error code
 */

return_code control_synthesis_convolution_matrix(const vector_generic_T* g, matrix_T* mat, uint f_length) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(g);
	if (returnval != return_OK)
		return returnval;
#endif

	int g_length = (int) vector_length(g);
	int f_g_length = f_length + g_length - 1;
	if (matrix_assert(mat) != return_OK) {
		returnval = matrix_init(mat, f_g_length, f_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(g, real_t))
		return return_WRONG_DIMENSIONS;
	if (mat->n_cols != f_length)
		return return_WRONG_DIMENSIONS;
	if (mat->n_rows != f_g_length)
		return return_WRONG_DIMENSIONS;
#endif

	for (int i = 0; i < f_g_length; i++) {
		for (int j = 0; j < MAX(f_length, g_length); j++) {
			if (i - j >= 0 && i - j < (int) f_length) {
				if (j < g_length)
					matrix_at(mat,i,i-j)=vector_type_at(g,real_t,j);
					else
					matrix_at(mat,i,i-j)=0;
				}
			}
		}
	return return_OK;
}

#ifdef USE_GSL

/**
 * RST polynomial regulator synthesis using pole-placement method.
 * Perform parameters synthesis of RST regulator linear equations system based on convolution matrices of controlled system numerator and denominator polynomials and desired polynomial on right side
 * @param A transfer function denominator coefficients vector pointer
 * @param B transfer function numerator coefficients vector pointer
 * @param P desired closed loop characteristic polynomial pointer
 * @param R destination R polynomial pointer
 * @param S destination S polynomial pointer
 * @param T destination polynomial pointer
 * @return system control library error code
 */
return_code control_synthesis_RST_poleplace(const polynom_T* A,const polynom_T* B,const polynom_T* P, polynom_T* R, polynom_T* S, polynom_T* T) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	returnval = vector_assert(A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(B);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(P);
	if (returnval != return_OK)
		return returnval;
#endif

	int P_length = vector_length(P);
	if (P_length != vector_length(A) + vector_length(B) - 1)
		return return_WRONG_DIMENSIONS;
	int R_length = vector_length(B);
	int S_length = vector_length(A) - 1;

	if (vector_assert(R) != return_OK) {
		returnval = vector_type_init(R, real_t, R_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}
	if (vector_assert(S) != return_OK) {
		returnval = vector_type_init(S, real_t, S_length, NULL);
		if (returnval != return_OK)
			return returnval;
	}

	if (vector_assert(T) != return_OK) {
		returnval = vector_type_init(T, real_t, 1, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(A, real_t))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(B, real_t))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(P, real_t))
		return return_WRONG_DIMENSIONS;
	if (vector_length(R) != R_length)
		return return_WRONG_DIMENSIONS;
	if (vector_length(S) != S_length)
		return return_WRONG_DIMENSIONS;
	if (vector_length(T) != 1)
		return return_WRONG_DIMENSIONS;
#endif

	matrix_T M = { 0 };
	returnval = matrix_init(&M, P_length, P_length, NULL);
	if (returnval != return_OK)
		return returnval;
	matrix_T M_submatrix_A = { 0 };
	matrix_T M_submatrix_B = { 0 };

	matrix_submatrix(&M_submatrix_A, &M, P_length, R_length, 0, 0);
	matrix_submatrix(&M_submatrix_B, &M, P_length - 1, S_length, 1, R_length);

	control_synthesis_convolution_matrix(A, &M_submatrix_A, R_length);
	control_synthesis_convolution_matrix(B, &M_submatrix_B, S_length);

	gsl_permutation* perm_gsl = NULL;
	gsl_vector* X_gsl = NULL;

	gsl_matrix M_gsl = matrix_to_gsl_matrix(&M);
	gsl_vector P_gsl = vector_real_to_gsl_vector(P);

	perm_gsl = gsl_permutation_alloc(P_length);
	if (perm_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	X_gsl = gsl_vector_alloc(P_length);
	if (X_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}

	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(&M_gsl, perm_gsl, NULL));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_solve(&M_gsl, perm_gsl, &P_gsl, X_gsl));
	if (returnval != return_OK)
		goto ret;

	vector_generic_load(R, X_gsl->data);
	vector_generic_load(S, X_gsl->data + R_length);

	real_t sum_P = 0;
	real_t sum_B = 0;
	vector_real_sum(P, &sum_P);
	vector_real_sum(B, &sum_B);
	vector_type_at(T,real_t,0) = sum_P / sum_B;

	ret: matrix_deinit(&M);
	gsl_permutation_free(perm_gsl);
	gsl_vector_free(X_gsl);
	return returnval;
}
#endif

/**
 * Creates discrete desired polynomial P from vector of continuous poles.
 * Uses @ref convert_continuous_root_to_discrete_root and @ref polynom_from_roots
 * @param poles_cont continuous poles complex vector
 * @param P desired closed loop characteristic polynomial pointer
 * @param Ts sample time
 * @return system control library error code
 */
return_code control_synthesis_create_discrete_polynom_from_continuous_poles(const vector_generic_T* poles_cont, polynom_T* P, real_t Ts) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(poles_cont);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(poles_cont, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	vector_generic_T poles_disc = { 0 };
	returnval = vector_type_init(&poles_disc, complex_t, vector_length(poles_cont), NULL);
	if (returnval != return_OK)
		return returnval;
	for (uint i = 0; i < vector_length(poles_cont); i++) {
		vector_type_at(&poles_disc,complex_t,i) = convert_continuous_root_to_discrete_root(vector_type_at(poles_cont, complex_t, i), Ts);
	}
	returnval = polynom_from_roots(P, &poles_disc);
	if (returnval != return_OK)
		goto ret;
	ret: vector_generic_deinit(&poles_disc);
	return returnval;
}
/**
 * Creates discrete aperiodic desired polynomial P from multiple continuous time constant.
 * Uses @ref convert_continuous_root_to_discrete_root and @ref control_synthesis_create_discrete_polynom_from_multiple_complex_root
 * @param P desired closed loop characteristic polynomial pointer
 * @param multiplicity
 * @param Tc continuous time constant
 * @param Ts sample time
 * @return system control library error code
 */
return_code control_synthesis_create_discrete_aperiodic_polynom_for_multiple_time_constant(polynom_T* P, uint multiplicity, real_t Tc, real_t Ts) {
	complex_t cont_root = { -1 / Tc, 0 };
	complex_t disc_root = convert_continuous_root_to_discrete_root(cont_root, Ts);
	return control_synthesis_create_discrete_polynom_from_multiple_complex_root(P, multiplicity, disc_root);

}

/**
 * Creates discrete desired polynomial P from multiple discrete complex root.
 * Uses @ref polynom_from_roots
 * @param P desired closed loop characteristic polynomial pointer
 * @param multiplicity multiplicity of root
 * @param root discrete complex root
 * @return system control library error code
 */
return_code control_synthesis_create_discrete_polynom_from_multiple_complex_root(polynom_T* P, uint multiplicity, complex_t root) {

	return_code returnval = return_OK;
	vector_generic_T roots_vector = { 0 };
	returnval = vector_type_init(&roots_vector, complex_t, multiplicity, NULL);
	if (returnval != return_OK)
		return returnval;
	for (uint i = 0; i < multiplicity; i++) {
		if (i % 2)
			vector_type_at(&roots_vector,complex_t,i) = root;
		else
			vector_type_at(&roots_vector,complex_t,i) = complex_conj(root);
	}
	returnval = polynom_from_roots(P, &roots_vector);
	vector_generic_deinit(&roots_vector);
	return returnval;
}

/**
 * Pole-placement method for PI/IP regulator and first order transfer function.
 * @param system_params parameters of controlled system
 * @param desired_polynom desired closed loop second order system characteristic polynomial pointer
 * @param regulator_params destination PI/IP regulator parameters pointer
 * @return system control library error code
 */
return_code control_synthesis_PI_poleplace(const control_synthesis_first_order_system_params_T* system_params,const control_synthesis_desired_polynom_2nd_order_T* desired_polynom, PID_regulator_params_T* regulator_params) {

#ifdef ASSERT_NULLPTR
	if (system_params == NULL || desired_polynom == NULL || regulator_params == NULL)
		return return_NULLPTR;
#endif

	regulator_params->P_gain = (2.0 * desired_polynom->b * desired_polynom->omega_0 * system_params->T - 1.0) / system_params->K;
	regulator_params->I_gain = POW2(desired_polynom->omega_0) * system_params->T / system_params->K;
	return return_OK;

}
/**
 * Pole-placement method for PIV (P+IP) regulator and first order astatic transfer function.
 * @param system_params parameters of controlled system
 * @param desired_polynom desired closed loop third order system characteristic polynomial pointer
 * @param IV_regulator_params destination IP regulator parameters pointer
 * @param P_regulator_params destination P regulator parameters pointer
 * @return system control library error code
 */
return_code control_synthesis_PIV_poleplace(const control_synthesis_first_order_system_params_T* system_params,const control_synthesis_desired_polynom_3rd_order_T* desired_polynom, PID_regulator_params_T* IV_regulator_params, PID_regulator_params_T* P_regulator_params) {
#ifdef ASSERT_NULLPTR
	if (system_params == NULL || desired_polynom == NULL || IV_regulator_params == NULL || P_regulator_params == NULL)
		return return_NULLPTR;
#endif
	IV_regulator_params->P_gain = ((2.0 * desired_polynom->b * desired_polynom->omega_0 + desired_polynom->k) * system_params->T - 1.0) / system_params->K;
	IV_regulator_params->I_gain = (POW2(desired_polynom->omega_0) + 2.0 * desired_polynom->b * desired_polynom->omega_0 * desired_polynom->k) * system_params->T / system_params->K;
	P_regulator_params->P_gain = (POW2(desired_polynom->omega_0) * desired_polynom->k) / (POW2(desired_polynom->omega_0) + 2.0 * desired_polynom->b * desired_polynom->omega_0 * desired_polynom->k);

	return return_OK;
}

