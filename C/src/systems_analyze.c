#include "systems_analyze.h"
#include "gsl_wrapper.h"

/**
 * Converts continuous root (zero or pole) to discrete.
 * Uses complex exponential
 * @param S_root continuous complex root
 * @param Ts sample time
 * @return discrete complex root
 */
complex_t convert_continuous_root_to_discrete_root(complex_t S_root, real_t Ts) {
	S_root.real *= Ts;
	S_root.imag *= Ts;
	complex_t Z_root = complex_exponential(S_root);
	return Z_root;
}
/**
 * Converts discrete root (zero or pole) to continuous.
 * Uses complex logarithm
 *
 * @param Z_root discrete complex root
 * @param Ts sample time
 * @return continuous complex root
 */
complex_t convert_discrete_root_to_continuous_root(complex_t Z_root, real_t Ts) {

	complex_t S_root = complex_log(Z_root);
	S_root.real /= Ts;
	S_root.imag /= Ts;
	return S_root;
}

/**
 * Analyzes stability of complex roots (poles) of system depending on time domain.
 * @param roots system poles complex vector
 * @param time_domain system time domain (continuous/discrete)
 * @return true/false
 */
bool_t analyze_roots_are_stable(const vector_generic_T* roots, system_time_domain time_domain) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(roots);
	if (returnval != return_OK)
		return bool_error;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(roots, complex_t))
		return bool_error;
#endif
	for (uint i = 0; i < vector_length(roots); i++) {
		complex_t pole = vector_type_at(roots, complex_t, i);
		if (time_domain == system_time_domain_continuous) {
			if (complex_real_part(pole) > 0)
				return bool_false;
		} else if (time_domain == system_time_domain_discrete) {
			if (complex_magnitude(pole) > 1)
				return bool_false;
		}
	}
	return bool_true;
}

/**
 * Analyzes static gain of transfer function depending on its' time domain.
 * @param transfer_function transfer function structure pointer
 * @param gain_ptr destination gain pointer
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code analyze_transfer_function_gain(const transfer_function_T* transfer_function, real_t* gain_ptr, system_time_domain time_domain) {

	const polynom_T* numerator = &transfer_function->numerator_coeffs;
	const polynom_T* denominator = &transfer_function->denominator_coeffs;
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (gain_ptr == NULL)
		return return_NULLPTR;
#endif
	returnval = transfer_function_assert(transfer_function);
	if (returnval != return_OK)
		return returnval;

	if (time_domain == system_time_domain_continuous) {
		*gain_ptr = vector_type_at(numerator,real_t,0) / vector_type_at(denominator, real_t, 0);
	} else if (time_domain == system_time_domain_discrete) {
		real_t num_sum = 0;
		real_t den_sum = 0;
		vector_real_sum(numerator, &num_sum);
		vector_real_sum(denominator, &den_sum);
		*gain_ptr = num_sum / den_sum;
	} else
		return return_ERROR;

	return return_OK;
}

/**
 * Sets static gain of transfer function depending on its' time domain.
 * Factorizes numerator coefficients
 * @param transfer_function transfer function structure pointer
 * @param gain gain value
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_transfer_function_set_gain(transfer_function_T* transfer_function, real_t gain, system_time_domain time_domain) {

	return_code returnval = return_OK;
	polynom_T* numerator = &transfer_function->numerator_coeffs;
	real_t gain_old = 0;
	returnval = analyze_transfer_function_gain(transfer_function, &gain_old, time_domain);
	if (returnval != return_OK)
		return returnval;
	real_t k = gain / gain_old;
	returnval = vector_real_scalar_operator(numerator, numerator, k, real_mul);
	if (returnval != return_OK)
		return returnval;

	return return_OK;

}

/**
 * Normalizes transfer function numerator and denominator coefficients for unit highest degree of denominator.
 * @param transfer_function transfer function structure pointer
 * @return system control library error code
 */
return_code convert_transfer_function_normalize(transfer_function_T* transfer_function) {
	return_code returnval = return_OK;

	polynom_T* numerator = &transfer_function->numerator_coeffs;
	polynom_T* denominator = &transfer_function->denominator_coeffs;

	returnval = transfer_function_assert(transfer_function);
	if (returnval != return_OK)
		return returnval;

	real_t k = vector_type_at(denominator, real_t, vector_length(denominator)-1);
	vector_real_scalar_operator(numerator, numerator, k, real_div);
	vector_real_scalar_operator(denominator, denominator, k, real_div);

	return return_OK;
}

/**
 * Analyzes stability (denominator roots location) of transfer function depending on its' time domain.
 * @param transfer_function transfer function structure pointer
 * @param time_domain system time domain (continuous/discrete)
 * @return true/false
 */
bool_t analyze_transfer_function_is_stable(const transfer_function_T* transfer_function, system_time_domain time_domain) {
	return_code returnval = return_OK;
	const polynom_T* denominator = &transfer_function->denominator_coeffs;
	returnval = transfer_function_assert(transfer_function);
	if (returnval != return_OK)
		return returnval;
	if (vector_length(denominator) == 1)
		return bool_true;

	vector_generic_T poles = { 0 };
	bool_t is_stable = bool_error;
	returnval = polynom_find_roots(denominator, &poles);
	if (returnval != return_OK)
		goto ret;
	is_stable = analyze_roots_are_stable(&poles, time_domain);
	ret: vector_generic_deinit(&poles);
	return is_stable;

}

/**
 * Converts transfer function to state space canonical form representation.
 * @param transfer_function transfer function structure pointer (source)
 * @param state_space state space structure pointer (destination)
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_transfer_function_to_state_space(const transfer_function_T* transfer_function, state_space_T* state_space, system_time_domain time_domain) {

	const polynom_T* numerator = &transfer_function->numerator_coeffs;
	const polynom_T* denominator = &transfer_function->denominator_coeffs;
	matrix_T* A = &state_space->A;
	vector_generic_T* B = &state_space->B;
	vector_generic_T* C = &state_space->C;

	return_code returnval = return_OK;
	returnval = transfer_function_assert(transfer_function);
	if (returnval != return_OK)
		return returnval;

	size_t order = vector_length(denominator) - 1;

	returnval = state_space_assert(state_space);
	if (returnval != return_OK) {
		state_space_deinit(state_space);
		returnval = state_space_init(state_space, order, NULL, NULL, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(numerator) >= vector_length(denominator))
		return return_WRONG_DIMENSIONS;
	if (A->n_rows != order)
		return return_WRONG_DIMENSIONS;

#endif

	for (uint i = 0; i < order; i++) {
		for (uint j = 0; j < order; j++) {
			if (i == order - 1) {
				matrix_at(A,i,j)=-vector_type_at(denominator,real_t,j)/vector_type_at(denominator,real_t,order);
			}
			else
			{
				if (j == i + 1)
				matrix_at(A,i,j)=1;
				else
				matrix_at(A,i,j)=0;
			}
		}
		if (i == order - 1)
		vector_type_at(B,real_t,i)=1;
		else
		vector_type_at(B,real_t,i)=0;
		if (i < vector_length(numerator))
		vector_type_at(C,real_t,i)=vector_type_at(numerator,real_t,i)/vector_type_at(denominator,real_t,order);
		else
		vector_type_at(C,real_t,i)=0;
	}
	return return_OK;
}

/**
 * Analyzes stability (poles location) of zeros-poles-gain representation depending on its' time domain.
 * @param zpk zeros-poles-gain structure pointer
 * @param time_domain system time domain (continuous/discrete)
 * @return true/false
 */
bool_t analyze_zero_pole_is_stable(const zeros_poles_gain_T* zpk, system_time_domain time_domain) {
	return analyze_roots_are_stable(&zpk->poles, time_domain);
}

/**
 * Converts zeros-poles-gain representation to transfer function.
 * @see polynom_from_roots
 * @see convert_transfer_function_set_gain
 * @param zpk zeros-poles-gain structure pointer (source)
 * @param transfer_function transfer function structure pointer (destination)
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_zero_pole_gain_to_transfer_function(const zeros_poles_gain_T* zpk, transfer_function_T* transfer_function, system_time_domain time_domain) {

	const vector_generic_T* zeros = &zpk->zeros;
	const vector_generic_T* poles = &zpk->poles;

	polynom_T* numerator = &transfer_function->numerator_coeffs;
	polynom_T* denominator = &transfer_function->denominator_coeffs;

	return_code returnval = return_OK;

	returnval = polynom_from_roots(numerator, zeros);
	if (returnval != return_OK)
		return returnval;
	returnval = polynom_from_roots(denominator, poles);
	if (returnval != return_OK)
		return returnval;
	returnval = convert_transfer_function_set_gain(transfer_function, zpk->gain, time_domain);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}
/**
 * Converts transfer function to zeros-poles-gain representation.
 * @see polynom_find_roots
 * @see analyze_transfer_function_gain
 * @param transfer_function transfer function structure pointer (source)
 * @param zpk zeros-poles-gain structure pointer (destination)
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_transfer_function_to_zeros_poles_gain(const transfer_function_T* transfer_function, zeros_poles_gain_T* zpk, system_time_domain time_domain) {

	const polynom_T* numerator = &transfer_function->numerator_coeffs;
	const polynom_T* denominator = &transfer_function->denominator_coeffs;

	vector_generic_T* zeros = &zpk->zeros;
	vector_generic_T* poles = &zpk->poles;

	return_code returnval = return_OK;

	returnval = analyze_transfer_function_gain(transfer_function, &zpk->gain, time_domain);
	if (returnval != return_OK)
		return returnval;
	returnval = polynom_find_roots(numerator, zeros);
	if (returnval != return_OK)
		return returnval;
	returnval = polynom_find_roots(denominator, poles);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}
/**
 * Converts continuous zeros-poles-gain representation to discrete zeros-poles-gain representation.
 * @see convert_continuous_root_to_discrete_root
 * @param zpk_cont continuous zeros-poles-gain structure pointer (source)
 * @param zpk_disc discrete zeros-poles-gain structure pointer (destination)
 * @param Ts sample time
 * @return system control library error code
 */
return_code convert_zeros_poles_gain_continuous_to_discrete(const zeros_poles_gain_T* zpk_cont, zeros_poles_gain_T* zpk_disc, real_t Ts) {

	const vector_generic_T* zeros_cont = &zpk_cont->zeros;
	const vector_generic_T* poles_cont = &zpk_cont->poles;
	vector_generic_T* zeros_disc = &zpk_disc->zeros;
	vector_generic_T* poles_disc = &zpk_disc->poles;

	if (Ts <= 0)
		return return_ERROR;
	return_code returnval = return_OK;
	returnval = zeros_poles_gain_assert(zpk_cont);
	if (returnval != return_OK)
		return returnval;

	returnval = zeros_poles_gain_assert(zpk_disc);
	if (returnval != return_OK) {
		zeros_poles_gain_deinit(zpk_disc);
		returnval = zeros_poles_gain_init(zpk_disc, vector_length(zeros_cont), NULL, vector_length(poles_cont), NULL, 0);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(zeros_disc) != vector_length(zeros_cont))
		return return_WRONG_DIMENSIONS;
	if (vector_length(poles_disc) != vector_length(poles_cont))
		return return_WRONG_DIMENSIONS;
#endif
	size_t n_zeros = vector_length(zeros_cont);
	for (uint i = 0; i < n_zeros; i++) {
		vector_type_at(zeros_disc,complex_t,i) = convert_continuous_root_to_discrete_root(vector_type_at(zeros_cont, complex_t, i), Ts);
	}
	size_t n_poles = vector_length(poles_cont);
	for (uint i = 0; i < n_poles; i++) {
		vector_type_at(poles_disc,complex_t,i) = convert_continuous_root_to_discrete_root(vector_type_at(poles_cont, complex_t, i), Ts);
	}
	zpk_disc->gain = zpk_cont->gain;
	return return_OK;
}
/**
 * Converts continuous transfer function to discrete transfer function using Forward Euler method.
 * @param tf_cont continuous transfer function structure pointer (source)
 * @param tf_disc discrete transfer function structure pointer (destination)
 * @param Ts sample time
 * @return system control library error code
 */
return_code convert_transfer_function_continuous_to_discrete_forward_Euler(const transfer_function_T* tf_cont, transfer_function_T* tf_disc, real_t Ts) {

	const polynom_T* numerator_cont = &tf_cont->numerator_coeffs;
	const polynom_T* denominator_cont = &tf_cont->denominator_coeffs;
	polynom_T* numerator_disc = &tf_disc->numerator_coeffs;
	polynom_T* denominator_disc = &tf_disc->denominator_coeffs;

	if (Ts <= 0)
		return return_ERROR;
	return_code returnval = return_OK;
	returnval = transfer_function_assert(tf_cont);
	if (returnval != return_OK)
		return returnval;

	returnval = transfer_function_assert(tf_disc);
	if (returnval != return_OK) {
		transfer_function_deinit(tf_disc);
		returnval = transfer_function_init(tf_disc, vector_length(numerator_cont), vector_length(denominator_cont), NULL, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(numerator_cont) != vector_length(denominator_cont)) //FIXME num/den must have same length
		return return_WRONG_DIMENSIONS;
	if (vector_length(numerator_cont) != vector_length(numerator_disc))
		return return_WRONG_DIMENSIONS;
	if (vector_length(denominator_cont) != vector_length(denominator_disc))
		return return_WRONG_DIMENSIONS;
#endif

	polynom_T aprox_polynom_i_0_deg = { 0 };
	polynom_T aprox_polynom_i_1_deg = { 0 };
	polynom_T aprox_polynom_base = { 0 };
	const real_t aprox_polynom_data[] = { -1.0 / Ts, 1.0 / Ts, };
	vector_type_init(&aprox_polynom_base, real_t, 2, aprox_polynom_data);

	returnval = vector_type_init(&aprox_polynom_i_0_deg, real_t, 1, NULL);
	if (returnval != return_OK)
		return returnval;
	vector_type_at(&aprox_polynom_i_0_deg,real_t,0) = 1.0;
	vector_type_at(numerator_disc,real_t,0) = vector_type_at(numerator_cont, real_t, 0);
	vector_type_at(denominator_disc,real_t,0) = vector_type_at(denominator_cont, real_t, 0);

	for (uint i = 1; i < vector_length(denominator_cont); i++) {
		vector_generic_deinit(&aprox_polynom_i_1_deg);

		returnval = polynom_mul(&aprox_polynom_i_0_deg, &aprox_polynom_base, &aprox_polynom_i_1_deg);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&aprox_polynom_i_0_deg, &aprox_polynom_i_1_deg);
		if (returnval != return_OK)
			goto ret;

		returnval = vector_real_scalar_operator(&aprox_polynom_i_0_deg, &aprox_polynom_i_0_deg, vector_type_at(numerator_cont, real_t, i), real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = polynom_add(&aprox_polynom_i_0_deg, numerator_disc, numerator_disc);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&aprox_polynom_i_0_deg, &aprox_polynom_i_1_deg);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_real_scalar_operator(&aprox_polynom_i_0_deg, &aprox_polynom_i_0_deg, vector_type_at(denominator_cont, real_t, i), real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = polynom_add(&aprox_polynom_i_0_deg, denominator_disc, denominator_disc);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_generic_copy(&aprox_polynom_i_0_deg, &aprox_polynom_i_1_deg);
		if (returnval != return_OK)
			goto ret;

	}
	convert_transfer_function_normalize(tf_disc);
	ret: vector_generic_deinit(&aprox_polynom_i_0_deg);
	vector_generic_deinit(&aprox_polynom_i_1_deg);

	return returnval;

}

/**
 * Converts continuous transfer function to discrete transfer function using bilinear transform (Tustin).
 * @param tf_cont continuous transfer function structure pointer (source)
 * @param tf_disc discrete transfer function structure pointer (destination)
 * @param Ts sample time
 * @return system control library error code
 */
return_code convert_transfer_function_continuous_to_discrete_bilinear(const transfer_function_T* tf_cont, transfer_function_T* tf_disc, real_t Ts) {

	const polynom_T* numerator_cont = &tf_cont->numerator_coeffs;
	const polynom_T* denominator_cont = &tf_cont->denominator_coeffs;
	polynom_T* numerator_disc = &tf_disc->numerator_coeffs;
	polynom_T* denominator_disc = &tf_disc->denominator_coeffs;
	if (Ts <= 0)
		return return_ERROR;
	return_code returnval = return_OK;
	returnval = transfer_function_assert(tf_cont);
	if (returnval != return_OK)
		return returnval;

	returnval = transfer_function_assert(tf_disc);
	if (returnval != return_OK) {
		transfer_function_deinit(tf_disc);
		returnval = transfer_function_init(tf_disc, vector_length(numerator_cont), vector_length(denominator_cont), NULL, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (vector_length(numerator_cont) != vector_length(denominator_cont)) //FIXME num/den must have same length
		return return_WRONG_DIMENSIONS;
	if (vector_length(numerator_cont) != vector_length(numerator_disc))
		return return_WRONG_DIMENSIONS;
	if (vector_length(denominator_cont) != vector_length(denominator_disc))
		return return_WRONG_DIMENSIONS;
#endif

	polynom_T aprox_polynom_numerator = { 0 };
	const real_t aprox_polynom_numerator_data[] = { -2.0 / Ts, 2.0 / Ts };
	polynom_T aprox_polynom_denominator = { 0 };
	const real_t aprox_polynom_denominator_data[] = { 1.0, 1.0 };
	vector_type_init(&aprox_polynom_numerator, real_t, 2, aprox_polynom_numerator_data);
	vector_type_init(&aprox_polynom_denominator, real_t, 2, aprox_polynom_denominator_data);
	size_t order = vector_length(denominator_cont) - 1;
	polynom_T aprox_polynom_i_0_deg = { 0 };
	polynom_T aprox_polynom_i_1_deg = { 0 };

	for (uint i = 0; i <= order; i++) {
		vector_generic_deinit(&aprox_polynom_i_0_deg);
		returnval = vector_type_init(&aprox_polynom_i_0_deg, real_t, 1, NULL);
		if (returnval != return_OK)
			goto ret;
		vector_type_at(&aprox_polynom_i_0_deg,real_t,0) = 1.0;
		vector_generic_deinit(&aprox_polynom_i_1_deg);
		returnval = vector_type_init(&aprox_polynom_i_1_deg, real_t, 1, NULL);
		if (returnval != return_OK)
			goto ret;
		vector_type_at(&aprox_polynom_i_1_deg,real_t,0) = 1.0;
		for (uint j = 0; j < i; j++) {
			vector_generic_deinit(&aprox_polynom_i_1_deg);
			returnval = polynom_mul(&aprox_polynom_i_0_deg, &aprox_polynom_numerator, &aprox_polynom_i_1_deg);
			if (returnval != return_OK)
				goto ret;
			returnval = vector_generic_copy(&aprox_polynom_i_0_deg, &aprox_polynom_i_1_deg);
			if (returnval != return_OK)
				goto ret;
		}
		for (uint j = i; j < order; j++) {
			vector_generic_deinit(&aprox_polynom_i_1_deg);
			returnval = polynom_mul(&aprox_polynom_i_0_deg, &aprox_polynom_denominator, &aprox_polynom_i_1_deg);
			if (returnval != return_OK)
				goto ret;
			returnval = vector_generic_copy(&aprox_polynom_i_0_deg, &aprox_polynom_i_1_deg);
			if (returnval != return_OK)
				goto ret;
		}

		returnval = vector_real_scalar_operator(&aprox_polynom_i_0_deg, &aprox_polynom_i_0_deg, vector_type_at(numerator_cont, real_t, i), real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = polynom_add(&aprox_polynom_i_0_deg, numerator_disc, numerator_disc);
		if (returnval != return_OK)
			goto ret;
		returnval = vector_real_scalar_operator(&aprox_polynom_i_1_deg, &aprox_polynom_i_1_deg, vector_type_at(denominator_cont, real_t, i), real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = polynom_add(&aprox_polynom_i_1_deg, denominator_disc, denominator_disc);
		if (returnval != return_OK)
			goto ret;

	}
	convert_transfer_function_normalize(tf_disc);
	ret: vector_generic_deinit(&aprox_polynom_i_0_deg);
	vector_generic_deinit(&aprox_polynom_i_1_deg);
	return returnval;
}
/**
 * Converts continuous transfer function to discrete transfer function using zero-pole matching method.
 * @param tf_cont continuous transfer function structure pointer (source)
 * @param tf_disc discrete transfer function structure pointer (destination)
 * @param Ts sample time
 * @return system control library error code
 */
return_code convert_transfer_function_continuous_to_discrete_match_zeros_poles(const transfer_function_T* tf_cont, transfer_function_T* tf_disc, real_t Ts) {

	return_code returnval = return_OK;
	zeros_poles_gain_T zpk = { 0 };

	returnval = convert_transfer_function_to_zeros_poles_gain(tf_cont, &zpk, system_time_domain_continuous);
	if (returnval != return_OK)
		goto ret;
	returnval = convert_zeros_poles_gain_continuous_to_discrete(&zpk, &zpk, Ts);
	if (returnval != return_OK)
		goto ret;
	returnval = convert_zero_pole_gain_to_transfer_function(&zpk, tf_disc, system_time_domain_discrete);
	if (returnval != return_OK)
		goto ret;
	convert_transfer_function_normalize(tf_disc);
	ret: zeros_poles_gain_deinit(&zpk);

	return returnval;
}

/**
 * Converts continuous state space system representation to discrete equivalent using Taylor's series approximation.
 * @param SS_cont continuous state space structure pointer (source)
 * @param SS_disc discrete state space structure pointer (destination)
 * @param Ts sample time
 * @param n series order (number of iterations)
 * @return system control library error code
 */
return_code convert_state_space_continuous_to_discrete_Taylor_series(const state_space_T* SS_cont, state_space_T* SS_disc, real_t Ts, uint n) {

	const matrix_T* A_cont = &SS_cont->A;
	const matrix_T B_cont =vector_real_to_col_matrix(&SS_cont->B);
	const vector_generic_T* C_cont = &SS_cont->C;
	matrix_T* A_disc = &SS_disc->A;
	matrix_T B_disc = vector_real_to_col_matrix(&SS_disc->B);
	vector_generic_T* C_disc = &SS_disc->C;

	if (Ts <= 0)
		return return_ERROR;
	return_code returnval = return_OK;
	returnval = state_space_assert(SS_cont);
	if (returnval != return_OK)
		return returnval;

	returnval = state_space_assert(SS_disc);
	if (returnval != return_OK) {
		state_space_deinit(SS_disc);
		returnval = state_space_init(SS_disc, A_cont->n_rows, NULL, NULL, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(A_cont, A_disc) || !matrix_equal_dimensions(&B_cont, &B_disc) || vector_length(C_cont)!=vector_length(C_disc) )
		return return_WRONG_DIMENSIONS;
#endif

	uint64_t n_factorial = 1;
	real_t Ts_power = 1;

	matrix_T A_cont_power_i_0 = { 0 };
	matrix_T A_cont_power_i_1 = { 0 };
	matrix_T A_cont_power_B_i_1 = { 0 };
	returnval = matrix_init(&A_cont_power_i_0, A_cont->n_rows, A_cont->n_cols, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_init(&A_cont_power_i_1, A_cont->n_rows, A_cont->n_cols, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_init(&A_cont_power_B_i_1, A_cont->n_rows, 1, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_set_identity(&A_cont_power_i_0);
	if (returnval != return_OK)
		goto ret;
	returnval = matrix_set_identity(&A_cont_power_i_1);
	if (returnval != return_OK)
		goto ret;
	for (uint i = 0; i < n; i++) {

		returnval = matrix_scalar_operation(&A_cont_power_i_1, &A_cont_power_i_1, Ts_power / n_factorial, real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_element_wise_operation(&A_cont_power_i_1, A_disc, A_disc, real_sum);
		if (returnval != return_OK)
			goto ret;
		n_factorial *= (i + 1);
		Ts_power *= Ts;

		returnval = matrix_multiply(&A_cont_power_i_0, &B_cont, &A_cont_power_B_i_1);
		if (returnval != return_OK)
			goto ret;

		returnval = matrix_element_wise_operation_and_scalar_operation(&A_cont_power_B_i_1, &B_disc, &B_disc, Ts_power / n_factorial, real_sum, real_mul);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_multiply(&A_cont_power_i_0, A_cont, &A_cont_power_i_1);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_copy(&A_cont_power_i_0, &A_cont_power_i_1);
		if (returnval != return_OK)
			goto ret;

	}
	returnval = vector_generic_copy(C_disc, C_cont);
	ret: matrix_deinit(&A_cont_power_i_0);
	matrix_deinit(&A_cont_power_i_1);
	matrix_deinit(&A_cont_power_B_i_1);

	return returnval;
}

#ifdef USE_GSL

/**
 * Analyzes static gain of state space system representation depending on it's time domain.
 * Requires GSL
 * @param state_space state space structure pointer
 * @param gain_ptr destination gain pointer
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code analyze_state_space_gain(const state_space_T* state_space, real_t* gain_ptr, system_time_domain time_domain) {

	return_code returnval = return_OK;
	const matrix_T* A = &state_space->A;
	const vector_generic_T* B = &state_space->B;
	const vector_generic_T* C = &state_space->C;

	returnval = state_space_assert(state_space);
	if (returnval != return_OK)
		return returnval;

	gsl_matrix* A_gsl = NULL;
	gsl_permutation* P_gsl = NULL;
	gsl_vector* X_gsl = NULL;

	size_t order = A->n_rows;
	A_gsl = matrix_to_gsl_matrix_dynamic_copy(A);
	if (A_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	const gsl_vector B_gsl =vector_real_to_gsl_vector(B);

	P_gsl = gsl_permutation_alloc(order);
	if (P_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	X_gsl = gsl_vector_alloc(order);
	if (X_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_matrix_scale(A_gsl, -1));
	if (returnval != return_OK)
		goto ret;
	if (time_domain == system_time_domain_discrete) {
		returnval = gsl_error_code_to_return_code(gsl_matrix_add_diagonal(A_gsl, 1));
		if (returnval != return_OK)
			goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(A_gsl, P_gsl, NULL));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_solve(A_gsl, P_gsl, &B_gsl, X_gsl));
	if (returnval != return_OK)
		goto ret;

	const gsl_vector C_gsl = vector_real_to_gsl_vector(C);
	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&C_gsl, X_gsl, gain_ptr));
	ret: gsl_permutation_free(P_gsl);
	gsl_vector_free(X_gsl);
	gsl_matrix_free(A_gsl);
	return returnval;
}

/**
 * Sets static gain of state space system representation.
 * Requires GSL
 * @param state_space  state space structure pointer
 * @param gain target gain value
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_state_space_set_gain(state_space_T* state_space, real_t gain, system_time_domain time_domain) {

	return_code returnval = return_OK;

	real_t gain_old = 0;
	returnval = analyze_state_space_gain(state_space, &gain_old, time_domain);
	if (returnval != return_OK)
		return returnval;
	real_t k = gain / gain_old;
	returnval = vector_real_scalar_operator(&state_space->C, &state_space->C, k, real_mul);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Converts continuous state space system representation to discrete using exact resolvent matrix exponential.
 * @param SS_cont continuous state space structure pointer (source)
 * @param SS_disc discrete state space structure pointer (destination)
 * @param Ts sample time
 * @return system control library error code
 */
return_code convert_state_space_continuous_to_discrete(const state_space_T* SS_cont, state_space_T* SS_disc, real_t Ts) {

	const matrix_T* A_cont = &SS_cont->A;
	const vector_generic_T* B_cont = &SS_cont->B;
	const vector_generic_T* C_cont = &SS_cont->C;
	matrix_T* A_disc = &SS_disc->A;
	vector_generic_T* B_disc = &SS_disc->B;
	vector_generic_T* C_disc = &SS_disc->C;

	if (Ts <= 0)
		return return_ERROR;
	return_code returnval = return_OK;

	returnval = state_space_assert(SS_cont);
	if (returnval != return_OK)
		return returnval;

	returnval = state_space_assert(SS_disc);
	if (returnval != return_OK) {
		state_space_deinit(SS_disc);
		returnval = state_space_init(SS_disc, A_cont->n_rows, NULL, NULL, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(A_cont, A_disc) || vector_length(B_cont)!=vector_length(B_disc)|| vector_length(C_cont)!=vector_length(C_disc) )
		return return_WRONG_DIMENSIONS;
#endif

	gsl_matrix* A_cont_gsl = NULL;
	gsl_matrix* A_cont_inv_gsl = NULL;
	gsl_vector* A_cont_inv_mul_B_cont = NULL;
	gsl_matrix* eAT_gsl = NULL;
	gsl_permutation* P_gsl = NULL;
	gsl_matrix A_disc_gsl = matrix_to_gsl_matrix(A_disc);
	gsl_vector B_disc_gsl = vector_real_to_gsl_vector(B_disc);
	const gsl_vector B_cont_gsl = vector_real_to_gsl_vector(B_cont);
	size_t order = A_cont->n_rows;

	A_cont_gsl = matrix_to_gsl_matrix_dynamic_copy(A_cont);
	if (A_cont_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	A_cont_inv_gsl = matrix_to_gsl_matrix_dynamic_copy(A_cont);
	if (A_cont_inv_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}

	eAT_gsl = gsl_matrix_alloc(order, order);
	if (eAT_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_matrix_scale(A_cont_gsl, Ts));
	if (returnval != return_OK)
		goto ret;
	gsl_matrix* AT_gsl = A_cont_gsl;
	returnval = gsl_error_code_to_return_code(gsl_linalg_exponential_ss(AT_gsl, &A_disc_gsl, GSL_MODE_DEFAULT));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(eAT_gsl, &A_disc_gsl));
	if (returnval != return_OK)
		goto ret;

	returnval = gsl_error_code_to_return_code(gsl_matrix_add_diagonal(eAT_gsl, -1));
	if (returnval != return_OK)
		goto ret;
	gsl_matrix* eAT_minus_diag = eAT_gsl;

	P_gsl = gsl_permutation_alloc(order);
	if (P_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(A_cont_inv_gsl, P_gsl, NULL));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_invx(A_cont_inv_gsl, P_gsl));
	if (returnval != return_OK)
		goto ret;

	A_cont_inv_mul_B_cont = gsl_vector_alloc(order);
	if (A_cont_inv_mul_B_cont == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}

	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans,1.0, A_cont_inv_gsl, &B_cont_gsl, 0, A_cont_inv_mul_B_cont));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans,1.0, eAT_minus_diag, A_cont_inv_mul_B_cont, 0, &B_disc_gsl));
	if (returnval != return_OK)
		goto ret;

	vector_generic_copy(C_disc, C_cont);

	ret: gsl_matrix_free(A_cont_gsl);
	gsl_matrix_free(A_cont_inv_gsl);
	gsl_matrix_free(eAT_gsl);
	gsl_vector_free(A_cont_inv_mul_B_cont);
	gsl_permutation_free(P_gsl);
	return returnval;
}

/**
 * Converts state space system representation to transfer function using interstage transformation to zeros-poles-gain representation.
 * @see convert_state_space_to_zero_pole_gain and
 * @see convert_zero_pole_gain_to_transfer_function
 * @param state_space state space structure pointer (source)
 * @param transfer_function transfer function structure pointer (destination)
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_state_space_to_transfer_function(const state_space_T* state_space, transfer_function_T* transfer_function, system_time_domain time_domain) {

	return_code returnval = return_OK;

	zeros_poles_gain_T zpk = { 0 };
	returnval = convert_state_space_to_zeros_poles_gain(state_space, &zpk, time_domain);
	if (returnval != return_OK)
		goto ret;
	returnval = convert_zero_pole_gain_to_transfer_function(&zpk, transfer_function, time_domain);

	ret: zeros_poles_gain_deinit(&zpk);
	return returnval;
}

/**
 * Converts state space system representation to zeros-poles-gain representation poles, zeros and agin analysis.
 * @see analyze_state_space_gain
 * @see analyze_state_space_poles
 * @see analyze_state_space_zeros
 * @param state_space state space structure pointer (source)
 * @param zpk zeros-poles-gain structure pointer (destination)
 * @param time_domain system time domain (continuous/discrete)
 * @return system control library error code
 */
return_code convert_state_space_to_zeros_poles_gain(const state_space_T* state_space, zeros_poles_gain_T* zpk, system_time_domain time_domain) {
	return_code returnval = return_OK;

	returnval = analyze_state_space_gain(state_space, &zpk->gain, time_domain);
	if (returnval != return_OK)
		return returnval;
	returnval = analyze_state_space_poles(state_space, &zpk->poles);
	if (returnval != return_OK)
		return returnval;
	returnval = analyze_state_space_zeros(state_space, &zpk->zeros);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Analyzes state space system representation poles using eigenvalues analysis.
 * @param state_space state space structure pointer (source)
 * @param poles poles complex vector (destination)
 * @return system control library error code
 */
return_code analyze_state_space_poles(const state_space_T* state_space, vector_generic_T* poles) {
	return_code returnval = return_OK;
	const matrix_T* A = &state_space->A;

	returnval = state_space_assert(state_space);
	if (returnval != return_OK)
		return returnval;
	size_t order = A->n_rows;
	if (vector_assert(poles) != return_OK) {
		returnval = vector_type_init(poles, complex_t, order, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(poles, complex_t))
		return return_WRONG_DIMENSIONS;
	if (vector_length(poles) != order)
		return return_WRONG_DIMENSIONS;
#endif

	gsl_matrix* A_gsl = NULL;
	gsl_eigen_nonsymmv_workspace * w = NULL;
	gsl_matrix_complex* evec = NULL;
	gsl_vector_complex eval = vector_complex_to_gsl_vector_complex(poles);

	A_gsl = matrix_to_gsl_matrix_dynamic_copy(A);
	if (A_gsl == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	evec = gsl_matrix_complex_calloc(order, order);
	if (evec == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	w = gsl_eigen_nonsymmv_alloc(order);
	if (w == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	returnval = gsl_error_code_to_return_code(gsl_eigen_nonsymmv(A_gsl, &eval, evec, w));
	ret: gsl_matrix_free(A_gsl);
	gsl_eigen_nonsymmv_free(w);
	gsl_matrix_complex_free(evec);

	return returnval;
}

/**
 * Analyzes state space system representation zeros using eigenvalues analysis.
 * @param state_space state space structure pointer
 * @param zeros zeros complex vector (destination)
 * @return system control library error code
 */
return_code analyze_state_space_zeros(const state_space_T* state_space, vector_generic_T* zeros) {
	return_code returnval = return_OK;
	const matrix_T* A = &state_space->A;
	const vector_generic_T* B = &state_space->B;
	const vector_generic_T* C = &state_space->C;

	returnval = state_space_assert(state_space);
	if (returnval != return_OK)
		return returnval;

	size_t order = A->n_rows;

	gsl_eigen_gen_workspace* w = NULL;
	gsl_matrix* eigen_A = NULL;
	gsl_matrix* eigen_B = NULL;
	gsl_vector_complex * alpha = NULL;
	gsl_vector * beta = NULL;
	vector_generic_T zeros_temp = { 0 };

	const gsl_matrix A_gsl = matrix_to_gsl_matrix(A);
	matrix_T B_mat=vector_real_to_col_matrix(B);
	matrix_T C_mat=vector_real_to_row_matrix(C);
	const gsl_matrix B_gsl = matrix_to_gsl_matrix(&B_mat);
	const gsl_matrix C_gsl = matrix_to_gsl_matrix(&C_mat);

	alpha = gsl_vector_complex_calloc(order + 1);
	if (alpha == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	beta = gsl_vector_calloc(order + 1);
	if (beta == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	w = gsl_eigen_gen_alloc(order + 1);
	if (w == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	eigen_A = gsl_matrix_calloc(order + 1, order + 1);
	if (eigen_A == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}
	eigen_B = gsl_matrix_calloc(order + 1, order + 1);
	if (eigen_B == NULL) {
		returnval = return_NULLPTR;
		goto ret;
	}

	gsl_matrix eigen_A_submatrix = gsl_matrix_submatrix(eigen_A, 0, 0, order, order).matrix;
	gsl_matrix eigen_A_last_col_submatrix = gsl_matrix_submatrix(eigen_A, 0, order, order, 1).matrix;
	gsl_matrix eigen_A_last_row_submatrix = gsl_matrix_submatrix(eigen_A, order, 0, 1, order).matrix;


	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_submatrix, &A_gsl));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_last_col_submatrix, &B_gsl));
	if (returnval != return_OK)
		goto ret;
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_last_row_submatrix, &C_gsl));
	if (returnval != return_OK)
		goto ret;

	gsl_matrix lambda_I = gsl_matrix_submatrix(eigen_B, 0, 0, order, order).matrix;
	gsl_matrix_set_identity(&lambda_I);

	returnval = gsl_error_code_to_return_code(gsl_eigen_gen(eigen_A, eigen_B, alpha, beta, w));
	if (returnval != return_OK)
		goto ret;

	uint n_zeros = 0;
	returnval = vector_type_init(&zeros_temp, complex_t, order + 1, NULL);
	if (returnval != return_OK)
		goto ret;

	for (uint i = 0; i < order + 1; i++) {
		if (gsl_vector_get(beta, i) != 0.0 && gsl_vector_get(beta, i) != NAN) {
			gsl_complex lambda = gsl_vector_complex_get(alpha, i);
			lambda.dat[0] /= gsl_vector_get(beta, i);
			lambda.dat[1] /= gsl_vector_get(beta, i);
			vector_type_at(&zeros_temp, gsl_complex, n_zeros) = lambda;
			n_zeros++;
		}
	}

	if (vector_assert(zeros) != return_OK) {
		returnval = vector_type_init(zeros, complex_t, n_zeros, NULL);
		if (returnval != return_OK)
			goto ret;
	}
#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(zeros, complex_t)) {
		returnval = return_WRONG_DIMENSIONS;
		goto ret;
	}
	if (vector_length(zeros) != n_zeros) {
		returnval = return_WRONG_DIMENSIONS;
		goto ret;
	}
#endif
	vector_generic_load(zeros, vector_generic_get_data_ptr(&zeros_temp));

	ret: gsl_eigen_gen_free(w);
	gsl_vector_complex_free(alpha);
	gsl_vector_free(beta);
	gsl_matrix_free(eigen_A);
	gsl_matrix_free(eigen_B);
	return returnval;

}

/**
 * Analyzes state space system representation using poles location analysis depending on time domain.
 * @param state_space state space structure pointer
 * @param time_domain system time domain (continuous/discrete)
 * @return true/false
 */
bool_t analyze_state_space_is_stable(const state_space_T* state_space, system_time_domain time_domain) {
	return_code returnval = return_OK;
	vector_generic_T poles = { 0 };
	bool_t is_stable = bool_error;
	returnval = analyze_state_space_poles(state_space, &poles);
	if (returnval != return_OK)
		goto ret;
	is_stable = analyze_roots_are_stable(&poles, time_domain);
	ret: vector_generic_deinit(&poles);
	return is_stable;
}
#endif

/**
 * Analytical transfer functions serial connection.
 * @param transfer_function_A transfer function A structure pointer (source)
 * @param transfer_function_B transfer function B structure pointer (source)
 * @param transfer_function_result destination transfer function structure pointer (destination)
 * @return system control library error code
 */
return_code convert_transfer_function_serial(const transfer_function_T* transfer_function_A, const transfer_function_T* transfer_function_B, transfer_function_T* transfer_function_result) {

	const polynom_T* numerator_A = &transfer_function_A->numerator_coeffs;
	const polynom_T* denominator_A = &transfer_function_A->denominator_coeffs;
	const polynom_T* numerator_B = &transfer_function_B->numerator_coeffs;
	const polynom_T* denominator_B = &transfer_function_B->denominator_coeffs;
	polynom_T* numerator_result = &transfer_function_result->numerator_coeffs;
	polynom_T* denominator_result = &transfer_function_result->denominator_coeffs;

	return_code returnval = return_OK;
	returnval = transfer_function_assert(transfer_function_A);
	if (returnval != return_OK)
		return returnval;
	returnval = transfer_function_assert(transfer_function_B);
	if (returnval != return_OK)
		return returnval;

	returnval = polynom_mul(numerator_A, numerator_B, numerator_result);
	if (returnval != return_OK)
		return returnval;

	returnval = polynom_mul(denominator_A, denominator_B, denominator_result);
	if (returnval != return_OK)
		return returnval;
	convert_transfer_function_normalize(transfer_function_result);

	return return_OK;
}

/**
 * Analytical transfer functions parallel connection.
 * @param transfer_function_A transfer function A structure pointer (source)
 * @param transfer_function_B transfer function B structure pointer (source)
 * @param transfer_function_result destination transfer function structure pointer (destination)
 * @return system control library error code
 */
return_code convert_transfer_function_paralel(const transfer_function_T* transfer_function_A, const transfer_function_T* transfer_function_B, transfer_function_T* transfer_function_result) {

	const polynom_T* numerator_A = &transfer_function_A->numerator_coeffs;
	const polynom_T* denominator_A = &transfer_function_A->denominator_coeffs;
	const polynom_T* numerator_B = &transfer_function_B->numerator_coeffs;
	const polynom_T* denominator_B = &transfer_function_B->denominator_coeffs;
	polynom_T* numerator_result = &transfer_function_result->numerator_coeffs;
	polynom_T* denominator_result = &transfer_function_result->denominator_coeffs;

	return_code returnval = return_OK;
	returnval = transfer_function_assert(transfer_function_A);
	if (returnval != return_OK)
		return returnval;
	returnval = transfer_function_assert(transfer_function_B);
	if (returnval != return_OK)
		return returnval;

	polynom_T product_1 = { 0 };
	polynom_T product_2 = { 0 };

	returnval = polynom_mul(numerator_A, denominator_B, &product_1);
	if (returnval != return_OK)
		goto ret;
	returnval = polynom_mul(numerator_B, denominator_A, &product_2);
	if (returnval != return_OK)
		goto ret;
	returnval = polynom_add(&product_1, &product_2, numerator_result);
	if (returnval != return_OK)
		goto ret;
	returnval = polynom_mul(denominator_A, denominator_B, denominator_result);
	if (returnval != return_OK)
		goto ret;

	convert_transfer_function_normalize(transfer_function_result);
	ret: vector_generic_deinit(&product_1);
	vector_generic_deinit(&product_2);
	return returnval;
}

/**
 * Analytical transfer functions feedback connection.
 * @param transfer_function_A transfer function A structure pointer (source)
 * @param transfer_function_B transfer function B structure pointer  (source)
 * @param transfer_function_result destination transfer function structure pointer (destination)
 * @return system control library error code
 */
return_code convert_transfer_function_feedback(const transfer_function_T* transfer_function_A, const transfer_function_T* transfer_function_B, transfer_function_T* transfer_function_result) {

	const polynom_T* numerator_A = &transfer_function_A->numerator_coeffs;
	const polynom_T* denominator_A = &transfer_function_A->denominator_coeffs;
	const polynom_T* numerator_B = &transfer_function_B->numerator_coeffs;
	const polynom_T* denominator_B = &transfer_function_B->denominator_coeffs;
	polynom_T* numerator_result = &transfer_function_result->numerator_coeffs;
	polynom_T* denominator_result = &transfer_function_result->denominator_coeffs;

	return_code returnval = return_OK;
	returnval = transfer_function_assert(transfer_function_A);
	if (returnval != return_OK)
		return returnval;
	returnval = transfer_function_assert(transfer_function_B);
	if (returnval != return_OK)
		return returnval;

	polynom_T numerators_product = { 0 };
	polynom_T denominators_product = { 0 };

	returnval = polynom_mul(numerator_A, numerator_B, &numerators_product);
	if (returnval != return_OK)
		goto ret;
	returnval = polynom_mul(denominator_A, denominator_B, &denominators_product);
	if (returnval != return_OK)
		goto ret;

	returnval = polynom_add(&numerators_product, &denominators_product, denominator_result);
	if (returnval != return_OK)
		goto ret;

	returnval = polynom_mul(numerator_A, denominator_B, numerator_result);
	if (returnval != return_OK)
		goto ret;

	convert_transfer_function_normalize(transfer_function_result);
	ret: vector_generic_deinit(&numerators_product);
	vector_generic_deinit(&denominators_product);

	return returnval;
}

