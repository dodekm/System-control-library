#include "systems_analyze.h"

#include "signal_process.h"
#include "polynom.h"
#include "wrappers.h"

using namespace SystemControl;
using namespace Convert;

Complex SystemsConvert::continuous_root_to_discrete_root(Complex S_root, real_t Ts) {
	S_root = S_root * Complex(Ts);
	return S_root.exp();
}

Complex SystemsConvert::discrete_root_to_continuous_root(Complex Z_root, real_t Ts) {

	Complex S_root = Z_root.log();
	S_root = S_root / Complex(Ts);
	return S_root;
}

bool SystemsAnalyze::roots_stability(const VectorComplex& roots, System::system_time_domain time_domain) {

	roots.assert();
	for (uint i = 0; i < roots.get_length(); i++) {
		if (time_domain == System::system_time_domain_continuous) {
			if (roots[i].get_real() > 0)
				return false;
		} else if (time_domain == System::system_time_domain_discrete) {
			if (roots[i].magnitude() > 1)
				return false;
		}
	}
	return true;
}

void TransferFunction::get_poles(VectorComplex& poles) const {
	denominator_coeffs.roots(poles);
}
void TransferFunction::get_zeros(VectorComplex& zeros) const {
	numerator_coeffs.roots(zeros);
}

bool TransferFunction::is_stable() const {

	assert();
	if (denominator_coeffs.get_length() == 1)
		return true;
	VectorComplex poles;
	get_poles(poles);
	return SystemsAnalyze::roots_stability(poles, time_domain);

}
real_t TransferFunction::get_gain() const

{
	assert();
	real_t gain = 0;
	if (time_domain == System::system_time_domain_continuous) {
		gain = numerator_coeffs[0] / denominator_coeffs[0];
	} else if (time_domain == System::system_time_domain_discrete) {
		real_t num_sum = numerator_coeffs.sum();
		real_t den_sum = denominator_coeffs.sum();
		gain = num_sum / den_sum;
	}
	return gain;
}

void TransferFunction::set_gain(real_t gain) {

	real_t gain_old = get_gain();
	real_t k = gain / gain_old;
	numerator_coeffs.VectorReal::mul(numerator_coeffs, k);
}
void TransferFunction::normalize() {
	assert();
	real_t k = denominator_coeffs[get_order()];
	numerator_coeffs.VectorReal::div(numerator_coeffs, k);
	denominator_coeffs.VectorReal::div(denominator_coeffs, k);
}

void TransferFunction::to_state_space(StateSpace& state_space) const {

	assert();
	try {
		state_space.assert();
	} catch (...) {
		state_space = StateSpace(get_order(), time_domain);
	}

#ifdef ASSERT_DIMENSIONS
	if (numerator_coeffs.get_length() >= denominator_coeffs.get_length())
		throw exception_WRONG_DIMENSIONS;
	if (state_space.get_order() != get_order())
		throw exception_WRONG_DIMENSIONS;
#endif

	size_t order = get_order();

	for (uint i = 0; i < order; i++) {
		for (uint j = 0; j < order; j++) {
			if (i == order - 1) {

				state_space.A(i, j) = -denominator_coeffs[j] / denominator_coeffs[order];
			} else {
				if (j == i + 1)
					state_space.A(i, j) = 1;
				else
					state_space.A(i, j) = 0;
			}
		}
		if (i == order - 1)
			state_space.B[i] = 1;
		else
			state_space.B[i] = 0;
		if (i < numerator_coeffs.get_length())
			state_space.C[i] = numerator_coeffs[i] / denominator_coeffs[order];
		else
			state_space.C[i] = 0;
	}

}
void TransferFunction::to_zero_pole_gain(ZeroPoleGain& zpk) const {

	zpk.gain = get_gain();
	numerator_coeffs.roots(zpk.zeros);
	denominator_coeffs.roots(zpk.poles);
}

void TransferFunction::to_discrete_forward_Euler(TransferFunction& tf_disc, real_t Ts) const {
	const TransferFunction& tf_cont = *this;

	if (Ts <= 0)
		throw exception_ERROR;
	tf_cont.assert();
	try {
		tf_disc.assert();
	} catch (...) {
		tf_disc = TransferFunction(tf_cont.numerator_coeffs.get_length(), tf_cont.denominator_coeffs.get_length(), System::system_time_domain_discrete);
	}

#ifdef ASSERT_DIMENSIONS
	if (tf_cont.numerator_coeffs.get_length() != tf_cont.denominator_coeffs.get_length()) //FIXME num/den must have same length
		throw exception_WRONG_DIMENSIONS;
	if (tf_cont.numerator_coeffs.get_length() != tf_disc.numerator_coeffs.get_length())
		throw exception_WRONG_DIMENSIONS;
	if (tf_cont.denominator_coeffs.get_length() != tf_disc.denominator_coeffs.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	real_t aprox_polynom_data[] = { (real_t) (-1.0) / Ts, (real_t) (1.0) / Ts };
	const Polynom aprox_polynom_base(2, aprox_polynom_data);

	Polynom aprox_polynom_i_0_deg(1);
	aprox_polynom_i_0_deg[0] = 1;

	tf_disc.numerator_coeffs[0] = tf_cont.numerator_coeffs[0];
	tf_disc.denominator_coeffs[0] = tf_cont.denominator_coeffs[0];

	for (uint i = 1; i < get_order(); i++) {
		Polynom aprox_polynom_i_1_deg = aprox_polynom_i_0_deg.mul(aprox_polynom_base);
		aprox_polynom_i_0_deg = aprox_polynom_i_1_deg;
		aprox_polynom_i_0_deg.VectorReal::mul(aprox_polynom_i_0_deg, tf_cont.numerator_coeffs[i]);
		tf_disc.numerator_coeffs.add(tf_disc.numerator_coeffs, aprox_polynom_i_0_deg);
		aprox_polynom_i_0_deg = aprox_polynom_i_1_deg;
		aprox_polynom_i_0_deg.VectorReal::mul(aprox_polynom_i_0_deg, tf_cont.denominator_coeffs[i]);
		tf_disc.denominator_coeffs.add(tf_disc.denominator_coeffs, aprox_polynom_i_0_deg);
		aprox_polynom_i_0_deg = aprox_polynom_i_1_deg;
	}
	tf_disc.normalize();
}

void TransferFunction::to_discrete_bilinear(TransferFunction& tf_disc, real_t Ts) const {
	const TransferFunction& tf_cont = *this;
	if (Ts <= 0)
		throw exception_ERROR;
	tf_cont.assert();
	try {
		tf_disc.assert();
	} catch (...) {
		tf_disc = TransferFunction(tf_cont.numerator_coeffs.get_length(), tf_cont.denominator_coeffs.get_length(), System::system_time_domain_discrete);
	}

#ifdef ASSERT_DIMENSIONS
	if (tf_cont.numerator_coeffs.get_length() != tf_cont.denominator_coeffs.get_length()) //FIXME num/den must have same length
		throw exception_WRONG_DIMENSIONS;
	if (tf_cont.numerator_coeffs.get_length() != tf_disc.numerator_coeffs.get_length())
		throw exception_WRONG_DIMENSIONS;
	if (tf_cont.denominator_coeffs.get_length() != tf_disc.denominator_coeffs.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	real_t aprox_polynom_numerator_data[] = { (real_t) -2.0 / Ts, (real_t) 2.0 / Ts };
	const Polynom aprox_polynom_numerator(2, aprox_polynom_numerator_data);
	real_t aprox_polynom_denominator_data[] = { 1.0, 1.0 };
	const Polynom aprox_polynom_denominator(2, aprox_polynom_denominator_data);

	Polynom aprox_polynom_i_1_deg;

	for (uint i = 0; i <= get_order(); i++) {
		Polynom aprox_polynom_i_1_deg(1);
		Polynom aprox_polynom_i_0_deg(1);
		aprox_polynom_i_0_deg[0] = 1;
		aprox_polynom_i_1_deg[0] = 1;

		for (uint j = 0; j < i; j++) {
			aprox_polynom_i_1_deg = aprox_polynom_i_0_deg.mul(aprox_polynom_numerator);
			aprox_polynom_i_0_deg = aprox_polynom_i_1_deg;
		}
		for (uint j = i; j < get_order(); j++) {
			aprox_polynom_i_1_deg = aprox_polynom_i_0_deg.mul(aprox_polynom_denominator);
			aprox_polynom_i_0_deg = aprox_polynom_i_1_deg;
		}
		aprox_polynom_i_0_deg.VectorReal::mul(aprox_polynom_i_0_deg, tf_cont.numerator_coeffs[i]);
		tf_disc.numerator_coeffs.add(tf_disc.numerator_coeffs, aprox_polynom_i_0_deg);

		aprox_polynom_i_1_deg.VectorReal::mul(aprox_polynom_i_1_deg, tf_cont.denominator_coeffs[i]);
		tf_disc.denominator_coeffs.add(tf_disc.denominator_coeffs, aprox_polynom_i_1_deg);
	}

	tf_disc.normalize();
}

void TransferFunction::to_discrete_match_zeros_poles(TransferFunction& tf_disc, real_t Ts) const {
	const TransferFunction& tf_cont = *this;
	ZeroPoleGain zpk(numerator_coeffs.get_length() - 1, denominator_coeffs.get_length() - 1, System::system_time_domain_discrete, 0);
	tf_cont.to_zero_pole_gain(zpk);
	zpk.to_discrete(zpk, Ts);
	zpk.to_transfer_function(tf_disc);

	tf_disc.normalize();
}

void TransferFunction::serial_transfer_function_(const TransferFunction& transfer_function_A, const TransferFunction& transfer_function_B) {

	transfer_function_A.assert();
	transfer_function_B.assert();

	numerator_coeffs.mul(transfer_function_A.numerator_coeffs, transfer_function_B.numerator_coeffs);
	denominator_coeffs.mul(transfer_function_A.denominator_coeffs, transfer_function_B.denominator_coeffs);
	normalize();

}
void TransferFunction::paralel_transfer_function(const TransferFunction& transfer_function_A, const TransferFunction& transfer_function_B) {

	transfer_function_A.assert();
	transfer_function_B.assert();

	Polynom product_1 = transfer_function_A.numerator_coeffs.mul(transfer_function_B.denominator_coeffs);
	Polynom product_2 = transfer_function_B.numerator_coeffs.mul(transfer_function_A.denominator_coeffs);

	numerator_coeffs.add(product_1, product_2);
	denominator_coeffs.mul(transfer_function_A.denominator_coeffs, transfer_function_B.denominator_coeffs);
	normalize();
}
void TransferFunction::feedback_transfer_function(const TransferFunction& transfer_function_A, const TransferFunction& transfer_function_B) {

	transfer_function_A.assert();
	transfer_function_B.assert();

	Polynom numerators_product = transfer_function_A.numerator_coeffs.mul(transfer_function_B.numerator_coeffs);
	Polynom denominators_product = transfer_function_A.denominator_coeffs.mul(transfer_function_B.denominator_coeffs);

	denominator_coeffs.add(numerators_product, denominators_product);
	numerator_coeffs.mul(transfer_function_A.numerator_coeffs, transfer_function_B.denominator_coeffs);
	normalize();
}

void ZeroPoleGain::to_discrete(ZeroPoleGain& zpk_disc, real_t Ts) const {

	const ZeroPoleGain& zpk_cont = *this;

	if (Ts <= 0)
		throw exception_ERROR;
	zpk_cont.assert();
	try {
		zpk_disc.assert();
	} catch (...) {
		zpk_disc = ZeroPoleGain(zpk_cont.zeros.get_length(), zpk_cont.poles.get_length(), System::system_time_domain_discrete, zpk_cont.gain);
	}
#ifdef ASSERT_DIMENSIONS
	if (zpk_cont.zeros.get_length() != zpk_disc.zeros.get_length())
		throw exception_WRONG_DIMENSIONS;
	if (zpk_cont.poles.get_length() != zpk_disc.poles.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < zeros.get_length(); i++) {
		zpk_disc.zeros[i] = SystemsConvert::continuous_root_to_discrete_root(zpk_cont.zeros[i], Ts);
	}
	for (uint i = 0; i < poles.get_length(); i++) {
		zpk_disc.poles[i] = SystemsConvert::continuous_root_to_discrete_root(zpk_cont.poles[i], Ts);
	}
	zpk_disc.gain = zpk_cont.gain;

}
void ZeroPoleGain::to_transfer_function(TransferFunction& transfer_function) const {

	transfer_function.numerator_coeffs.from_roots(zeros);
	transfer_function.denominator_coeffs.from_roots(poles);
	transfer_function.set_gain(gain);
}
bool ZeroPoleGain::is_stable() const {
	return SystemsAnalyze::roots_stability(poles, time_domain);
}

void StateSpace::to_discrete_Taylor_series(StateSpace& ss_disc, real_t Ts, uint series_order) const {

	const StateSpace& ss_cont = *this;
	if (series_order > 20)
		throw exception_ERROR;
	if (Ts <= 0)
		throw exception_ERROR;
	ss_cont.assert();
	try {
		ss_disc.assert();
	} catch (...) {
		ss_disc = StateSpace(get_order(), System::system_time_domain_discrete);
	}

#ifdef ASSERT_DIMENSIONS
	if (ss_cont.get_order() != ss_disc.get_order())
		throw exception_WRONG_DIMENSIONS;
#endif

	uint64_t n_factorial = 1;
	real_t Ts_power = 1;

	Matrix A_cont_power_i_0(ss_cont.A, false);
	Matrix A_cont_power_i_1(ss_cont.A, false);
	VectorReal A_cont_power_B_i_1(get_order());
	A_cont_power_i_0.set_identity();
	A_cont_power_i_1.set_identity();

	for (uint i = 0; i < series_order; i++) {

		A_cont_power_i_1.element_mul(A_cont_power_i_1, Ts_power / (double) n_factorial);
		ss_disc.A.element_add(ss_disc.A, A_cont_power_i_1);
		n_factorial *= (i + 1);
		Ts_power *= Ts;
		VectorReal2colMatrix(A_cont_power_B_i_1).multiply(A_cont_power_i_0, VectorReal2colMatrix(ss_cont.B));
		VectorReal2colMatrix(ss_disc.B).multiply_by_scalar_and_accumulate(VectorReal2colMatrix(A_cont_power_B_i_1), Ts_power / (double) n_factorial);
		A_cont_power_i_1.multiply(A_cont_power_i_0, ss_cont.A);
		A_cont_power_i_0 = A_cont_power_i_1;
	}
	ss_disc.C = ss_cont.C;
}

#ifdef USE_GSL
void StateSpace::set_gain(real_t gain) {

	real_t gain_old = get_gain();
	real_t k = gain / gain_old;
	C.mul(C, k);
}
real_t StateSpace::get_gain() const {

	assert();
	gsl_matrix* A_gsl = NULL;
	gsl_permutation* P_gsl = NULL;
	gsl_vector* X_gsl = NULL;

	auto dealloc = [&]() {
		gsl_permutation_free(P_gsl);
		gsl_vector_free(X_gsl);
		gsl_matrix_free(A_gsl);
	};

	A_gsl = A.to_gsl_matrix_dynamic_copy();
	if (A_gsl == NULL) {
		throw exception_NULLPTR;
	}
	const gsl_vector B_gsl = B.to_gsl_vector();

	P_gsl = gsl_permutation_alloc(get_order());
	if (P_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	X_gsl = gsl_vector_calloc(get_order());
	if (X_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	exception_code returnval = exception_OK;
	returnval = gsl_error_code_to_return_code(gsl_matrix_scale(A_gsl, -1));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	if (time_domain == System::system_time_domain_discrete) {
		returnval = gsl_error_code_to_return_code(gsl_matrix_add_diagonal(A_gsl, 1));
		if (returnval != exception_OK) {
			dealloc();
			throw returnval;
		}
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(A_gsl, P_gsl, NULL));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_solve(A_gsl, P_gsl, &B_gsl, X_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}

	const gsl_vector C_gsl = C.to_gsl_vector();
	real_t gain = 0;
	returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&C_gsl, X_gsl, &gain));
	dealloc();

	if (returnval != exception_OK) {
		throw returnval;
	}
	return gain;

}
void StateSpace::get_poles(VectorComplex& poles) const {

	assert();
	try {
		poles.assert();
	} catch (...) {
		poles = VectorComplex(get_order());
	}

#ifdef ASSERT_DIMENSIONS
	if (poles.get_length() != get_order())
	throw exception_WRONG_DIMENSIONS;
#endif

	gsl_matrix* A_gsl = NULL;
	gsl_eigen_nonsymmv_workspace * w = NULL;
	gsl_matrix_complex* evec = NULL;
	gsl_vector_complex eval = poles.to_gsl_vector();

	auto dealloc = [&]() {
		gsl_matrix_free(A_gsl);
		gsl_eigen_nonsymmv_free(w);
		gsl_matrix_complex_free(evec);
	};

	A_gsl = A.to_gsl_matrix_dynamic_copy();
	if (A_gsl == NULL) {
		throw exception_NULLPTR;
	}
	evec = gsl_matrix_complex_calloc(get_order(), get_order());
	if (evec == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	w = gsl_eigen_nonsymmv_alloc(get_order());
	if (w == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	exception_code returnval = gsl_error_code_to_return_code(gsl_eigen_nonsymmv(A_gsl, &eval, evec, w));
	dealloc();
	if (returnval != exception_OK)
	throw returnval;

}
void StateSpace::get_zeros(VectorComplex& zeros) const {

	assert();

	gsl_eigen_gen_workspace* w = NULL;
	gsl_matrix* eigen_A = NULL;
	gsl_matrix* eigen_B = NULL;
	gsl_vector_complex * alpha = NULL;
	gsl_vector * beta = NULL;

	const gsl_matrix A_gsl = A.to_gsl_matrix();
	const gsl_matrix B_gsl = VectorReal2colMatrix(B).to_gsl_matrix();
	const gsl_matrix C_gsl = VectorReal2rowMatrix(C).to_gsl_matrix();

	auto dealloc = [&]() {
		gsl_eigen_gen_free(w);
		gsl_vector_complex_free(alpha);
		gsl_vector_free(beta);
		gsl_matrix_free(eigen_A);
		gsl_matrix_free(eigen_B);
	};

	alpha = gsl_vector_complex_calloc(get_order() + 1);
	if (alpha == NULL) {
		throw exception_NULLPTR;
	}
	beta = gsl_vector_calloc(get_order() + 1);
	if (beta == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	w = gsl_eigen_gen_alloc(get_order() + 1);
	if (w == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	eigen_A = gsl_matrix_calloc(get_order() + 1, get_order() + 1);
	if (eigen_A == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	eigen_B = gsl_matrix_calloc(get_order() + 1, get_order() + 1);
	if (eigen_B == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}

	gsl_matrix eigen_A_submatrix = gsl_matrix_submatrix(eigen_A, 0, 0, get_order(), get_order()).matrix;
	gsl_matrix eigen_A_last_col_submatrix = gsl_matrix_submatrix(eigen_A, 0, get_order(), get_order(), 1).matrix;
	gsl_matrix eigen_A_last_row_submatrix = gsl_matrix_submatrix(eigen_A, get_order(), 0, 1, get_order()).matrix;

	exception_code returnval = exception_OK;
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_submatrix, &A_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_last_col_submatrix, &B_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(&eigen_A_last_row_submatrix, &C_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	gsl_matrix lambda_I = gsl_matrix_submatrix(eigen_B, 0, 0, get_order(), get_order()).matrix;
	gsl_matrix_set_identity(&lambda_I);

	returnval = gsl_error_code_to_return_code(gsl_eigen_gen(eigen_A, eigen_B, alpha, beta, w));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	uint n_zeros = 0;
	VectorComplex zeros_temp(get_order() + 1);
	for (uint i = 0; i < get_order() + 1; i++) {
		real_t beta_i = gsl_vector_get(beta, i);
		if (beta_i != 0.0 && beta_i != NAN) {
			Complex lambda = Complex(gsl_vector_complex_get(alpha, i)) / Complex(beta_i);
			zeros_temp[n_zeros] = lambda;
			n_zeros++;
		}
	}

	try {
		zeros.assert();
	} catch (...) {
		zeros = VectorComplex(n_zeros);
	}

#ifdef ASSERT_DIMENSIONS
	if (zeros.get_length() != n_zeros)
	throw exception_WRONG_DIMENSIONS;
#endif
	zeros = zeros_temp.subvector(n_zeros, 0);
	dealloc();

}
bool StateSpace::is_stable() const {

	VectorComplex poles(get_order());
	get_poles(poles);
	return SystemsAnalyze::roots_stability(poles, time_domain);
}

void StateSpace::to_discrete(StateSpace& ss_disc, real_t Ts) const {

	const StateSpace& ss_cont = *this;
	if (Ts <= 0)
	throw exception_ERROR;
	ss_cont.assert();
	try {
		ss_disc.assert();
	} catch (...) {
		ss_disc = StateSpace(get_order(), System::system_time_domain_discrete);
	}

#ifdef ASSERT_DIMENSIONS
	if (ss_cont.get_order() != ss_disc.get_order())
	throw exception_WRONG_DIMENSIONS;
#endif

	gsl_matrix* A_cont_gsl = NULL;
	gsl_matrix* A_cont_inv_gsl = NULL;
	gsl_vector* A_cont_inv_mul_B_cont = NULL;
	gsl_matrix* eAT_gsl = NULL;
	gsl_permutation* P_gsl = NULL;
	gsl_matrix A_disc_gsl = ss_disc.A.to_gsl_matrix();
	gsl_vector B_disc_gsl = ss_disc.B.to_gsl_vector();
	const gsl_vector B_cont_gsl = ss_cont.B.to_gsl_vector();
	size_t order = ss_cont.get_order();

	auto dealloc = [&]() {
		gsl_matrix_free(A_cont_gsl);
		gsl_matrix_free(A_cont_inv_gsl);
		gsl_matrix_free(eAT_gsl);
		gsl_vector_free(A_cont_inv_mul_B_cont);
		gsl_permutation_free(P_gsl);
	};

	A_cont_gsl = ss_cont.A.to_gsl_matrix_dynamic_copy();
	if (A_cont_gsl == NULL) {
		throw exception_NULLPTR;
	}
	A_cont_inv_gsl = ss_cont.A.to_gsl_matrix_dynamic_copy();
	if (A_cont_inv_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}

	A_cont_inv_gsl = gsl_matrix_alloc(order, order);
	if (A_cont_inv_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	exception_code returnval = exception_OK;
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(A_cont_inv_gsl, A_cont_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	eAT_gsl = gsl_matrix_alloc(order, order);
	if (eAT_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}

	returnval = gsl_error_code_to_return_code(gsl_matrix_scale(A_cont_gsl, Ts));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	gsl_matrix* AT_gsl = A_cont_gsl;
	returnval = gsl_error_code_to_return_code(gsl_linalg_exponential_ss(AT_gsl, &A_disc_gsl, GSL_MODE_DEFAULT));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_matrix_memcpy(eAT_gsl, &A_disc_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}

	returnval = gsl_error_code_to_return_code(gsl_matrix_add_diagonal(eAT_gsl, -1));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	gsl_matrix* eAT_minus_diag = eAT_gsl;

	P_gsl = gsl_permutation_alloc(order);
	if (P_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(A_cont_inv_gsl, P_gsl, NULL));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_invx(A_cont_inv_gsl, P_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}

	A_cont_inv_mul_B_cont = gsl_vector_alloc(order);
	if (A_cont_inv_mul_B_cont == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1.0, A_cont_inv_gsl, &B_cont_gsl, 0, A_cont_inv_mul_B_cont));
	if (returnval != exception_OK)
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_blas_dgemv(CblasNoTrans, 1.0, eAT_minus_diag, A_cont_inv_mul_B_cont, 0, &B_disc_gsl));
	if (returnval != exception_OK)
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}

	ss_disc.C = ss_cont.C;

}
void StateSpace::to_transfer_function(TransferFunction& transfer_function) const {

	ZeroPoleGain zpk;
	to_zero_pole_gain(zpk);
	zpk.to_transfer_function(transfer_function);

}
void StateSpace::to_zero_pole_gain(ZeroPoleGain& zpk) const {
	zpk.gain = get_gain();
	get_poles(zpk.poles);
	get_zeros(zpk.zeros);
}
#endif

