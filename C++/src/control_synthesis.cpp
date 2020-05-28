#include "control_synthesis.h"
#include "wrappers.h"

using namespace SystemControl;
using namespace Convert;

Complex ControlSynthesis::convert_continuous_root_to_discrete_root(Complex S_root, real_t Ts) {
	S_root.get_real() *= Ts;
	S_root.get_imag() *= Ts;
	Complex Z_root = S_root.exp();
	return Z_root;
}

Complex ControlSynthesis::convert_discrete_root_to_continuous_root(Complex Z_root, real_t Ts) {

	Complex S_root = Z_root.log();
	S_root.get_real() /= Ts;
	S_root.get_imag() /= Ts;
	return S_root;
}

void ControlSynthesis::create_discrete_polynom_from_continuous_poles(const VectorComplex& poles_cont, Polynom& P, real_t Ts) {

	poles_cont.assert();
	VectorComplex poles_disc(poles_cont.get_length());
	for (uint i = 0; i < poles_cont.get_length(); i++) {
		poles_disc[i] = convert_continuous_root_to_discrete_root(poles_cont[i], Ts);
	}
	P.from_roots(poles_disc);

}

void ControlSynthesis::create_discrete_polynom_from_multiple_complex_root(Polynom& P, uint multiplicity, Complex root) {
	VectorComplex roots_vector(multiplicity);
	auto lambda = [&root](Complex& C,uint i) ->auto {
		if (i % 2)
		C = root;
		else
		C = root.conj();
		return true;
	};
	roots_vector.for_each(lambda);
	P.from_roots(roots_vector);
}

void ControlSynthesis::create_discrete_aperiodic_polynom_for_multiple_time_constant(Polynom& P, uint multiplicity, real_t Tc, real_t Ts) {
	const Complex cont_root((real_t)-1 / Tc, 0);
	Complex disc_root = convert_continuous_root_to_discrete_root(cont_root, Ts);
	create_discrete_polynom_from_multiple_complex_root(P, multiplicity, disc_root);
}

void ControlSynthesis::convolution_matrix(const VectorReal& g, Matrix& convMat, uint f_length) {
	g.assert();

	int g_length = (int) g.get_length();
	int f_g_length = f_length + g_length - 1;

	try {
		convMat.assert();
	} catch (...) {
		convMat = Matrix(f_g_length, f_length);
	}

#ifdef ASSERT_DIMENSIONS
	if ((int) convMat.get_n_rows() != f_g_length || convMat.get_n_cols() != f_length)
		throw exception_WRONG_DIMENSIONS;
#endif

	for (int i = 0; i < f_g_length; i++) {
		for (int j = 0; j < MAX((int )f_length, g_length); j++) {
			if (i - j >= 0 && i - j < (int) f_length) {
				if (j < g_length)
					convMat(i, i - j) = g[j];
				else
					convMat(i, i - j) = 0;
			}
		}
	}

}

Matrix ControlSynthesis::convolution_matrix(const VectorReal& g, uint f_length) {
	Matrix convMat(f_length + g.get_length() - 1, f_length);
	convolution_matrix(g, convMat, f_length);
	return convMat;
}

#ifdef USE_GSL
void ControlSynthesis::RST_poleplace(const Polynom& A, const Polynom& B, const Polynom& P, Polynom& R, Polynom& S, Polynom& T) {

	A.assert();
	B.assert();
	P.assert();

	int P_length = (int) P.get_length();
	if (P_length != (int) (A.get_length() + B.get_length() - 1))
		throw exception_WRONG_DIMENSIONS;
	int R_length = (int) B.get_length();
	int S_length = (int) A.get_length() - 1;

	try {
		R.assert();
	} catch (...) {
		R = Polynom(R_length);
	}
	try {
		S.assert();
	} catch (...) {
		S = Polynom(S_length);
	}
	try {
		T.assert();
	} catch (...) {
		T = Polynom(1);
	}

#ifdef ASSERT_DIMENSIONS
	if ((int) R.get_length() != R_length)
		throw exception_WRONG_DIMENSIONS;
	if ((int) S.get_length() != S_length)
		throw exception_WRONG_DIMENSIONS;
	if ((int) T.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	Matrix M(P_length, P_length);
	Matrix M_submatrix_A = M.submatrix(P_length, R_length, 0, 0);
	Matrix M_submatrix_B = M.submatrix(P_length - 1, S_length, 1, R_length);

	convolution_matrix(A, M_submatrix_A, R_length);
	convolution_matrix(B, M_submatrix_B, S_length);

	gsl_permutation* perm_gsl = NULL;
	gsl_vector* X_gsl = NULL;

	auto dealloc = [&]() {
		gsl_permutation_free(perm_gsl);
		gsl_vector_free(X_gsl);
	};

	gsl_matrix M_gsl = M.to_gsl_matrix();
	gsl_vector P_gsl = P.to_gsl_vector();

	perm_gsl = gsl_permutation_alloc(P_length);
	if (perm_gsl == NULL)
		throw exception_NULLPTR;

	X_gsl = gsl_vector_alloc(P_length);
	if (X_gsl == NULL) {
		dealloc();
		throw exception_NULLPTR;
	}
	exception_code returnval = exception_OK;

	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_decomp(&M_gsl, perm_gsl, NULL));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}
	returnval = gsl_error_code_to_return_code(gsl_linalg_LU_solve(&M_gsl, perm_gsl, &P_gsl, X_gsl));
	if (returnval != exception_OK) {
		dealloc();
		throw returnval;
	}

	R.load_data(X_gsl->data);
	S.load_data(X_gsl->data + R_length);
	T[0] = P.sum() / B.sum();

	dealloc();
}
#endif

void ControlSynthesis::PI_poleplace(const System1stOrderParams& system_params, const ReferencePolynom2ndOrder& desired_polynom, PID_regulator_params& regulator_params) {

	regulator_params.P_gain = ((real_t)2.0 * desired_polynom.b * desired_polynom.omega_0 * system_params.T - 1.0) / system_params.K;
	regulator_params.I_gain = POW2(desired_polynom.omega_0) * system_params.T / system_params.K;
}

void ControlSynthesis::PIV_poleplace(const System1stOrderParams& system_params, const ReferencePolynom3rdOrder& desired_polynom, PID_regulator_params& IV_regulator_params, PID_regulator_params& P_regulator_params) {

	IV_regulator_params.P_gain = (((real_t)2.0 * desired_polynom.b * desired_polynom.omega_0 + desired_polynom.k) * system_params.T - (real_t)1.0) / system_params.K;
	IV_regulator_params.I_gain = (POW2(desired_polynom.omega_0) + (real_t)2.0 * desired_polynom.b * desired_polynom.omega_0 * desired_polynom.k) * system_params.T / system_params.K;
	P_regulator_params.P_gain = (POW2(desired_polynom.omega_0) * desired_polynom.k) / (POW2(desired_polynom.omega_0) + (real_t)2.0 * desired_polynom.b * desired_polynom.omega_0 * desired_polynom.k);

}
