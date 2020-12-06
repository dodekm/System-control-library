#include "control_synthesis.h"

using namespace SystemControl;

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
	const Complex cont_root((real_t) -1 / Tc, 0);
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

void ControlSynthesis::RST_poleplace(const VectorReal& A, const VectorReal& B, const VectorReal& P, VectorReal& R, VectorReal& S, VectorReal& T) {

	A.assert();
	B.assert();
	P.assert();

	size_t P_length = P.get_length();
	if (P_length != (A.get_length() + B.get_length() - 2))
		throw exception_WRONG_DIMENSIONS;
	size_t R_length = B.get_length() - 1;
	size_t S_length = A.get_length() - 1;

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

#ifndef ASSERT_DIMENSIONS
	if (R.get_length() != R_length)
		throw exception_WRONG_DIMENSIONS;
	if (S.get_length() != S_length)
		throw exception_WRONG_DIMENSIONS;
	if (T.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	Matrix M(P_length, P_length);
	Matrix M_submatrix_A(M, P_length, R_length, 0, 0);
	Matrix M_submatrix_B(M, P_length, S_length, 0, R_length);

	convolution_matrix(A, M_submatrix_A, R_length);
	convolution_matrix(B, M_submatrix_B, S_length);

	VectorReal X;
	M.solve(P, X);

	R = VectorReal(X, R_length, 0);
	S = VectorReal(X, S_length, R_length);
	T[0] = P.sum() / B.sum();

}

void ControlSynthesis::PI_poleplace(const System1stOrderParams& system_params, const ReferencePolynom2ndOrder& desired_polynom, PID_regulator_params& regulator_params) {

	regulator_params.P_gain = ((real_t) 2.0 * desired_polynom.b * desired_polynom.omega_0 * system_params.T - 1.0) / system_params.K;
	regulator_params.I_gain = POW2(desired_polynom.omega_0) * system_params.T / system_params.K;
}

void ControlSynthesis::PIV_poleplace(const System1stOrderParams& system_params, const ReferencePolynom3rdOrder& desired_polynom, PID_regulator_params& IV_regulator_params, PID_regulator_params& P_regulator_params) {

	IV_regulator_params.P_gain = (((real_t) 2.0 * desired_polynom.b * desired_polynom.omega_0 + desired_polynom.k) * system_params.T - (real_t) 1.0) / system_params.K;
	IV_regulator_params.I_gain = (POW2(desired_polynom.omega_0) + (real_t) 2.0 * desired_polynom.b * desired_polynom.omega_0 * desired_polynom.k) * system_params.T / system_params.K;
	P_regulator_params.P_gain = (POW2(desired_polynom.omega_0) * desired_polynom.k) / (POW2(desired_polynom.omega_0) + (real_t) 2.0 * desired_polynom.b * desired_polynom.omega_0 * desired_polynom.k);

}

