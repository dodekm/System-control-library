#include "polynom.h"

using namespace SystemControl;


real_t Polynom::eval(real_t x) const {
	assert();

#ifdef USE_GSL
	return gsl_poly_eval(data_ptr, length, x);
#else
	return eval<real_t>(x);
#endif
}

Complex Polynom::eval(Complex x) const {
	assert();
#ifdef USE_GSL
	return Complex(gsl_poly_complex_eval(data_ptr, length, x.to_gsl_complex()));

#else
	return eval<Complex>(x);
#endif
}

template<typename T>
T Polynom::eval(T x) const {
	T y(0.0);
	T x_pow(1.0);
	auto lambda = [x,&x_pow,&y](auto P_i,auto i)->auto {
		y = y + T(P_i) * x_pow;
		x_pow = x_pow * x;
		return true;
	};
	for_each(lambda);
	return y;
}

Polynom& Polynom::mul(const Polynom& A, const Polynom& B) {
	convolute(A, B);
	return *this;
}

Polynom Polynom::mul(const Polynom& B) const {
	return Polynom(convolute(B));

}

Polynom& Polynom::add(const Polynom& A, const Polynom& B) {

	A.assert();
	B.assert();
	size_t max_degree = MAX(A.length, B.length);

	try {
		assert();
	} catch (...) {
		*this = Polynom(max_degree);
	}
#ifdef ASSERT_DIMENSIONS
	if (length != max_degree)
		throw exception_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < max_degree; i++) {
		real_t x = 0;
		if (i < A.length)
			x += A[i];
		if (i < B.length)
			x += B[i];
		this->at(i) = x;
	}
	return *this;
}

Polynom Polynom::add(const Polynom& B) const {
	const Polynom& A = *this;
	Polynom C(MAX(A.length, B.length));
	C.add(A, B);
	return C;
}

Polynom& Polynom::pow(const Polynom& P_base, uint power) {
	if (power == 0)
		throw exception_OK;
	P_base.assert();
	size_t polynom_length = power * (P_base.length - 1) + 1;

	try {
		assert();
	} catch (...) {
		*this = Polynom(polynom_length);
	}

#ifdef ASSERT_DIMENSIONS
	if (length != polynom_length)
		throw exception_WRONG_DIMENSIONS;
#endif
	Polynom P_i0(1);
	P_i0[0] = 1.0;
	for (uint i = 0; i < power; i++) {
		Polynom P_i1 = P_i0.mul(P_base);
		P_i0 = P_i1;
	}
	*this = P_i0;
	return *this;
}

Polynom& Polynom::from_roots(const VectorComplex& roots) {

	roots.assert();
	size_t n_roots = roots.get_length();

	try {
		assert();
	} catch (...) {
		*this = Polynom(n_roots + 1);
	}

#ifdef ASSERT_DIMENSIONS
	if (length != n_roots + 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	VectorComplex polynom_i0_stage(1);
	polynom_i0_stage[0] = Complex(1.0, 0.0);

	for (uint i = 0; i < n_roots; i++) {
		if (isnan((double)roots[0].get_real()) || isnan((double)roots[0].get_imag()) || isinf((double)roots[0].get_real()) || isinf((double)roots[0].get_imag()))
			continue;
		Complex polynom_root_coeffs[2] = { -roots[i], Complex(1.0) };
		VectorComplex polynom_root(2, polynom_root_coeffs);
		VectorComplex polynom_i1_stage = polynom_i0_stage.convolute(polynom_root);
		polynom_i0_stage = polynom_i1_stage;
	}
	polynom_i0_stage.real_part(*this);
	return *this;
}
#ifdef USE_GSL
void Polynom::roots_GSL(VectorComplex& roots) const {

	assert();
	try {
		roots.assert();
	} catch (...) {
		roots = VectorComplex(length - 1);
	}

#ifdef ASSERT_DIMENSIONS
	if (roots.get_length() != length - 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc(length);
	if (w == NULL)
		throw exception_NULLPTR;
	exception_code returnval = exception_code(gsl_poly_complex_solve((double*) this->get_data_ptr(), length, w, (double*) roots.get_data_ptr()));
	gsl_poly_complex_workspace_free(w);

	if (returnval != exception_OK)
		throw returnval;

}
#endif

void Polynom::roots_DurandKerner(VectorComplex& roots, real_t space, real_t tolerance, uint max_iterations) const {

	assert();
	uint n_roots = length - 1;

	try {
		roots.assert();
	} catch (...) {
		roots = VectorComplex(length - 1);
	}

#ifdef ASSERT_DIMENSIONS
	if (n_roots != roots.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	for (uint n = 0; n < n_roots; n++) {
		roots[n] = Complex(RANDF(-space, space), RANDF(-space, space));
	}
	for (uint i = 0; i < max_iterations; i++) {
		bool found_all = true;
		for (uint n = 0; n < n_roots; n++) {
			Complex den(1, 0);
			for (uint k = 0; k < n_roots; k++) {
				if (k != n)
					den = den * (roots[n] - roots[k]);
			}
			Complex f_x = eval(roots[n]);
			Complex new_root = roots[n] - (f_x / den);
			if ((new_root - roots[n]).magnitude() > tolerance) {
				found_all = false;
				roots[n] = new_root;
			}
		}
		if (found_all)
			return;
	}

}

void Polynom::roots(VectorComplex& roots) const {
#ifdef USE_GSL
	roots_GSL(roots);
#else
	roots_DurandKerner(roots, DURAND_KERNER_DEFAULT_INITIAL_SPACE_SIZE, DURAND_KERNER_DEFAULT_TOLERANCE,DURAND_KERNER_DEFAULT_N_ITER);
#endif
}

VectorComplex Polynom::roots() const {
	VectorComplex roots_vector(length - 1);
	roots(roots_vector);
	return roots_vector;
}
