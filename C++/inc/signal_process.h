#ifndef SIGNAL_PROCESS_H_
#define SIGNAL_PROCESS_H_

#include "common_def.h"
#include "complex.h"
#include "vector.h"

namespace SystemControl {

template<typename T>
class VectorNumeric: public Vector<T> {
	using Vector<T>::Vector;
public:

	VectorNumeric() :
			Vector<T>() {
	}
	VectorNumeric(const Vector<T>& vectorSrc) :
			Vector<T>(vectorSrc) {
	}
	void convolute(const VectorNumeric&, const VectorNumeric&);
	VectorNumeric convolute(const VectorNumeric&) const;
	void convolute_symetric_kernel(const VectorNumeric&, const VectorNumeric&);
	VectorNumeric convolute_symetric_kernel(const VectorNumeric&) const;

	void filtrate(const VectorNumeric&, const VectorNumeric&);
	void filtrate_symetric_kernel(const VectorNumeric&, const VectorNumeric&);

	void reverse();
	T sum() const;
	void add(const VectorNumeric&, const VectorNumeric&);
	void add(const VectorNumeric&, T);
	void sub(const VectorNumeric&, const VectorNumeric&);
	void sub(const VectorNumeric&, T);
	void mul(const VectorNumeric&, const VectorNumeric&);
	void mul(const VectorNumeric&, T);
	void div(const VectorNumeric&, const VectorNumeric&);
	void div(const VectorNumeric&, T);

	inline VectorNumeric operator+(const VectorNumeric& B) const {
		const VectorNumeric& A = *this;
		VectorNumeric C(this->length);
		C.add(A, B);
		return C;
	}
	inline VectorNumeric operator-(const VectorNumeric& B) const {
		const VectorNumeric& A = *this;
		VectorNumeric C(this->length);
		C.sub(A, B);
		return C;
	}

private:

};

template<typename T>
void VectorNumeric<T>::reverse() {
	VectorNumeric& vec = *this;
	vec.assert();
	for (uint i = 0; i < vec.length / 2; i++) {
		T a = vec[i];
		T b = vec[vec.length - 1 - i];
		vec[i] = b;
		vec[vec.length - 1 - i] = a;
	}
}

template<typename T>
T VectorNumeric<T>::sum() const {
	T sum = 0;
	auto lambda = [&sum](auto A_i,auto i)->auto {sum +=A_i;return true;};
	Vector<T>::for_each(lambda);
	return sum;
}

template<typename T>
void VectorNumeric<T>::add(const VectorNumeric& A, const VectorNumeric& B) {
	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i)->auto {C_i=A_i+B_i; return true;};
	Vector<T>::for_each(A, B, lambda);
}

template<typename T>
void VectorNumeric<T>::add(const VectorNumeric& B, T value) {
	auto lambda = [value](auto& A_i,auto B_i,auto i)->auto {A_i=B_i+value;return true;};
	Vector<T>::for_each(B, lambda);
}

template<typename T>
void VectorNumeric<T>::sub(const VectorNumeric& A, const VectorNumeric& B) {
	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i)->auto {C_i=A_i-B_i;return true;};
	Vector<T>::for_each(A, B, lambda);
}

template<typename T>
void VectorNumeric<T>::sub(const VectorNumeric& B, T value) {
	auto lambda = [value](auto& A_i,auto B_i,auto i)->auto {A_i=B_i-value;return true;};
	Vector<T>::for_each(B, lambda);
}

template<typename T>
void VectorNumeric<T>::mul(const VectorNumeric& A, const VectorNumeric& B) {
	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i)->auto {C_i=A_i*B_i;return true;};
	Vector<T>::for_each(A, B, lambda);
}

template<typename T>
void VectorNumeric<T>::mul(const VectorNumeric& B, T value) {
	auto lambda = [value](auto& A_i,auto B_i,auto i)->auto {A_i=B_i*value;return true;};
	Vector<T>::for_each(B, lambda);
}

template<typename T>
void VectorNumeric<T>::div(const VectorNumeric& A, const VectorNumeric& B) {
	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i)->auto {C_i=A_i/B_i;return true;};
	Vector<T>::for_each(A, B, lambda);
}

template<typename T>
void VectorNumeric<T>::div(const VectorNumeric& B, T value) {
	auto lambda = [value](auto& A_i,auto B_i,auto i)->auto {A_i=B_i/value;return true;};
	Vector<T>::for_each(B, lambda);
}

template<typename T>
void VectorNumeric<T>::convolute(const VectorNumeric& f, const VectorNumeric& g) {

	if (this == &f || this == &g)
		throw exception_ERROR;
	f.assert();
	g.assert();

	int length_f = f.length;
	int length_g = g.length;
	int length_f_g = length_f + length_g - 1;

	try {
		this->assert();
	} catch (...) {
		*this = VectorNumeric(length_f_g);
	}
	VectorNumeric& f_g = *this;

#ifdef ASSERT_DIMENSIONS
	if (f_g.length != (size_t) length_f_g)
		throw exception_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < length_f_g; n++) {
		int kmin = (n >= length_g - 1) ? n - (length_g - 1) : 0;
		int kmax = (n < length_f - 1) ? n : length_f - 1;
		T sum(0);
		for (int k = kmin; k <= kmax; k++) {
			sum = sum + (f[k] * g[n - k]);
		}
		f_g[n] = sum;
	}
}
template<typename T>
VectorNumeric<T> VectorNumeric<T>::convolute(const VectorNumeric& g) const {
	const VectorNumeric& f = *this;
	VectorNumeric f_g(f.length + g.length - 1);
	f_g.convolute(f, g);
	return f_g;
}

template<typename T>
void VectorNumeric<T>::convolute_symetric_kernel(const VectorNumeric& f, const VectorNumeric& g) {
	exception_code returnval = exception_OK;

	if (this == &f || this == &g)
		throw exception_ERROR;

	f.assert();
	g.assert();

	int length_f = f.length;
	int length_g = g.length;
	int length_f_g = length_f + length_g - 1;

	try {
		this->assert();
	} catch (...) {
		*this = VectorNumeric(length_f_g);
	}
	VectorNumeric& f_g = *this;

#ifdef ASSERT_DIMENSIONS
	if (f_g.length != (size_t) length_f_g)
		throw exception_WRONG_DIMENSIONS;
	if (g.length % 2 == 0)
		throw exception_WRONG_DIMENSIONS;
#endif

	int M = (g.length - 1) / 2;
	for (int n = 0; n < length_f_g; n++) {
		T sum(0);
		for (int m = -M; m <= M; m++) {
			int f_idx = n - m;
			f_idx = CLIP_BOTTOM(f_idx, 0);
			f_idx = CLIP_TOP(f_idx, length_f - 1);
			int g_idx = m + M;
			g_idx = CLIP_BOTTOM(g_idx, 0);
			g_idx = CLIP_TOP(f_idx, length_g - 1);
			sum = sum + (f[f_idx] * g[g_idx]);
		}
		f_g[n] = sum;
	}
}

template<typename T>
void VectorNumeric<T>::filtrate(const VectorNumeric& vSrc, const VectorNumeric& vKernel) {

	if (this == &vSrc)
		throw exception_ERROR;

	vSrc.assert();
	vKernel.assert();

	int signal_length = vSrc.length;
	int kernel_length = vKernel.length;

	try {
		this->assert();
	} catch (...) {
		*this = VectorNumeric(signal_length);
	}
	VectorNumeric& vDst = *this;

#ifdef ASSERT_DIMENSIONS
	if (vDst.length != vSrc.length)
		throw exception_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < signal_length; n++) {
		T sum(0);
		for (int k = 0; k < kernel_length; k++) {
			int i = NEG_CLIP(n - kernel_length + 1 + k);
			sum += vSrc[i] * vKernel[k];
		}
		vDst[n] = sum;
	}

}

template<typename T>
void VectorNumeric<T>::filtrate_symetric_kernel(const VectorNumeric& vSrc, const VectorNumeric& vKernel) {

	if (this == &vSrc)
		throw exception_ERROR;

	vSrc.assert();
	vKernel.assert();

	int signal_length = vSrc.length;
	int kernel_length = (vKernel.length - 1) / 2;

	try {
		this->assert();
	} catch (...) {
		*this = VectorNumeric(signal_length);
	}
	VectorNumeric& vDst = *this;

#ifdef ASSERT_DIMENSIONS
	if (vKernel.length % 2 == 0)
		throw exception_WRONG_DIMENSIONS;
	if (vSrc.length != vDst.length)
		throw exception_WRONG_DIMENSIONS;
#endif

	for (int n = 0; n < signal_length; n++) {
		T sum(0);
		for (int k = -kernel_length; k <= kernel_length; k++) {
			int i = n + k;
			i = CLIP_BOTTOM(i, 0);
			i = CLIP_TOP(i, signal_length - 1);
			sum += vSrc[i] * vKernel[k + kernel_length];
		}
		vDst[n] = sum;
	}

}

class VectorReal: public VectorNumeric<real_t> {
	using VectorNumeric<real_t>::VectorNumeric;
public:
	VectorReal() :
			VectorNumeric<real_t>() {
	}

	real_t dot_product(const VectorReal&) const;
	void transform_linear(const VectorReal&, real_t, real_t);
	void diff(const VectorReal&);
	void integrate(const VectorReal&);

#ifdef USE_GSL
	gsl_vector to_gsl_vector();
	const gsl_vector to_gsl_vector() const;
	gsl_vector* to_gsl_vector_dynamic_copy() const;
	VectorReal(const gsl_vector&);
#endif

private:

};
class VectorComplex: public VectorNumeric<Complex> {
	using VectorNumeric<Complex>::VectorNumeric;
public:

	VectorComplex() :
			VectorNumeric<Complex>() {
	}
	void real_part(VectorReal&) const;
	VectorReal real_part() const;
	void imag_part(VectorReal&) const;
	VectorReal imag_part() const;
	void magnitude(VectorReal&) const;
	VectorReal magnitude() const;
	void phase(VectorReal&) const;
	VectorReal phase() const;

#ifdef USE_GSL
	gsl_vector_complex to_gsl_vector();
	const gsl_vector_complex to_gsl_vector() const;
	gsl_vector_complex* to_gsl_vector_dynamic_copy() const;
	VectorComplex(const gsl_vector_complex&);
#endif

private:
	template<typename F>
	void for_each_to_real(VectorReal&, F) const;

};

class SignalSampled: public VectorReal {
public:
	SignalSampled(size_t length, real_t Ts, real_t* data_ptr=NULL):VectorReal(length,data_ptr),Ts(Ts) {
	}
	inline real_t& at(real_t time)
	{
		return at_safe((uint) (time / Ts));
	}
private:
	real_t Ts;
};

class SignalTimeseries: public VectorReal {
public:
   SignalTimeseries(size_t length, real_t* values_ptr, real_t* times_ptr):values_vector(length,values_ptr),time_vector(length,times_ptr)
	{
	}
   	real_t read(real_t);
   	void write(real_t,real_t);
private:
	VectorReal values_vector;
	VectorReal time_vector;
	uint idx = 0;
};

}

#endif /* SIGNAL_PROCESS_H_ */

