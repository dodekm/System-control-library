#ifndef INC_COMPLEX_H_
#define INC_COMPLEX_H_

#include "common_def.h"

namespace SystemControl {
class Complex;
class Polar;

class Complex {
	friend class Polar;
public:

	inline Complex(real_t real = 0.0, real_t imag = 0.0) {
		this->real = real;
		this->imag = imag;
	}

	Complex(const class Polar&);
	inline real_t& get_real() {
		return real;
	}
	inline real_t& get_imag() {
		return imag;
	}

	inline real_t get_real() const {
		return real;
	}
	inline real_t get_imag() const {
		return imag;
	}
	inline Complex operator=(const Complex& B) {
		real = B.real;
		imag = B.imag;
		return *this;
	}
	inline bool operator==(const Complex& B) const {
		const Complex& A = *this;
		return (A.real == B.real && A.imag == B.imag);
	}
	inline Complex operator+(const Complex& B) const {
		const Complex& A = *this;
		return Complex(A.real + B.real, A.imag + B.imag);
	}
	inline Complex& operator+=(const Complex& B) {
		real += B.real;
		imag += B.imag;
		return *this;
	}
	inline Complex operator-(const Complex& B) const {
		const Complex& A = *this;
		return Complex(A.real - B.real, A.imag - B.imag);
	}

	inline Complex operator-() const {
		return Complex(-real, -imag);
	}
	inline Complex operator*(const Complex& B) const {
		const Complex& A = *this;
		return Complex(A.real * B.real - A.imag * B.imag, A.real * B.imag + A.imag * B.real);
	}

	inline Complex operator/(const Complex& B) const {
		const Complex& A = *this;
		real_t den = POW2(B.real) + POW2(B.imag);
		return Complex((A.real * B.real + A.imag * B.imag) / den, (A.imag * B.real - B.imag * A.real) / den);
	}
	inline Complex conj() const {
		return Complex(real, -imag);
	}
	inline real_t magnitude() const {
		return ::sqrt((double) (POW2(real) + POW2(imag)));
	}
	inline real_t magnitude_fast() const {
		static constexpr real_t alpha = 0.960433870103420;
		static constexpr real_t beta = 0.397824734759316;
		real_t a=abs(real);
		real_t b=abs(imag);
		return alpha * MAX(a,b ) + beta * MIN(a,b);
	}
	inline real_t phase() const {
		return ::atan2((double) imag, (double) real);
	}
	inline Complex exp() const {
		real_t real_exponential = ::exp((double) real);
		return Complex(real_exponential * ::cos((double) imag), real_exponential * ::sin((double) imag));
	}

	inline Complex log() const {
		return Complex(::log((double) magnitude()), phase());
	}

#ifdef USE_GSL
	inline Complex(const gsl_complex& C) {
		real = C.dat[0];
		imag = C.dat[1];
	}

	inline gsl_complex to_gsl_complex() const {
		gsl_complex C = {0};
		C.dat[0] = real;
		C.dat[1] = imag;
		return C;
	}

#endif

protected:
	real_t real;
	real_t imag;

};

class Polar {
	friend class Complex;
public:
	inline Polar(real_t mag, real_t phase) {
		this->mag = mag;
		this->phase = phase;
	}
	Polar(const class Complex&);

	inline real_t& get_mag() {
		return mag;
	}
	inline real_t& get_phase() {
		return phase;
	}

protected:
	real_t mag;
	real_t phase;

};

}
#endif /* INC_COMPLEX_H_ */
