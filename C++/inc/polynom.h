#ifndef POLYNOM_H_
#define POLYNOM_H_

#include "vector_numeric.h"

namespace SystemControl {

class Polynom: public VectorReal {

#define DURAND_KERNER_DEFAULT_N_ITER 100 ///<polynomial root find default number of iterations
#define DURAND_KERNER_DEFAULT_INITIAL_SPACE_SIZE 10.0 ///<polynomial root find default space size
#define DURAND_KERNER_DEFAULT_TOLERANCE 0.0001 ///<polynomial root find default convergence tolerance

	using VectorReal::VectorReal;

public:
	Polynom() {
	}
	Polynom(const VectorComplex& roots) :
			VectorReal(roots.get_length() + 1) {
		from_roots(roots);
	}

	real_t eval(real_t) const;
	Complex eval(Complex) const;

	Polynom& add(const Polynom&, const Polynom&);
	Polynom add(const Polynom&) const;
	inline Polynom operator+(const Polynom& B) const {
		return add(B);
	}
	Polynom& mul(const Polynom&, const Polynom&);
	Polynom mul(const Polynom&) const;
	inline Polynom operator*(const Polynom& B) const {
		return mul(B);
	}
	Polynom& pow(const Polynom&, uint);
	Polynom& from_roots(const VectorComplex&);
	void roots(VectorComplex&) const;
	VectorComplex roots() const;

private:
	template<typename T>
	T eval(T) const;
#ifdef USE_GSL
	void roots_GSL(VectorComplex&) const;
#endif
	void roots_DurandKerner(VectorComplex&, real_t = DURAND_KERNER_DEFAULT_INITIAL_SPACE_SIZE, real_t = DURAND_KERNER_DEFAULT_TOLERANCE, uint = DURAND_KERNER_DEFAULT_N_ITER) const;
};

}

#endif /* POLYNOM_H_ */
