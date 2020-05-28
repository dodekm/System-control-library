#include "complex.h"

using namespace SystemControl;

Complex::Complex(const Polar& P) {
	real = P.mag * ::cos((double)P.phase);
	imag = P.mag * ::sin((double)P.phase);
}
Polar::Polar(const Complex& C) {
	mag = C.magnitude();
	phase = C.phase();
}

