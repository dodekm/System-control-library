#include "signal_process.h"
#include "wrappers.h"

using namespace SystemControl;
using namespace Convert;

#ifdef USE_GSL
gsl_vector VectorReal::to_gsl_vector() {
	assert();
	return gsl_vector_view_array((double*) data_ptr, length).vector;
}

const gsl_vector VectorReal::to_gsl_vector() const {
	assert();
	return gsl_vector_const_view_array((const double*) data_ptr, length).vector;
}
gsl_vector* VectorReal::to_gsl_vector_dynamic_copy() const {
	gsl_vector gsl_vec_src = to_gsl_vector();
	gsl_vector* gsl_vec_dst = gsl_vector_alloc(length);
	if (gsl_vec_dst == NULL)
		throw exception_NULLPTR;
	gsl_vector_memcpy(gsl_vec_dst, &gsl_vec_src);
	return gsl_vec_dst;
}

VectorReal::VectorReal(const gsl_vector& gsl_vector) :
		VectorNumeric(gsl_vector.size, gsl_vector.data) {
}

#endif

real_t VectorReal::dot_product(const VectorReal& B) const {
	real_t dot_product = 0;
	const VectorReal& A = *this;
#ifdef USE_GSL
	const gsl_vector vector_A_gsl = A.to_gsl_vector();
	const gsl_vector vector_B_gsl = B.to_gsl_vector();
	exception_code returnval = gsl_error_code_to_return_code(gsl_blas_ddot(&vector_A_gsl, &vector_B_gsl, &dot_product));
	if (returnval != exception_OK)
		throw returnval;

#else
	auto lambda = [&dot_product](auto A_i,auto B_i,auto i)->auto {dot_product +=(A_i*B_i);return true;};
	A.for_each(B, lambda);
#endif

	return dot_product;
}

void VectorReal::transform_linear(const VectorReal& B, real_t k, real_t q) {

	auto lambda = [k,q](auto& A_i,auto B_i,auto i)->auto {A_i=Operators::transform_linear(B_i, k, q);return true;};
	for_each(B, lambda);
}
void VectorReal::diff(const VectorReal& B) {
	real_t B_i_last = B[0];
	auto lambda = [&B_i_last](auto& A_i,auto B_i,auto i)->auto {A_i=B_i-B_i_last;B_i_last=B_i;return true;};
	for_each(B, lambda);
}
void VectorReal::integrate(const VectorReal& B) {

	real_t sum = 0;
	auto lambda = [&sum](auto& A_i,auto B_i,auto i)->auto {sum+=B_i;A_i=sum;return true;};
	for_each(B, lambda);
}

template<typename F>
void VectorComplex::for_each_to_real(VectorReal& vReal, F function) const {
	const VectorComplex& vComplex = *this;

	vComplex.assert();
	vReal.assert();

#ifdef ASSERT_DIMENSIONS
	if (vComplex.length != vReal.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif
	for (uint i = 0; i < length; i++) {
		if (!function(vComplex[i], vReal[i]))
			break;
	}
}

void VectorComplex::real_part(VectorReal& vReal) const {

	auto lambda = [](const Complex& C,real_t& R)->auto {R=C.get_real();return true;};
	for_each_to_real(vReal, lambda);

}
VectorReal VectorComplex::real_part() const {
	VectorReal vReal(length);
	real_part(vReal);
	return vReal;
}

void VectorComplex::imag_part(VectorReal& vReal) const {

	auto lambda = [](const Complex& C,real_t& R)->auto {R=C.get_imag();return true;};
	for_each_to_real(vReal, lambda);
}

VectorReal VectorComplex::imag_part() const {
	VectorReal vReal(length);
	imag_part(vReal);
	return vReal;
}

void VectorComplex::magnitude(VectorReal& vReal) const {

	auto lambda = [](const Complex& C,real_t& R)->auto {R=C.magnitude();return true;};
	for_each_to_real(vReal, lambda);
}
VectorReal VectorComplex::magnitude() const {
	VectorReal vReal(length);
	magnitude(vReal);
	return vReal;
}

void VectorComplex::phase(VectorReal& vReal) const {

	auto lambda = [](const Complex& C,real_t& R)->auto {R=C.phase();return true;};
	for_each_to_real(vReal, lambda);
}

VectorReal VectorComplex::phase() const {
	VectorReal vReal(length);
	phase(vReal);
	return vReal;
}

#ifdef USE_GSL

gsl_vector_complex VectorComplex::to_gsl_vector() {
	assert();
	return gsl_vector_complex_view_array((double*) data_ptr, length).vector;
}

const gsl_vector_complex VectorComplex::to_gsl_vector() const {
	assert();
	return gsl_vector_complex_const_view_array((const double*) data_ptr, length).vector;
}

gsl_vector_complex* VectorComplex::to_gsl_vector_dynamic_copy() const {
	gsl_vector_complex gsl_vec_src = to_gsl_vector();
	gsl_vector_complex* gsl_vec_dst = gsl_vector_complex_alloc(length);
	if (gsl_vec_dst == NULL)
		throw(gsl_vector_complex*) NULL;
	gsl_vector_complex_memcpy(gsl_vec_dst, &gsl_vec_src);
	return gsl_vec_dst;

}

VectorComplex::VectorComplex(const gsl_vector_complex& gsl_vector) :
		VectorNumeric(gsl_vector.size, (Complex*) gsl_vector.data) {
}

#endif

real_t SignalTimeseries::read(real_t time) {

	time_vector.assert();
	values_vector.assert();

	while (1) {
		if (idx > 0) {
			if (time_vector[idx - 1] > time) {
				idx--;
				continue;
			}
		}
		if (idx < time_vector.get_length() - 1) {
			if (time_vector[idx + 1] <= time) {
				idx++;
				continue;
			}
		}
		break;
	}
	return values_vector[idx];

}
void SignalTimeseries::write(real_t time, real_t value) {

	time_vector.assert();
	values_vector.assert();

	if (time > time_vector[idx]) {
		if (idx + 1 >= time_vector.get_length())
			throw exception_INDEX_OUT_OF_RANGE;
		idx++;
	} else if (time < time_vector[idx]) {
		idx = 0;
	}
	time_vector[idx] = time;
	values_vector[idx] = value;
}

