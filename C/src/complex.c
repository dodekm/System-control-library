#include "complex.h"

/**
 * Generic complex number vector to vector element wise binary operation.
 * C(i)=f(A(i),B(i))
 * @param vector_complex_A source vector of complex numbers A
 * @param vector_complex_B source vector of complex numbers B
 * @param vector_complex_Dst destination vector of complex numbers C
 * @param operator pointer to operator function
 * @return system control library error code
 */
return_code vector_complex_to_complex_binary_operator(const vector_generic_T* vector_complex_A,const  vector_generic_T* vector_complex_B, vector_generic_T* vector_complex_Dst, complex_to_complex_binary_operator_T operator) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	returnval = vector_assert(vector_complex_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_complex_B);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_complex_Dst);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_complex_A) != vector_length(vector_complex_Dst) || vector_length(vector_complex_B) != vector_length(vector_complex_Dst))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_complex_A, complex_t) || !vector_is_type(vector_complex_B, complex_t) || !vector_is_type(vector_complex_Dst, complex_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_complex_A);
	for (uint i = 0; i < length; i++) {
		vector_type_at(vector_complex_Dst,complex_t, i) = operator(vector_type_at(vector_complex_A, complex_t, i), vector_type_at(vector_complex_B, complex_t, i));
	}
	return return_OK;
}

/**
 * Generic complex number vector element wise unary operation.
 * B(i)=f(A(i))
 * @param vector_complex_Src source vector of complex numbers A
 * @param vector_complex_Dst destination vector of complex numbers B
 * @param operator pointer to operator function
 * @return system control library error code
 */

return_code vector_complex_to_complex_unary_operator(const vector_generic_T* vector_complex_Src, vector_generic_T* vector_complex_Dst, complex_to_complex_unary_operator_T operator) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	returnval = vector_assert(vector_complex_Src);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_complex_Dst);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_complex_Src) != vector_length(vector_complex_Dst))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_complex_Src, complex_t) || !vector_is_type(vector_complex_Dst, complex_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_complex_Src);
	for (uint i = 0; i < length; i++) {
		vector_type_at(vector_complex_Dst,complex_t, i) = operator(vector_type_at(vector_complex_Src, complex_t, i));
	}
	return return_OK;
}


/**
 * Generic complex number vector to real number vector element wise binary operation.
 * @param vector_complex source vector of complex numbers
 * @param vector_real destination vector of real numbers
 * @param operator pointer to operator function
 * @return  system control library error code
 */
return_code vector_complex_to_real_unary_operator(const vector_generic_T* vector_complex, vector_generic_T* vector_real, complex_to_real_unary_operator_T operator) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	returnval = vector_assert(vector_complex);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_real);
	if (returnval != return_OK)
		return returnval;
	if (operator == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_complex) != vector_length(vector_real))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_complex, complex_t) || !vector_is_type(vector_real, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_complex);
	for (uint i = 0; i < length; i++) {
		vector_type_at(vector_real,real_t, i) = operator(vector_type_at(vector_complex, complex_t, i));
	}
	return return_OK;
}

/**
 * Converts vector of component represented complex numbers to vector of goniometric represented complex numbers.
 * @param vector_complex  source vector of component complex numbers
 * @param vector_polar destination vector of goniometric complex numbers
 * @return system control library error code
 */
return_code vector_complex_to_polar(const vector_generic_T* vector_complex, vector_generic_T* vector_polar) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_complex);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_polar);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_complex) != vector_length(vector_polar))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_complex);
	for (uint i = 0; i < length; i++) {
		vector_type_at(vector_polar,polar_t, i) = complex_to_polar(vector_type_at(vector_complex, complex_t, i));
	}
	return return_OK;
}

/**
 * Converts vector of goniometric represented complex numbers to vector of component represented complex numbers.
 * @param vector_polar source vector of goniometric complex numbers
 * @param vector_complex destination vector of component complex numbers
 * @return  system control library error code
 */
return_code vector_polar_to_complex(const vector_generic_T* vector_polar, vector_generic_T* vector_complex) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(vector_complex);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_polar);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_complex) != vector_length(vector_polar))
		return return_WRONG_DIMENSIONS;
	if (!vector_is_type(vector_complex, complex_t) || !vector_is_type(vector_polar, polar_t))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_complex);
	for (uint i = 0; i < length; i++) {
		vector_type_at(vector_complex,complex_t, i) = polar_to_complex(vector_type_at(vector_polar, polar_t, i));
	}
	return return_OK;
}
