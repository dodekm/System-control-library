#include "vector.h"

/**
 * Asserts/verifies vector structure integrity.
 * Checks:
 * - data pointer
 * - allocation info
 * - zero dimensions
 * @param vector vector to assert
 * @return system control library error code
 * - @ref return_NULLPTR if vector struct or data pointer is NULL
 * - @ref return_ERROR if allocation type is unallocated.
 * - @ref return_WRONG_DIMENSIONS if type_size or length is 0
 * - else  @ref return_OK
 */
return_code vector_assert(const vector_generic_T* vector) {
	if (vector == NULL)
		return return_NULLPTR;
	if (vector->data_ptr == NULL)
		return return_NULLPTR;
	if (vector->length == 0 || vector->type_size == 0)
		return return_WRONG_DIMENSIONS;
	if (vector->allocation_info == allocation_type_unallocated)
		return return_ERROR;
	return return_OK;
}

/**
 * Initializes generic vector structure.
 * Assigns data pointer and sets vector length and type size
 * @param vector vector structure pointer to be initialized
 * @param length number of elements - vector length
 * @param size size of vector data type in bytes
 * @param data_ptr pointer to vector data if static allocation or NULL if dynamic allocation
 * @return system control library error code
 */

return_code vector_generic_init(vector_generic_T* vector, size_t length, size_t size, void* data_ptr) {
#ifdef ASSERT_NULLPTR
	if (vector == NULL)
		return return_NULLPTR;
#endif

#ifdef ASSERT_DIMENSIONS
	if (length == 0 || size == 0)
		return return_WRONG_DIMENSIONS;
#endif

	if (vector->allocation_info != allocation_type_unallocated)
		return return_ERROR;
	if (data_ptr == NULL) {
		data_ptr = malloc(length * size);
		if (data_ptr == NULL)
			return return_NULLPTR;
		memset(data_ptr, 0, length * size);
		vector->allocation_info = allocation_type_dynamic;
	} else {
		vector->allocation_info = allocation_type_static;
	}
	vector->data_ptr = data_ptr;
	vector->length = length;
	vector->type_size = size;

	return return_OK;
}
/**
 * De-initializes vector structure.
 * First asserts vector, resets vector structure, if dynamic allocated frees memory
 * @param vector pointer to vector structure to be deinitialized
 * @return system control library error code
 */
return_code vector_generic_deinit(vector_generic_T* vector) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
	if (vector->allocation_info == allocation_type_dynamic) {
		free(vector->data_ptr);
	}
	vector->data_ptr = NULL;
	vector->length = 0;
	vector->type_size = 0;
	vector->allocation_info = allocation_type_unallocated;
	return return_OK;

}

/**
 * Copies vector data from source vector to destination vector using memcpy.
 * If vectors dimensions (length and type size) not agree - deinits and inits new vector dynamically
 * @param dst destination vector structure pointer
 * @param src source vector structure pointer
 * @return system control library error code
 */
return_code vector_generic_copy(vector_generic_T* dst, const vector_generic_T* src) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (dst == NULL)
		return return_NULLPTR;
	returnval = vector_assert(src);
	if (returnval != return_OK)
		return returnval;
#endif

	if (vector_assert(dst) != return_OK || vector_length(src) != vector_length(dst) || src->type_size != dst->type_size) {
		vector_generic_deinit(dst);
		returnval = vector_generic_init(dst, src->length, src->type_size, NULL);
		if (returnval != return_OK)
			return returnval;
	}
	return vector_generic_load(dst, src->data_ptr);

}

/**
 * Loads new data to vector.
 * Copies data from source pointer to vector data pointer address according to vector length and type size.
 * Asserts vector and source pointer
 * @param vector destination vector structure pointer
 * @param data_ptr source data pointer
 * @return system control library error code
 */
return_code vector_generic_load(vector_generic_T* vector,const void* data_ptr) {

#ifdef ASSERT_NULLPTR
	if (data_ptr == NULL)
		return return_NULLPTR;
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
	memcpy(vector->data_ptr, data_ptr, vector->length * vector->type_size);
	return return_OK;
}

/**
 * Extracts subvector form generic vector.
 * Subvector is statically allocated
 * @param vecDst subvector structure pointer
 * @param vecSrc origin vector structure pointer
 * @param length length of subvector
 * @param offset offset (position) of subvector in origin vector
 * @return system control library error code
 */
return_code vector_subvector(vector_generic_T* vecDst,const vector_generic_T* vecSrc, size_t length, size_t offset)
{
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vecSrc);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (length + offset > vecSrc->length)
		return return_WRONG_DIMENSIONS;
#endif
	vector_generic_init(vecDst,length,vecSrc->type_size,vecSrc->data_ptr+offset*vecSrc->type_size);

	return return_OK;

}

/**
 * Sets all elements of vector to certain value.
 * Copies data from input element pointer to all elements of vector  (vector can represent numeric type or structure).
 * Not efficient (uses memcpy) use @ref vector_type_set_all instead
 * @param vector destination vector structure pointer
 * @param data_ptr source data pointer to one vector element
 * @return system control library error code
 */

return_code vector_generic_set_all(vector_generic_T* vector,const void* data_ptr) {

#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
	if (data_ptr == NULL)
		return return_NULLPTR;
#endif
	size_t length = vector->length;
	for (uint i = 0; i < length; i++) {
		void* vector_data_ptr = vector->data_ptr + i * vector->type_size;
		memcpy(vector_data_ptr, data_ptr, vector->type_size);
	}
	return return_OK;
}


#define vector_type_set_value_safe_definition(T) \
return_code vector_##T_set_value_safe(vector_generic_T* vector, uint idx, T value) {\
	return_code returnval = vector_assert(vector);\
	if (returnval != return_OK)\
		return returnval;\
	if (idx >= vector->length)\
		return return_INDEX_OUT_OF_RANGE;\
	vector_type_at(vector,T,idx) = value;\
	return return_OK;\
}

#define vector_type_get_value_safe_definition(T)\
return_code vector_##T_get_value_safe(const vector_generic_T* vector, uint idx, T* value_ptr) {\
	return_code returnval = vector_assert(vector);\
	if (returnval != return_OK)\
		return returnval;\
	if (value_ptr == NULL)\
		return return_NULLPTR;\
	if (idx >= vector->length)\
		return return_INDEX_OUT_OF_RANGE;\
	(*value_ptr) = vector_type_at(vector,T, idx);\
	return return_OK;\
}

#define vector_type_set_all_definition(T)\
return_code vector_##T_set_all(vector_generic_T* vector, T value) {\
	return_code returnval = vector_assert(vector);\
	if (returnval != return_OK)\
		return returnval;\
	if (vector->type_size != sizeof(T))\
		return return_WRONG_DIMENSIONS;\
	T* data_ptr = (T*) vector->data_ptr;\
	size_t length = vector->length;\
	for (uint i = 0; i < length; i++) {\
		data_ptr[i] = value;\
	}\
	return return_OK;\
}

/**
 * Sets vector indexed item data using safety restrictions.
 * Asserts vector and checks index range - slower but safer
 * @param vector pointer to vector structure
 * @param idx accessed element index
 * @param value_ptr destination data pointer
 * @return
 *  -return_INDEX_OUT_OF_RANGE if index is out of range (length)
 *  -else return_OK
 */
vector_type_set_value_safe_definition(real_t)

/**
 * Gets vector indexed item data using safety restrictions.
 * Asserts vector and checks index range - slower but safer
 * @param vector pointer to vector structure
 * @param idx  accessed element index
 * @param value_ptr source data pointer
 * @return
 *  -return_INDEX_OUT_OF_RANGE if index is out of range (length)
 *  -else return_OK
 */
vector_type_get_value_safe_definition(real_t)

/**
 * Sets all elements of vector to certain value.
 * Writes value of input argument to all elements of vector  (vector can represent numeric type or structure)
 * Uses direct type assignment
 * @param vector destination vector structure pointer
 * @param numeric value or structure
 * @return system control library error code
 */
vector_type_set_all_definition(real_t)

