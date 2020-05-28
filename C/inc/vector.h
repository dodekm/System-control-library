/**
 * @file vector.h
 */

#ifndef VECTOR_H_
#define VECTOR_H_

/** @addtogroup vector Vectors
 *  @brief Generic vector interface.
 *
 * # Implemented functionality
 * Common generic vector functions and macros:
 * 	- initialization and de-initialization of vector (static and dynamic)
 * 	- vector assert function
 * 	- vectors copy
 * 	- fast and safe vector element access
 *
 * # Templates
 * This file uses macro templates for type specific vector functions:
 * vector_type_set_value_safe
 * vector_type_get_value_safe
 * vector_type_set_all_declaration
 * For every specific type they need to be declared and defined by template macros:
 *
 * 	  vector_type_set_value_safe_declaration(T)
 * 	  vector_type_get_value_safe_declaration(T)
 * 	  vector_type_set_all_declaration(T)
 *
 * 	  vector_type_set_value_safe_definition(T)
 * 	  vector_type_get_value_safe_definition(T)
 * 	  vector_type_set_all_definition(T)
 *
 * @{
 */

#include "common_def.h"
/**
 * Generic vector structure.
 * Vector is universal data type - uses void* data pointer
 * Vector can be static or dynamic allocated
 */
typedef struct vector_generic {
	void* data_ptr; ///<generic pointer to data
	size_t length; ///<vector length - number of elements
	allocation_type_enum allocation_info; ///<vector data allocation type - static,dynamic or unallocated
	size_t type_size; ///<size of vector assigned type in bytes - equivalent to sizeof operator
} vector_generic_T;

#define vector_length(vector) ((vector)->length) ///<gets number of elements
#define vector_generic_get_data_ptr(vector) ((vector)->data_ptr) ///<gets vector data address/pointer
#define vector_type_init(vector,type,length,data_ptr) vector_generic_init((vector),(length),sizeof(type),(void*)(data_ptr)) ///<initializes vector according to data type (calls @ref vector_generic_init) with length and data pointer
#define vector_init_from_array(vector,array) vector_generic_init(vector,array_length(array),sizeof(array[0]),(void*)(array))///<initializes vector directly from array (static allocation)
#define vector_type_load(vector,data_ptr) vector_generic_load((vector),(const void*)(data_ptr)) ///<calls vector_generic_load with (void*) typecast
#define vector_type_data_ptr(vector,type) ((type*)((vector)->data_ptr)) ///<gets vector data pointer
#define vector_type_at(vector,type,i) (((type*)((vector)->data_ptr))[i])///<access vector element of type indexed by i, allows read/write operations
#define vector_is_type(vector,type) ((vector)->type_size==sizeof(type)) ///<checks vector type by type_size and sizeof operator

#ifdef __cplusplus
extern "C" {
#endif

return_code vector_assert(const vector_generic_T*);
return_code vector_generic_init(vector_generic_T*, size_t, size_t, void*);
return_code vector_generic_deinit(vector_generic_T*);
return_code vector_generic_copy(vector_generic_T*, const vector_generic_T*);
return_code vector_generic_load(vector_generic_T*, const void*);
return_code vector_subvector(vector_generic_T*, const vector_generic_T*, size_t, size_t);
return_code vector_generic_set_all(vector_generic_T*, const void*);

#define vector_type_set_value_safe_declaration(T) \
return_code vector_##T_set_value_safe(vector_generic_T*,uint,T);

#define vector_type_get_value_safe_declaration(T)\
return_code vector_##T_get_value_safe(const vector_generic_T*,uint,T*);

#define vector_type_set_value_safe(vector,T,idx,value) vector_##T_set_value_safe(vector,idx,value); ///<Sets vector indexed item data of type T from value_ptr using safety restrictions. Asserts vector and checks index range  Macro is type specific template function call.
#define vector_type_get_value_safe(vector,T,idx,value_ptr) vector_##T_get_value_safe(vector,idx,value_ptr); ///<Gets vector indexed item data of type T to value_ptr using safety restrictions. Asserts vector and checks index range  Macro is type specific template function call.

#define vector_type_set_all_declaration(T) \
return_code vector_##T_set_all(vector_generic_T*,T);

#define vector_type_set_all(vector,T,value) vector_##T_set_all(vector,value); ///<Sets all elements of vector to certain value of type T. Macro is type specific template function call. More efficient than @ref vector_generic_set_all.

vector_type_set_value_safe_declaration(real_t)
vector_type_get_value_safe_declaration(real_t)
vector_type_set_all_declaration(real_t)

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* VECTOR_H_ */
