#include "gsl_wrapper.h"


/**
 * Converts - reshapes matrix with dimensions n_rows x n_cols to equivalent real number vector with length of n_rows*n_cols.
 * Creates shallow copy only - vector has the same data pointer as matrix
 * @param mat  system control library matrix structure
 * @return system control library generic vector structure
 */

vector_generic_T matrix_to_vector_real(const matrix_T* mat) {
	vector_generic_T vector = { 0 };
	vector_type_init(&vector, real_t, mat->n_rows * mat->n_cols, mat->data_ptr);
	return vector;
}


/**
 * Converts real vector to column matrix with dimensions n x 1.
 * Creates shallow copy only - matrix has the same data pointer as vector
 * @param vector real number vector length of n
 * @return resulting n x 1 matrix structure
 */
matrix_T vector_real_to_col_matrix(const vector_generic_T* vector) {
	matrix_T mat = { 0 };
	matrix_init(&mat, vector->length, 1, vector->data_ptr);
	return mat;

}

/**
 * Converts real vector to column matrix with dimensions 1 x n.
 * Creates shallow copy only - matrix has the same data pointer as vector
 * @param vector real number vector length of n
 * @return resulting 1 x n matrix structure
 */
matrix_T vector_real_to_row_matrix(const vector_generic_T* vector) {
	matrix_T mat = { 0 };
	matrix_init(&mat, 1, vector->length, vector->data_ptr);
	return mat;
}

#ifdef USE_GSL

/**
 * Converts system control library matrix structure to equivalent GSL matrix.
 * GSL matrix does not inherit allocation information (GSL matrix has static data). Memory layout of matrix is taken into account (tda/n_cols_mem)
 * Creates shallow copy only - GSL matrix has the same data pointer as the input matrix
 * @param mat system control library matrix structure
 * @return GSL matrix structure
 */
gsl_matrix matrix_to_gsl_matrix(const matrix_T* mat) {
	_gsl_matrix_view gsl_mat_view = gsl_matrix_view_array_with_tda((double*) mat->data_ptr, mat->n_rows, mat->n_cols, mat->n_cols_mem);
	return gsl_mat_view.matrix;
}
/**
 * Converts system control library matrix structure to equivalent GSL matrix by dynamic allocation and memcopy.
 * Creates full data copy -  GSL matrix has different data pointer from the input matrix and thus must be deallocated properly
 * @param mat system control library matrix structure
 * @return GSL matrix structure allocated pointer
 */
gsl_matrix* matrix_to_gsl_matrix_dynamic_copy(const matrix_T* mat) {
	gsl_matrix* gsl_mat_dst = gsl_matrix_alloc(mat->n_rows, mat->n_cols);
	if (gsl_mat_dst == NULL)
		return NULL;
	const gsl_matrix gsl_mat_src = matrix_to_gsl_matrix(mat);
	gsl_matrix_memcpy(gsl_mat_dst, &gsl_mat_src);
	return gsl_mat_dst;
}

/**
 * Converts GSL matrix structure to equivalent system control library matrix structure.
 * Returned matrix does not inherit allocation information and is set as static. Memory layout of matrix is taken into account (tda/n_cols_mem)
 * Creates shallow copy only - returned matrix has the same data pointer as the input GSL matrix
 * @param gsl_mat GSL matrix structure
 * @return system control library matrix structure
 */
matrix_T gsl_matrix_to_matrix(const gsl_matrix* gsl_mat) {
	matrix_T mat = { n_rows:gsl_mat->size1, n_cols:gsl_mat->size2, n_cols_mem:gsl_mat->tda, allocation_info:allocation_type_static, data_ptr:gsl_mat->data };
	return mat;
}
/**
 * Converts system control library real double vector to equivalent GSL vector.
 * GSL vector does not inherit allocation information (GSL vector has static data)
 * Creates shallow copy only - GSL vector has the same data pointer as the input vector
 * @param vector system control library vector structure
 * @return GSL vector structure
 */
gsl_vector vector_real_to_gsl_vector(const vector_generic_T* vector) {
	_gsl_vector_view gsl_vector_view = gsl_vector_view_array((double*) vector->data_ptr, vector->length);
	return gsl_vector_view.vector;
}
/**
 * Converts system control library real double vector structure to equivalent GSL vector by dynamic allocation and memcopy.
 * Creates full data copy -  GSL vector has different data pointer from the input vector and thus must be later deallocated properly
 * @param vector system control library vector structure
 * @return GSL vector structure allocated pointer
 */

gsl_vector* vector_real_to_gsl_vector_dynamic_copy(const vector_generic_T* vector) {
	gsl_vector* gsl_vec_dst = gsl_vector_alloc(vector->length);
	if (gsl_vec_dst == NULL)
		return NULL;
	gsl_vector gsl_vec_src = vector_real_to_gsl_vector(vector);
	gsl_vector_memcpy(gsl_vec_dst, &gsl_vec_src);
	return gsl_vec_dst;
}

/**
 * Converts GSL real double vector structure to equivalent system control library vector structure.
 * Returned vector does not inherit allocation information and is set as static.
 * Creates shallow copy only - returned vector has the same data pointer as the input GSL vector
 * @param gsl_vector GSL vector structure
 * @return system control library vector structure
 */
vector_generic_T gsl_vector_to_vector_real(const gsl_vector* gsl_vector) {
	vector_generic_T vector = { 0 };
	vector_type_init(&vector, real_t, gsl_vector->size, gsl_vector->data);
	return vector;
}

/**
 * @see vector_real_to_gsl_vector
 * @param vector system control library complex vector structure
 * @return GSL complex vector structure
 */
gsl_vector_complex vector_complex_to_gsl_vector_complex(const vector_generic_T* vector) {
	_gsl_vector_complex_view gsl_vector_view = gsl_vector_complex_view_array((double*) vector->data_ptr, vector->length);
	return gsl_vector_view.vector;
}
/**
 * @see gsl_vector_to_vector_real
 * @param gsl_vector  GSL complex vector structure
 * @return system control library complex vector structure
 */
vector_generic_T gsl_vector_complex_to_vector_complex(const gsl_vector_complex* gsl_vector) {
	vector_generic_T vector = { 0 };
	vector_type_init(&vector, complex_t, gsl_vector->size, gsl_vector->data);
	return vector;
}

/**
 * Converts system control matrix structure with dimensions n_rows x n_cols to equivalent GSL real double vector structure with length n_rows*n_cols.
 * GSL vector does not inherit allocation information (GSL vector has static data).
 * Creates shallow copy only - GSL vector has the same data pointer as the input matrix
 * @param mat system control library matrix structure
 * @return GLS vector structure
 */

gsl_vector matrix_to_gsl_vector(const matrix_T* mat) {
	_gsl_vector_view gsl_vector_view = gsl_vector_view_array((double*) mat->data_ptr, mat->n_cols * mat->n_rows);
	return gsl_vector_view.vector;
}

/**
 * Wrapper for GSL and system control library errorcode conversion.
 * Converts generic GSL library error code (integer returnval of GSL functions) to equivalent system control library errorcode enum.
 * @note Not all GSL returncodes are handled and compatible with system control library.
 * @param error GSL function return value
 * @return system control library errorcode
 */
return_code gsl_error_code_to_return_code(int error) {
	switch (error) {

	case GSL_SUCCESS:
		return return_OK;
	case GSL_ENOMEM:
		return return_NULLPTR;
	case GSL_EFAULT:
		return return_NULLPTR;
	case GSL_EBADLEN:
		return return_WRONG_DIMENSIONS;
	case GSL_ENOTSQR:
		return return_WRONG_DIMENSIONS;
	default:
		return return_ERROR;

	}
}
/**
 * Wrapper for system control library and GSL errorcode conversion.
 * Converts system control library errorcode enum to equivalent  generic GSL library error code (integer returnval of GSL functions).
 * @param error system control library errorcode
 * @return GSL integer errorcode
 */
int return_code_to_gsl_error_code(return_code error) {
	switch (error) {
	case return_OK:
		return GSL_SUCCESS;
	case return_ERROR:
		return GSL_FAILURE;
	case return_NULLPTR:
		return GSL_EFAULT;
	case return_WRONG_DIMENSIONS:
		return GSL_EBADLEN;
	default:
		return GSL_FAILURE;

	}
}
#endif

