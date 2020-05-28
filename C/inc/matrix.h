/**
 * @file  matrix.h
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "common_def.h"

/** @addtogroup matrix Matrix
 *  @brief Generic matrix interface and basic linear algebra.
 *
 * # Implemented functionality:
 * 	- element access
 * 	- initialization and de-initialization
 * 	- assert function
 * 	- copy and load functions
 * 	- selecting a sub-matrix
 * 	- linear algebra: matrix multiplication, element wise operations and scalar-matrix operations
 * 	- matrix power and exponential
 * 	- matrix transpose
 * 	- GSL and BLAS support
 * @{
 */

/**
 * Matrix instance structure.
 * Contains matrix dimensions and allocation information
 */
typedef struct matrix {
	size_t n_rows; ///<number of rows - first dimensions
	size_t n_cols; ///<number of columns - second dimensions
	size_t n_cols_mem; ///<memory layout - row length in memory
	allocation_type_enum allocation_info; ///<allocation information
	real_t* data_ptr; ///<data pointer to matrix elements
} matrix_T;

typedef struct matrix_2x2 {
	matrix_T;
	real_t data[2][2];
} matrix_2x2_T;

typedef struct matrix_3x3 {
	matrix_T;
	real_t data[3][3];
} matrix_3x3_T;

typedef struct matrix_4x4 {
	matrix_T;
	real_t data[4][4];
} matrix_4x4_T;

#define matrix_init_from_array(mat,array) matrix_init((mat),array_n_rows(array),array_n_cols(array),*(array)) ///< initializes matrix from static array
#define matrix_init_static_dim(mat) matrix_init_from_array((mat),((mat)->data)) ///< initializes static matrix structure containing static data @see matrix_4x4_T
#define matrix_at(mat,row,col) ((mat)->data_ptr)[(col)+(row)*(mat)->n_cols_mem] ///<direct indexed (row,col) access to matrix element (read and write)
#define matrix_ptr_at(mat,row,col)(&matrix_at(mat,row,col)) ///<gets indexed (row,col) address of matrix element (read and write)

#ifdef __cplusplus
extern "C" {
#endif

return_code matrix_assert(const matrix_T*);
return_code matrix_set_value_safe(matrix_T*, uint, uint, real_t);
return_code matrix_get_value_safe(const matrix_T*, uint, uint, real_t*);

return_code matrix_init(matrix_T*, size_t, size_t, real_t*);
return_code matrix_deinit(matrix_T*);
return_code matrix_copy(matrix_T*, const matrix_T*);
return_code matrix_set_all_elements(matrix_T*, real_t);
return_code matrix_set_identity(matrix_T*);
return_code matrix_load(matrix_T*, const real_t*);
return_code matrix_submatrix(matrix_T*, const matrix_T*, size_t, size_t, size_t, size_t);

return_code matrix_multiply(const matrix_T*, const matrix_T*, matrix_T*);
return_code matrix_element_wise_operation(const matrix_T*, const matrix_T*, matrix_T*, real_binary_operator_T);
return_code matrix_scalar_operation(const matrix_T*, matrix_T*, real_t, real_binary_operator_T);
return_code matrix_element_wise_operation_and_scalar_operation(const matrix_T*, const matrix_T*, matrix_T*, real_t, real_binary_operator_T, real_binary_operator_T);
return_code matrix_diagonal_operation(const matrix_T*, matrix_T*, real_t, real_binary_operator_T);

return_code matrix_transpose(matrix_T*);
return_code matrix_transpose_copy(const matrix_T*, matrix_T*);
return_code matrix_power(const matrix_T*, matrix_T*, uint);
return_code matrix_exp(const matrix_T*, matrix_T*, uint);

/**
 * Gets number of matrix rows.
 * @param mat matrix pointer
 * @return number of matrix rows
 */
static inline size_t matrix_get_n_rows(const matrix_T* mat) {
	return mat->n_rows;
}
/**
 * Gets number of matrix columns.
 * @param mat matrix pointer
 * @return number of matrix columns
 */
static inline size_t matrix_get_n_cols(const matrix_T* mat) {
	return mat->n_cols;
}
/**
 * Checks matrix is scalar.
 * Dimensions must be 1x1
 * @param mat matrix pointer
 * @return true(1) or false(0)
 */
static inline bool_t matrix_is_scalar(const matrix_T* mat) {
	return mat->n_rows == 1 && mat->n_cols == 1;
}

/**
 * Checks matrix is square matrix.
 * Dimensions must be nxn
 * @param mat matrix pointer
 * @return true(1) or false(0)
 */
static inline bool_t matrix_is_square(const matrix_T* mat) {
	return mat->n_rows == mat->n_cols;
}
/**
 * Checks matrix is column matrix.
 * Dimensions must be nx1
 * @param mat matrix pointer
 * @return true(1) or false(0)
 */

static inline bool_t matrix_is_col_vector(const matrix_T* mat) {
	return mat->n_cols == 1 && mat->n_rows > 1;
}
/**
 * Checks matrix is row matrix.
 * Dimensions must be 1xn
 * @param mat matrix pointer
 * @return true(1) or false(0)
 */
static inline bool_t matrix_is_row_vector(const matrix_T* mat) {
	return mat->n_rows == 1 && mat->n_cols > 1;
}
/**
 * Checks matrix A has same dimensions as matrix B.
 * @param matA matrix A pointer
 * @param matB matrix B pointer
 * @return true(1) or false(0)
 */
static inline bool_t matrix_equal_dimensions(const matrix_T* matA, matrix_T* matB) {
	return matA->n_rows == matB->n_rows && matA->n_cols == matB->n_cols;
}

#ifdef __cplusplus
}
#endif


/** @} */
#endif /* MATRIX_H_ */
